
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>

#include "i82545_hw.h"
#include "i82545.h"
#include "dbg.h"

static const char drv_name[] = "tc-i82545-drv";

static const struct pci_device_id i82545_pci_tbl[] = {
    INTEL_82545_ETH_DEVICE( 0x100f ),
    { 0 }
};
MODULE_DEVICE_TABLE( pci,i82545_pci_tbl );

/**
 * i82545_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog task is started,
 * and the stack is notified that the interface is ready.
 **/
int i82545_open( struct net_device *netdev )
{
    struct i82545_adapter *adapter = netdev_priv( netdev );
    struct i82545_hw *hw = &adapter->hw;
    ENTER;

    if( test_bit( __I82545_TESTING,&adapter->flags ) )
        return -EBUSY;

    netif_carrier_off( netdev );

    if( (err = i82545_setup_all_tx_resources( adapter )) )
        goto err_tx_setup;

    if( (err = i82545_setup_all_rx_resources( adapter )) )
        goto err_rx_setup;

    i82545_power_up_phy( adapter );

    adapter->mng_vlan_id = I82545_MNG_VLAN_NONE;
    if( (hw->mng_cookie.status & I82545_MNG_DHCP_COOKIE_STATUS_VLAN_SUPPORT ))
        i82545_update_mng_vlan( adapter );

    /* before we allocate an interrupt, we must be ready to handle it.
     * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
     * as soon as we call pci_request_irq, so we have to setup our
     * clean_rx handler before we do so.
     */
    i82545_configure( adapter );

    if( (err = i82545_request_irq( adapter )) )
        goto err_req_irq;

    clear_bit( __I82545_DOWN,&adapter->flags );
    napi_enable( &adapter->napi );
    i82545_irq_enable( adapter );
    netif_start_queue( netdev );

    /* fire a link status change interrupt to start the watchdog */
    ew32( ICS,I82545_ICS_LSC );

    EXIT;

err_req_irq:
    i82545_power_down_phy( adapter );
    i82545_free_all_rx_resources( adapter );
err_rx_setup:
    i82545_free_all_tx_resources( adapter );
err_tx_setup:
    i82545_reset( adapter );

    EXIT;
}

/**
 * i82545_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
int i82545_close( struct net_device *netdev )
{
    struct i82545_adapter *adapter = netdev_priv( netdev );
    struct i82545_hw *hw = &adapter->hw;
    int count = I82545_CHECK_RESET_COUNT;
    ENTER;

    while( test_bit( __I82545_RESETTING,&adapter->flags ) && count-- )
        usleep_range( 10000,20000 );

    WARN_ON( test_bit( __I82545_RESETTING,&adapter->flags ) );
    i82545_down( adapter );
    i82545_power_down_phy( adapter );
    i82545_free_irq( adapter );

    i82545_free_all_tx_resources( adapter );
    i82545_free_all_rx_resources( adapter );

    /* kill manageability vlan ID if supported, but not if a vlan with
     * the same ID is registered on the host OS (let 8021q kill it)
     */
    if( ( hw->mng_cookie.status & I82545_MNG_DHCP_COOKIE_STATUS_VLAN_SUPPORT ) &&
        !test_bit( adapter->mng_vlan_id,adapter->active_vlans ) ) {
        i82545_vlan_rx_kill_vid( netdev,htons( ETH_P_8021Q ),adapter->mng_vlan_id );
    }

    EXIT;
}

#define TXD_USE_COUNT(S, X) (((S) + ((1 << (X)) - 1)) >> (X))
static netdev_tx_t i82545_xmit_frame( struct sk_buff *skb,struct net_device *netdev )
{
    struct i82545_adapter *adapter = netdev_priv(netdev);
    struct i82545_hw *hw = &adapter->hw;
    struct i82545_tx_ring *tx_ring;
    unsigned int first, max_per_txd = I82545_MAX_DATA_PER_TXD;
    unsigned int max_txd_pwr = I82545_MAX_TXD_PWR;
    unsigned int tx_flags = 0;
    unsigned int len = skb_headlen(skb);
    unsigned int nr_frags;
    unsigned int mss;
    int count = 0;
    int tso;
    unsigned int f;
    __be16 protocol = vlan_get_protocol(skb);
    ENTER;

    tx_ring = adapter->tx_ring;

    /* On PCI/PCI-X HW, if packet size is less than ETH_ZLEN,
    * packets may get corrupted during padding by HW.
    * To work around this issue, pad all small packets manually.
    */
    if( eth_skb_pad( skb ) )
        EXITRC( NETDEV_TX_OK );

    mss = skb_shinfo( skb )->gso_size;
    /* reserve a descriptor for the offload context */
    if( mss || skb->ip_summed == CHECKSUM_PARTIAL )
        count++;
    count++;

    /* Controller Erratum workaround */
    if( !skb->data_len && tx_ring->last_tx_tso && !skb_is_gso( skb ) )
        count++;

    count += TXD_USE_COUNT( len,max_txd_pwr );

    /* work-around for errata 10 and it applies to all controllers
    * in PCI-X mode, so add one more descriptor to the count
    */
    if( unlikely( hw->bus_type == i82545_bus_type_pcix && len > 2015 ) )
        count++;

    nr_frags = skb_shinfo( skb )->nr_frags;
    for( f=0; f < nr_frags; f++ )
        count += TXD_USE_COUNT( skb_frag_size( &skb_shinfo( skb )->frags[f] ),max_txd_pwr );

    /* need count + 2 desc gap to keep tail from touching
    * head, otherwise try next time
    */
    if( unlikely( i82545_maybe_stop_tx( netdev,tx_ring,count + 2 ) ) )
        EXITRC( NETDEV_TX_BUSY );

    if( skb_vlan_tag_present( skb ) ) {
        tx_flags |= I82545_TX_FLAGS_VLAN;
        tx_flags |= (skb_vlan_tag_get( skb ) << I82545_TX_FLAGS_VLAN_SHIFT );
    }

    first = tx_ring->next_to_use;
    tso   = i82545_tso( adapter,tx_ring,skb,protocol );
    if( tso < 0 ) {
        dev_kfree_skb_any( skb );
        EXITRC( NETDEV_TX_OK );
    }

    if( likely( tso ) ) {
        tx_ring->last_tx_tso = true;
        tx_flags             = I82545_TX_FLAGS_TSO;
    }
    else if( likely( i82545_tx_csum( adapter,tx_ring,skb,protocol ) ) )
        tx_flags |= I82545_TX_FLAGS_CSUM;

    if( protocol == htons( ETH_P_IP ) )
        tx_flags |= I82545_TX_FLAGS_IPV4;
    if( unlikely( skb->no_fcs ) )
        tx_flags |= I82545_TX_FLAGS_NO_FCS;

    count = i82545_tx_map( adapter,tx_ring,skb,first,max_per_txd,nr_frags,mss );
    if( count ) {
        /* The descriptors needed is higher than other Intel drivers
        * due to a number of workarounds.  The breakdown is below:
        * Data descriptors: MAX_SKB_FRAGS + 1
        * Context Descriptor: 1
        * Keep head from touching tail: 2
        * Workarounds: 3
        */
        int desc_needed = MAX_SKB_FRAGS + 7;

        netdev_sent_queue( netdev,skb->len );
        skb_tx_timestamp( skb );

        i82545_tx_queue( adapter,tx_ring,tx_flags,count );
        i82545_maybe_stop_tx( netdev,tx_ring,desc_needed );

        if( !netdev_xmit_more() ||
            netif_xmit_stopped( netdev_get_tx_queue( netdev,0 ) ) )
            writel( tx_ring->next_to_use,hw->hw_addr + tx_ring->tdt );
    }
    else {
        dev_kfree_skb_any( skb );
        tx_ring->buffer_info[first].time_stamp = 0;
        tx_ring->next_to_use = first;
    }

    EXITRC( NETDEX_TX_OK );
}

static void i82545_set_rx_mode( struct net_device *netdev )
{
    struct i82545_adapter *adapter = netdev_priv( netdev );
    struct i82545_hw *hw = &adapter->hw;
    struct netdev_hw_addr *ha;
    uint32_t *mcarray;
    bool use_uc = false;
    uint32_t rctl;
    int rar_entries = I82545_RAR_ENTRIES;
    int i = 1;
    ENTER;

    if( (mcarray = kcalloc( I82545_NUM_MTA_REGISTERS,sizeof( uint32_t ),GFP_ATOMIC )) == NULL )
        EXITRC( -NOMEM );

    rctl = er32( RCTL );

    if( netdev->flags & IFF_PROMISC ) {
        rctl |= (I82545_RCTL_UPE | I82545_RCTL_MPE);
        rctl &= ~I82545_RCTL_VFE;
    }
    else {
        if( netdev->flags & IFF_ALLMULTI )
            rctl |= I82545_RCTL_MPE;
        else
            rctl &= ~I82545_RCTL_MPE;
        /* Enable VLAN filter if there is a VLAN */
        if( i82545_vlan_used( adapter ) )
            rctl |= I82545_RCTL_VFE;
    }

    if( netdev_uc_count( netdev ) > rar_entries - 1 )
        rctl |= I82545_RCTL_UPE;
    else if( !(netdev->flags & IFF_PROMISC) ) {
        rctl &= ~I82545_RCTL_UPE;
        use_rc = true;
    }

    ew32( RCTL,rctl );

    if( use_uc ) {
        netdev_for_each_uc_addr( ha,netdev ) {
            if( i == rar_entries )
                break;
            i82545_rar_set( hw,ha->addr,i++ );
        }
    }

    netdev_for_each_mc_addr( ha,netdev ) {
        if( i == rar_entries ) {
            uint32_t hash_value, hash_reg, hash_bit, mta;

            hash_value = i82545_hash_mc_addr( hw,ha->addr );
            hash_reg   = (hash_value >> 5) & 0x7F;
            hash_bit   = hash_value & 0x1F;
            mta        = (1 << hash_bit);
            mcarray[hash_reg] |= mta;
        }
        else
            i82545_rar_set( hw,ha->addr,i++ );
    }

    for( ; i < rar_entries; i++ ) {
        I82545_WRITE_REG_ARRAY( hw,RA,i << 1,0 );
        I82545_WRITE_FLUSH();
        I82545_WRITE_REG_ARRAY( hw,RA,(i << 1) + 1,0 );
        I82545_WRITE_FLUSH();
    }

    for( i = mta_reg_count - 1; i >= 0; i-- )
        I82545_WRITE_REG_ARRAY( hw,MTA,i,mcarray[i] );
    I82545_WRITE_FLUSH();

    kfree( mcarray );
}

static const struct net_device_ops i82545_netdev_ops = {
    .ndo_open               = i82545_open,
    .ndo_stop               = i82545_close,
    .ndo_start_xmit         = i82545_xmit_frame,
    .ndo_set_rx_mode        = i82545_set_rx_mode,
    .ndo_set_mac_address    = i82545_set_mac,
    .ndo_tx_timeout         = i82545_tx_timeout,
    .ndo_change_mtu         = i82545_change_mtu,
    .ndo_do_ioctl           = i82545_ioctl,
    .ndo_validate_addr      = eth_validate_addr,
    .ndo_vlan_rx_add_vid    = i82545_vlan_rx_add_vid,
    .ndo_vlan_rx_kill_vid   = i82545_vlan_rx_kill_vid,
#ifdef CONFIG_NET_POLL_CONTROLLER
    .ndo_poll_controller    = i82545_netpoll,
#endif
    .ndo_fix_features       = i82545_fix_features,
    .ndo_set_features       = i82545_set_features,
};

static int i82545_hw_init_struct( struct i82545_adapter *adapter,
                                  struct i82545_hw *hw )
{
    struct pci_dev *pdev = adapter->pdev;
    ENTRY

    hw->vendor_id           = pdev->vendor;
    hw->device_id           = pdev->device;
    hw->subsystem_vendor_id = pdev->subsystem_vendor;
    hw->subsystem_id        = pdev->subsystem_device;
    hw->revision_id         = pdev->revision;

    pci_read_config_word( pdev,PCI_COMMAND,&hw->pci_cmd_word );

    hw->max_frame_size = adapter->netdev->mtu + ENET_HEADER_SIZE + ETHERNET_FCS_SIZE;
    hw->min_frame_size = MINIMUM_ETHERNET_FRAME_SIZE;

    hw->mac_type = hw->device_id == i82545_DEV_ID_82545_COPPER ? MAC_TYPE_82545EM : MAC_TYPE_UNKNOWN;
    if( hw->mac_type == MAC_TYPE_UNKNOWN ) {
        pr_err( "MAC Type Unknown" );
        ret = -EIO;
    }

    hw->tbi_compatibility_en = false;
    hw->media_type           = i82545_media_type_copper;

    EXIT
}

static int i82545_probe( struct pci_dev *pdev,const struct pci_device_id *ent )
{
    struct net_device *netdev;
    struct i82545_adapter *adapter;
    struct i82545_hw *hw;
    int bars,err,i;
    ENTRY

    bars = pci_select_bars( pdev,IORESOURCE_MEM | IORESOURCE_IO );
    if( (err = pci_enable_device( pdev )) )
        goto err_pci_enable_device;

    if( (err = pci_request_selected_regions( pdev,bars,drv_name )) )
        goto err_pci_select_regions;

    pci_set_master( pdev );
    if( (err = pci_save_state( pdev )) )
        goto err_pci_alloc_etherdev;

    if( (netdev = alloc_etherdev( sizeof( struct i82545_adapter ) ) ) == NULL )
        goto err_pci_alloc_ether_dev;
    
    SET_NETDEV_DEV( netdev,&pdev->dev );
    pci_set_drvdata( pdev,netdev );

    adapter = netdev_priv( netdev );
    adapter->netdev = netdev;
    adapter->pdev   = pdev;
    adapter->bars   = bars;

    hw = &adapter->hw;
    hw->back = adapter;

    ret = -EIO;
    if( (hw->hw_addr = pci_ioremap_bar( pdev,BAR_0 )) == NULL )
        goto err_ioremap;

    for( i=BAR_1; i < PCI_STD_NUM_BARS; i++ ) {
        if( pci_resource_len( pdev,i ) == 0 )
            continue;
        if( pci_resource_flags( pdev,i ) & IORESOURCE_IO ) {
            hw->io_base = pci_resource_start( pdev,i );
            break;
        }
    }

    if( (err = i82545_init_hw_struct( adapter,hw )) )
        goto err_sw_init;

    if( !(hw->bus_type == i82545_bus_type_pcix && !dma_set_mask_and_coherent( &pdev->dev,DMA_BITMASK( 64 ) ) ) ) {
        pr_err( "No usable DMA config, aborting\n" );
        goto err_dma;
    }

    netdev->netdev_ops = &i82545_netdev_ops;
    i82545_set_ethtool_ops( netdev );
    netdev->watchdog_timeo = 5 * HZ;
    netif_napi_add( netdev,&adapter->napi,i82545_clean,64 );

    strncpy( netdev->name,pci_name( pdev ),sizeof( netdev->name ) - 1 );
    adapter->bd_number = 1; /*cards_found;*/

    if( (err = i82545_sw_init( adapter )) )
        goto err_sw_init;

    err = -EIO;
    netdev->hw_features = NETIF_F_SG | NETIF_F_HW_CSUM | NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_TSO;
    netdev->features    = NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_FILTER;
    netdev->priv_flags  |= IFF_SUPP_NOFCS;
    netdev->features    |= netdev->hw_features;
    netdev->hw_features |= (NETIF_F_RXCSUM | NETIF_F_RXALL | NETIF_F_RXFCS);
    netdev->features    |= NETIF_F_HIGHDMA;
    netdev->vlan_features |= NETIF_F_HIGHDMA;
    netdev->vlan_features |= (NETIF_F_TSO | NETIF_F_HW_CSUM | NETIF_F_SG);

    netdev->min_mtu = ETH_ZLEN - ETH_HLEN;
    netdev->max_mtu = MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + ETH_FCS_LEN);

    adapter->en_mng_pt = i82545_enable_mng_pass_thru( hw );

    if( i82545_init_eeprom_params( hw ) ) {
        e_err( probe,"EEPROM Initialization Failed\n" );
        goto err_eeprom;
    }

    /* before reading the EEPROM, reset the controller to
     * put the device in a known good starting state
     */
    i82545_reset_hw(hw);

    if( i82545_validate_eeprom_checksum( hw ) < 0 ) {
        e_err( probe,"The EEPROM Checksum Is Not Valid\n" );

        /* set MAC address to all zeroes to invalidate and temporary
         * disable this device for the user. This blocks regular
         * traffic while still permitting ethtool ioctls from reaching
         * the hardware as well as allowing the user to run the
         * interface after manually setting a hw addr using
         * `ip set address`
         */
        memset( hw->mac_addr,0,netdev->addr_len );
    }
    else {
        /* copy the MAC address out of the EEPROM */
        if( i82545_read_mac_addr( hw ) )
            e_err(probe, "EEPROM Read Error\n");
    }

    memcpy( netdev->dev_addr,hw->mac_addr,netdev->addr_len );

    if( !is_valid_ether_addr( netdev->dev_addr ) )
        e_err( probe,"Invalid MAC Address\n" );

    INIT_DELAYED_WORK( &adapter->watchdog_task,i82545_watchdog );
    INIT_DELAYED_WORK( &adapter->phy_info_task,i82545_update_phy_info_task );
    INIT_WORK( &adapter->reset_task,i82545_reset_task );

    i82545_check_options( adapter );
    i82545_reset_hw(hw);
    strcpy( netdev->name,"eth%d" );
    if( (err = register_netdev( netdev )) )
        goto err_register;

    i82545_vlan_filter_on_off( adapter,false );

   e_info(probe, "(PCI%s:%dMHz:%d-bit) %pM\n",
       ((hw->bus_type == i82545_bus_type_pcix) ? "-X" : ""),
       ((hw->bus_speed == i82545_bus_speed_133) ? 133 :
       (hw->bus_speed == i82545_bus_speed_120) ? 120 :
       (hw->bus_speed == i82545_bus_speed_100) ? 100 :
        (hw->bus_speed == i82545_bus_speed_66) ? 66 : 33),
       ((hw->bus_width == i82545_bus_width_64) ? 64 : 32),
       netdev->dev_addr);

   netif_carrier_off( netdev );
   e_info( probe,"Intel(R) i82545 Network Adapter\n" );

    ret = 0;
    EXIT

err_register:
err_eeprom:
    i82545_phy_hw_reset( hw );

err_dma:
err_sw_init:
    iounmap( hw->ce4100_gbe_mdio_base_virt );
    iounmap( hw->hw_addr );
err_ioremap:
    free_netdev( netdev );
err_pci_alloc_etherdev:
    pci_release_select_regions( pdev,bars );
err_pci_selected_regions:
    pci_disable_device( pdev );
err_pci_enable_device:
    EXIT
}

static void i82545_remove( struct pci_dev *pdev )
{
    ENTRYV
    EXITV
}

static void i82545_shutdown( struct pci_dev *pdev )
{
    ENTRYV
    EXITV
}

static struct pci_driver i82545_driver = {
    .name     = drv_name,
    .id_table = i82545_pci_tbl,
    .probe    = i82545_probe,
    .remove   = i82545_remove,
    .shutdown = i82545_shutdown
};

static int __init i82545_module_init( void )
{
    ENTRY

    ret = pci_register_driver( &i82545_driver );

    EXIT
}

static void __exit i82545_module_exit( void )
{
    ENTRYV

    pci_unregister_driver( &i82545_driver );

    EXITV
}

module_init( i82545_module_init );
module_exit( i82545_module_exit );
MODULE_LICENSE( "GPL v2" );
