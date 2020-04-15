
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

    ret = 0;
    EXIT

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
