
#ifndef INCLUDED_TC_82545_H
#define INCLUDED_TC_82545_H

#define ERR_UNKNOWN_MAC_TYPE -254
#define I82545_MAC_TYPE      6

#define INTEL_82545_ETH_DEVICE( device_id ) { \
    PCI_DEVICE( PCI_VENDOR_ID_INTEL,device_id ) }

struct i82545_adapter {
    struct net_device *netdev;
    struct pci_device *pdev;

    struct i82545_hw hw;

    uint16_t link_speed;
    uint16_t link_duplex;
    int msg_enable;
    int bars;
};

#endif
