
#include "i82545_hw.h"

void i82545_get_bus_info( struct i82545_hw *hw )
{
    uint32_t status;

    status = er32(STATUS);
    hw->bus_type = (status & I82545_STATUS_PCIX_MODE) ? i82545_bus_type_pcix : i82545_bus_type_pci;

    if (hw->bus_type == i82545_bus_type_pci)
        hw->bus_speed = (status & I82545_STATUS_PCI66) ? i82545_bus_speed_66 : i82545_bus_speed_33;
    else {
        switch (status & I82545_STATUS_PCIX_SPEED) {
            case I82545_STATUS_PCIX_SPEED_66:
                hw->bus_speed = i82545_bus_speed_66;
                break;
            case I82545_STATUS_PCIX_SPEED_100:
                hw->bus_speed = i82545_bus_speed_100;
                break;
            case I82545_STATUS_PCIX_SPEED_133:
                hw->bus_speed = i82545_bus_speed_133;
                break;
            default:
                hw->bus_speed = i82545_bus_speed_reserved;
                break;
        }
    }

    hw->bus_width = (status & E1000_STATUS_BUS64) ? e1000_bus_width_64 : e1000_bus_width_32;
}
