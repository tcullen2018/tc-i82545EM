
#ifndef INCLUDED_i82545_HW_H
#define INCLUDED_i82545_HW_H

#include "i82545_osdep.h"

#define i82545_DEV_ID_82545EM_COPPER 0x100f

#define MAC_TYPE_82545   6
#define MAC_TYPE_UNKNOWN 0

/* PCI bus types */
typedef enum {
    i82545_bus_type_unknown = 0,
    i82545_bus_type_pci,
    i82545_bus_type_pcix,
    i82545_bus_type_reserved
} i82545_bus_type;

/* PCI bus speeds */
typedef enum {
    i82545_bus_speed_unknown = 0,
    i82545_bus_speed_33,
    i82545_bus_speed_66,
    i82545_bus_speed_100,
    i82545_bus_speed_120,
    i82545_bus_speed_133,
    i82545_bus_speed_reserved
} i82545_bus_speed;

/* PCI bus widths */
typedef enum {
    i82545_bus_width_unknown = 0,
    i82545_bus_width_32,
    i82545_bus_width_64,
    i82545_bus_width_reserved
} i82545_bus_width;

#endif
