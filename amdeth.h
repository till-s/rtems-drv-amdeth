/* $Id$ */
#ifndef TILL_AMD_ETHER_DRIVER_H
#define TILL_AMD_ETHER_DRIVER_H

#ifdef __rtems
#include <rtems.h>
#include <bsp/pci.h>
#include <libcpu/io.h>
#include "bspExt.h"
typedef unsigned int  pci_ulong;
typedef unsigned char pci_ubyte;
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr) + _ISA_MEM_BASE)
#define LOCAL2PCI(memaddr) ((pci_ulong)(memaddr) + PCI_DRAM_OFFSET)

#elif defined(__vxworks)
#include <vxWorks.h>
typedef unsigned long pci_ulong;
typedef unsigned char pci_ubyte;
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr))
#else
#error "Unknown Architecture"
#endif

/* use the default device */
#define AMDETH_DEFAULT_DEVICE	0

typedef struct AmdEthDevRec_ *AmdEthDev;

/* error codes */
#define AMDETH_OK		0
#define AMDETH_ERROR		(-1)
#define AMDETH_BUSY		(-2)

/* allocate and initialize a device structure
 * which is returned in d.
 */
int
amdEthInit(AmdEthDev *d, int instance);

/* broadcast a packet */
int
amdEthBroadcast(AmdEthDev d, void *packet, int size);

#endif

