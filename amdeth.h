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
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr) + PCI_MEM_BASE)
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

typedef struct AmdEthDevRec_	*AmdEthDev;
typedef struct EtherHeaderRec_	*EtherHeader;

/* error codes - some code relies on them being < 0 */
#define AMDETH_OK		0
#define AMDETH_ERROR		(-1)
#define AMDETH_BUSY		(-2)

/* allocate and initialize a device structure
 * which is returned in d.
 */
int
amdEthInit(AmdEthDev *d, int instance, int flags);
#define AMDETH_FLG_USE_RX	(1<<0)

/* initialize an ethernet/snap header
 *  - if dst!= 0, fill in the destination address
 *  - if d!= 0, fill in the source address using d's
 *    station address.
 */
void
amdEthHeaderInit(EtherHeader h, char *dst, AmdEthDev d);

/* send a packet with a header. If no header is specified
 * (NULL), use the default broadcast header.
 */

/* broadcast a packet */
int
amdEthSendPacket(AmdEthDev d, EtherHeader h, void *payload, int size);
	
#define amdEthBroadcast(dev, payload, size) amdEthSendPacket((dev),0,(payload),(size))

#endif

