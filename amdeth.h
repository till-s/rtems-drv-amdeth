/* $Id$ */
#ifndef TILL_AMD_ETHER_DRIVER_H
#define TILL_AMD_ETHER_DRIVER_H

/* use the default device */
#define AMDETH_DEFAULT_DEVICE	0

typedef struct AmdEthDevRec_	*AmdEthDev;
typedef struct EtherHeaderRec_	*EtherHeader;

/* error codes - some code relies on them being < 0 */
#define AMDETH_OK		0
#define AMDETH_ERROR	(-1)
#define AMDETH_BUSY		(-2)

/* allocate and initialize a device structure
 * which is returned in d.
 */
int
amdEthInit(AmdEthDev *d, int instance, int flags);
/* whether to use/enable the receiver */
#define AMDETH_FLG_USE_RX			(1<<0)
/* whether to automatically update stats/clear TX errors
 * when in asynchronous TX mode
 */
#define AMDETH_FLG_AUTO_TX_STATS	(1<<1)
/* whether to let the ISR automatically update
 * RX statistics and let the RX overwrite the
 * same buffer.
 */
#define AMDETH_FLG_AUTO_RX			(1<<2)
/* disable broadcast reception */
#define AMDETH_FLG_NOBCST			(1<<4)

/* stop and release a device
 * RETURNS: 0 on success, -1 on error (invalid d pointer)
 */
int
amdEthCloseDev(AmdEthDev d);

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

int
amdEthReceivePacket(AmdEthDev d, char *buf, int size);

#endif

