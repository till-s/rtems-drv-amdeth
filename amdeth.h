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
/* Transmitter modes */
#define AMDETH_FLG_TX_MODE_MSK		(0xf<<0)
#define AMDETH_FLG_TX_MODE(flagword)	((flagword)&AMDETH_FLG_TX_MODE_MSK)

/* TX unused / disabled */
#define AMDETH_FLG_TX_MODE_OFF		(0<<0)
/* whether to automatically update stats/clear TX errors
 * when in asynchronous TX mode
 */
#define AMDETH_FLG_TX_MODE_AUTO		(1<<0)
/* user must poll (call amdEthUpdateTxStats()) to release desc/update stats */
#define AMDETH_FLG_TX_MODE_POLL		(2<<0)

/* Receiver modes */
#define AMDETH_FLG_RX_MODE_MSK		(0xf<<4)
#define AMDETH_FLG_RX_MODE(flagword)	((flagword)&AMDETH_FLG_RX_MODE_MSK)

/* RX unused / disabled */
#define AMDETH_FLG_RX_MODE_OFF		(0<<4)
/* whether to let the ISR automatically update
 * RX statistics and let the RX overwrite the
 * same buffer.
 */
#define AMDETH_FLG_RX_MODE_AUTO		(1<<4)
/* polled mode; user must call amdEthReceivePacket() to get status and provide new buffer */
#define AMDETH_FLG_RX_MODE_POLL		(2<<4)
/* synchronous mode; amdEthReceivePacket() blocks for a new packet */
#define AMDETH_FLG_RX_MODE_SYNC		(3<<4)

/* disable broadcast reception */
#define AMDETH_FLG_NOBCST			(1<<8)

/* I don't know of a way how to detect if the card has
 * a fiber or copper interface. Unfortunately, some
 * settings need to be different and hence YOU tell ME
 */
#define AMDETH_FLG_FIBER			(1<<9)

/* allow retries / collisions when sending */
#define AMDETH_FLG_DO_RETRY			(3<<8)

/* stop and release a device
 * RETURNS: 0 on success, -1 on error (invalid d pointer)
 */
int
amdEthCloseDev(AmdEthDev d);

/* obtain the size of the (opaque) snap header */
int
amdEthGetHeaderSize();

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

/* send a packet */
int
amdEthSendPacket(AmdEthDev d, EtherHeader header, void *payload, int size);
	
/* update tx statistics from buffer descriptors; pass i==-1 */
int amdEthUpdateTxStats(AmdEthDev d, unsigned i);

#define amdEthBroadcast(dev, payload, size) amdEthSendPacket((dev),0,(payload),(size))

int
amdEthReceivePacket(AmdEthDev d, char *buf, int size);

#endif

