/* $Id$ */
#ifndef TILL_AMD_ETHER_DRIVER_H
#define TILL_AMD_ETHER_DRIVER_H

#include <stdio.h>

/* use the default device */
#define AMDETH_DEFAULT_DEVICE	0

typedef struct AmdEthDevRec_	*AmdEthDev;
typedef struct EtherHeaderRec_	*EtherHeader;

typedef struct EtherHeaderRec_ {
	unsigned char	dst[6];
	unsigned char	src[6];
	unsigned short	len;	/* network byte order */
	struct	{
		unsigned char dsap;
		unsigned char ssap;
		unsigned char ctrl;
	} __attribute__ ((packed)) llc8022;
	struct  {
		unsigned char org[3];
		unsigned short type;
	} __attribute__ ((packed)) snap;
} __attribute__ ((packed)) EtherHeaderRec;

typedef struct {
	EtherHeaderRec	header;
	char			data[];
} EtherPacketRec, *EtherPacket; 

/* Provide a macro to declare a aligned header
 * so that any appended data (on receive) will be aligned.
 *
 * Use like this:
 *   typedef ALIGNED_ETHER_PACKET(alignement, data_size) XXX;
 *   XXX buffer;
 *   
 * Receive:
 *   amdEthReceivePacket(dev, &buffer.packet, sizeof(buffer.packet));
 * Access header
 *   buffer.packet.header / * is a EtherReaderRec 
 *   buffer.packet.data   / * is a char[data_size] array aligned to 'alignment'
 * Total size is sizeof(buffer);
 */

#define ALIGNED_ETHER_PACKET(alignment, data_size)      \
	struct {                                            \
		char pad[                                       \
              (                                         \
               (sizeof(EtherHeaderRec)+((alignment)-1)) \
                & ~((alignment)-1)                      \
              ) - sizeof(EtherHeaderRec)                \
                ];                                      \
		struct {                                        \
			EtherHeaderRec header;                      \
			char		   data[data_size];             \
		} packet;                                       \
	} __attribute__((aligned(alignment)))

/* error codes - some code relies on them being < 0 */
#define AMDETH_OK		0
#define AMDETH_ERROR	(-1)
#define AMDETH_BUSY		(-2)

/* allocate and initialize a device structure
 * which is returned in d.
 */
int
amdEthInit(AmdEthDev *d, int instance, int flags, int n_rx_desc, int n_tx_desc);
/* Transmitter modes */
#define AMDETH_FLG_TX_MODE_MSK		(0xf<<0)
#define AMDETH_FLG_TX_MODE(flagword)	((flagword)&AMDETH_FLG_TX_MODE_MSK)
#define AMDETH_FLG_TX_MODE_MAX		AMDETH_FLG_TX_MODE_LAZY

/* TX unused / disabled */
#define AMDETH_FLG_TX_MODE_OFF		(0<<0)
/* whether to automatically update stats/clear TX errors
 * when in asynchronous TX mode
 */
#define AMDETH_FLG_TX_MODE_AUTO		(1<<0)
/* user must poll (call amdEthUpdateTxStats()) to release desc/update stats */
#define AMDETH_FLG_TX_MODE_POLL		(2<<0)
/* TX ring is not swiped after interrupts but only when the user sends
 * new packets then statistics from the oldest ring entry are picked
 * up.
 */
#define AMDETH_FLG_TX_MODE_LAZY		(3<<0)

/* Receiver modes */
#define AMDETH_FLG_RX_MODE_MSK		(0xf<<4)
#define AMDETH_FLG_RX_MODE(flagword)	((flagword)&AMDETH_FLG_RX_MODE_MSK)
#define AMDETH_FLG_RX_MODE_MAX		AMDETH_FLG_RX_MODE_SYNC

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
#define AMDETH_FLG_DO_RETRY			(1<<10)

#define AMDETH_FLG_HDR_TYPE_MASK    (7<<11)

#define AMDETH_FLG_HDR_SNAP			(0<<11)

/* use ethernet headers rather than 802.2/snap headers */
#define AMDETH_FLG_HDR_ETHERNET		(1<<11)
/* IP header (w/o options)                             */
#define AMDETH_FLG_HDR_IP           (2<<11)
/* UDP header (w/o IP options)                         */
#define AMDETH_FLG_HDR_UDP          (3<<11)

/* disable some more strict 'real-time' constraints
 * (might be beneficial if operating on a normal LAN
 * for testing purposes). E.g., in 'real-time' mode
 * TX deferrals due to collisions are considered errors.
 * Also, letting the chip poll for work is disabled
 * in real-time mode.
 */
#define AMDETH_FLG_RT_DIS			(1<<12)

/* stop and release a device
 * RETURNS: 0 on success, -1 on error (invalid d pointer)
 *
 * An optional 'cleanup' callback function may be specified
 * which is called on all non-NULL tx- and rx- ring entries.
 * This is useful for releasing resources when the device
 * is shutting down.
 * The 'tx' argument is nonzero when the TX-ring is cleaned
 * and zero when the RX-ring is cleaned.
 */
int
amdEthCloseDev(AmdEthDev d, void (*cleanup)(int tx, void *buf_p, void *closure), void *closure);

/* obtain the size of the (opaque) snap header */
int
amdEthGetHeaderSize(AmdEthDev d);

/* initialize an ethernet/snap header
 *  - if dst!= 0, fill in the destination address
 *  - if d!= 0, fill in the source address using d's
 *    station address.
 */
void
amdEthHeaderInit(EtherHeader h, char *dst, AmdEthDev d);

/* send a packet with a header. If no header is specified
 * (NULL), use the default broadcast header.
 *
 * 'header' can also be AMDETH_TX_HEADER_NONE in order to
 * omit the header (it is then assumed the 'payload' already
 * contains a header).
 */
#define AMDETH_TX_HEADER_NONE ((EtherHeader)-1)

/* send a packet */
int
amdEthSendPacket(AmdEthDev d, EtherHeader header, void *payload, int size);

/* send a packet and obtain the old buffer;
 * if a header had been provided then a pointer to the header
 * is returned - otherwise a pointer to the payload.
 */
int
amdEthSendPacketSwp(AmdEthDev d, EtherHeader header, void **payload, int size);
	
/* update tx statistics from buffer descriptors; pass i==-1 */
int amdEthUpdateTxStats(AmdEthDev d, unsigned i);

#define amdEthBroadcast(dev, payload, size) amdEthSendPacket((dev),0,(payload),(size))

/* Pass pointer to pointer to buffer. In polled mode, if a new packet is available
 * the supplied buffer is passed to the driver, and the new packet is returned
 * in *pbuf. *pbuf is unchanged if no valid new packet is available (error or no
 * packet received).
 * *pbuf is unchanged in AUTO or SYNC mode.
 * RETURNS: <0 on error, number of chars received if OK
 *
 * MODES:
 *    AUTO: after 'init' call this routine n_rx_desc times with different
 *          buffers. After the last of n_rx_desc buffers is passed, the RX
 *          starts filling the buffer chain automatically in a round-robin
 *          fashion (re-starting from the beginning after reaching the end).
 *          ('poor-man's virtual memory mode'). The Receive routine cannot be
 *          called again.
 *    SYNC: call this routine 'n_rx_desc' times with different buffers.
 *          During the first 'n_rx_desc' calls 'Receive' passes buffers
 *          to the receiver and returns immediately. After that, 'Receive'
 *          blocks until a packet is received and swaps buffers, i.e., the
 *          new packet is returned in *pbuf and the buffer arg is passed to
 *          the driver. If errors are encountered, the buffer is *not* swapped,
 *          i.e., the driver re-uses the previous buffer.
 *    POLL: on each call, the oldest buffer is inspected. If there is new
 *          data available, *pbuf is swapped, i.e., a pointer to the
 *          buffer with new data is returned in *pbuf and the buffer passed
 *          to the routine is passed on to the driver. This effectively
 *          implements a FIFO with n_rx_desc entries, i.e., it is possible
 *          to call 'Receive' n_rx_desc times with new/different buffers
 *          before it returns 'BUSY' [meaning that all n_rx_desc buffers
 *          are owned by the device which is waiting to receive data].
 */
int
amdEthReceivePacket(AmdEthDev d, char **pbuf, int size);

/* size is size of the data portion only */
typedef void (*AmdEthAutoRxCallback)(EtherPacket buf, int size, void *closure);

/* register a packet reception callback (AUTO mode only)
 * NOTE: the callback runs in ISR context if the driver is compiled
 *       for IRQ driven mode and in the context of the driver task
 *       if compiled for task driven mode.
 */
int
amdEthAutoCbRegister(AmdEthDev d, AmdEthAutoRxCallback callback, void *udata);

/* start driver task (task driven mode) - if callbacks use FP registers
 * the task must be started with 'fpTask'!=0. 
 * RETURNS: task ID or 0 on failure, -1 if task already running
 *          -1 in IRQ driven mode
 */
int
amdEthStart(int prio, int fpTask);

/* request suspend/resume; note that entering suspend mode may take
 * a while (until all network buffers/fifos etc. are drained).
 * This routine returns immediately, however.
 */
void
amdEthSuspend(AmdEthDev d);
void
amdEthResume(AmdEthDev d);

#define AMDETH_LINK_MSK_FD   1
#define AMDETH_LINK_MSK_10   2
#define AMDETH_LINK_MSK_100  4

#define AMDETH_LINK_DOWN     0
#define AMDETH_LINK_10_HD    2
#define AMDETH_LINK_10_FD    3
#define AMDETH_LINK_100_HD   4
#define AMDETH_LINK_100_FD   5

int
amdEthLinkStatus(AmdEthDev d);

typedef void (*AmdEthLinkCallback)(int linkStatus, void *closure);

int
amdEthLinkCbRegister(AmdEthDev d, AmdEthLinkCallback callback, void *closure);

AmdEthDev
amdEthGetDev(int inst);

void
amdEthMcFilterClear(AmdEthDev d);

/* Add mac-address to multicast filter hash table.
 * NOTE: an internal reference-count is kept so that
 *       it is OK to add the same address (or one
 *       that aliases to another) multiple times.
 */
void
amdEthMcFilterAdd(AmdEthDev d, unsigned char *mac_addr);

/*
 * Decrement reference count and delete mac-address
 * from multicast filter hash table when it reaches
 * zero.
 */
void
amdEthMcFilterDel(AmdEthDev d, unsigned char *mac_addr);

int
amdEthDumpStats(AmdEthDev d, FILE *f);

#endif

