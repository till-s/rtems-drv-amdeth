/* $Id$ */

/* Author: Till Straumann, <strauman@slac.stanford.edu> 7/2001 */

#include <assert.h>
#include <stdio.h>

#include <netinet/in.h>	/* htons */

#include "amdeth.h"
#include "wrap.h"

#define NUM_ETH_DEVICES 4	/* how many devices our table supports */
#define RT_DRIVER			/* whether to set some flags for real-time specific application */

#ifdef __rtems__
#include <rtems.h>
#include <bsp.h>
#include <bsp/pci.h>
#include <bsp/irq.h>
#include <libcpu/io.h>
#include "bsp/bspExt.h"
typedef unsigned int  pci_ulong;
typedef unsigned char pci_ubyte;
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr) + PCI_MEM_BASE)
#define LOCAL2PCI(memaddr) ((pci_ulong)(memaddr) + PCI_DRAM_OFFSET)
#define pciFindDevice BSP_pciFindDevice
#define pciConfigOutLong pci_write_config_dword
#define pciConfigInLong pci_read_config_dword
#define pciConfigInByte pci_read_config_byte

#elif defined(__vxworks)
#include <vxWorks.h>
typedef unsigned long pci_ulong;
typedef unsigned char pci_ubyte;
//#define PCI_VENDOR_ID_AMD	0x1022
//#define PCI_DEVICE_ID_AMD_LANCE	0x2000
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr))
#else
#error "Unknown Architecture"
#endif

#ifdef CPU_BIG_ENDIAN
#if defined(__PPC) || defined(__PPC__)
static inline void wrle(unsigned long w, void *addr)
{
	__asm__ __volatile__("stwbrx %0, 0, %1"::"r"(w),"r"(addr));
}
static inline unsigned long rdle(unsigned long *addr)
{
	__asm__ __volatile__("lwbrx %0, 0, %0":"=r"(addr):"0"(addr));
	return (unsigned long)addr;
}
#endif
#endif

#ifndef PCI_VENDOR_ID_AMD
#define PCI_VENDOR_ID_AMD 0x1022
#endif
#ifndef PCI_DEVICE_ID_AMD_LANCE
#define PCI_DEVICE_ID_AMD_LANCE 0x2000
#endif

#define STYLEFLAGS	BCR20_SWSTYLE_3
#define STYLE		style3

#undef RXTEST

/* register bit definitions */
#define BCR19_PVALID	(1<<15) /* status: EEPROM (detected and contents) valid */
#define BCR19_PREAD	(1<<14)	/* EEPROM read command bit */
#define BCR19_EEDET	(1<<13)	/* EEPROM detect */
#define BCR19_EEN	(1<<4)	/* EEPROM port enable */
#define BCR19_ECS	(1<<2)	/* EEPROM chip select */
#define BCR19_ESK	(1<<1)	/* EEPROM serial clock */
#define BCR19_EDIO	(1<<0)	/* EEPROM data in/out */

#define BCR20_SWSTYLE_3	0x0003

#define CSR0_ERR	(1<<15)	/* status: cerr | miss | merr */
#define CSR0_CERR	(1<<13)	/* status: collision error */
#define CSR0_MISS	(1<<12)	/* status: missed frame interrupt */
#define CSR0_MERR	(1<<11)	/* status: memory error interrupt */
#define CSR0_RINT	(1<<10)	/* status: rx interrupt */
#define CSR0_TINT	(1<<9)	/* status: tx interrupt */
#define CSR0_IDON	(1<<8)	/* status: init done interrupt */
#define CSR0_INTR	(1<<7)	/* status: any interrupt */
#define CSR0_IENA	(1<<6)	/* enable interrupts */
#define CSR0_RXON	(1<<5)	/* receiver enabled (RO) */
#define CSR0_TXON	(1<<4)	/* transmitter enabled (RO) */
#define CSR0_TDMD	(1<<3)	/* demand transmission */
#define CSR0_STOP	(1<<2)	/* STOP all DMA activity */
#define CSR0_STRT	(1<<1)	/* START */
#define CSR0_INIT	(1<<0)	/* read init block */
#define CSR0_IRQ_STAT	( CSR0_MISS | CSR0_MERR | CSR0_RINT | CSR0_TINT | CSR0_IDON)


#define CSR3_MISSM	(1<<12)	/* mask missed frame irq */
#define CSR3_MERRM	(1<<11)	/* mask memory error irq */
#define CSR3_RINTM	(1<<10)	/* mask rx irq */
#define CSR3_TINTM	(1<<9)	/* mask tx irq */
#define CSR3_IDONM	(1<<8)	/* mask init done irq */
#define CSR3_IRQ_MSK	(CSR3_MISSM | CSR3_MERRM | CSR3_RINTM | CSR3_TINTM | CSR3_IDONM)
#define CSR3_DXSUFLO	(1<<6)	/* disable xmit stop on underflow error */
#define CSR3_LAPPEN	(1<<5)	/* enable look ahead packet procesing */
#define CSR3_DXMT2PD	(1<<4)	/* disable xmit two part deferral */
#define CSR3_EMBA	(1<<3)	/* enable modified backoff algorithm */
#define CSR3_BSWP	(1<<2)	/* byte swap (FIFO access only) */

#define CSR4_TXDPOLL	(1<<12)	/* disable tx polling */
#define CSR4_APAD_XMT	(1<<11)	/* autopad transmission */
#define CSR4_ASTRP_RCV	(1<<10) /* autostrip padded reception */
#define CSR4_MFCO	(1<<9)	/* status: missed frame counter overflow */
#define CSR4_MFCOM	(1<<8)	/* mask missed frame cnt ovfl irq */
#define CSR4_UINTCMD	(1<<7)	/* user irq command */
#define CSR4_UINT	(1<<6)	/* status: user command irq */
#define CSR4_RCVCCO	(1<<5)	/* status: rx collision counter overflow */
#define CSR4_RCVCCOM	(1<<4)	/* mask rx collision cnt ovfl irq */
#define CSR4_TXSTRT	(1<<3)	/* status: xmission started */
#define CSR4_TXSTRTM	(1<<2)	/* mask TXSTRT irq */
#define CSR4_IRQ_MSK	(CSR4_MFCOM | CSR4_RCVCCOM | CSR4_TXSTRTM)
#define CSR4_IRQ_STAT	(CSR4_MFCO | CSR4_UINT | CSR4_RCVCCO | CSR4_TXSTRT)

#define CSR5_TOKINTD	(1<<15)	/* disable xmit ok irq */
#define CSR5_LTINTEN	(1<<14)	/* enable last xmit irq */
#define CSR5_SINT	(1<<11)	/* status: system error (bus master) irq */
#define CSR5_SINTE	(1<<10)	/* enable system error irq */
#define CSR5_EXDINT	(1<<7)	/* status: excessive deferral irq */
#define CSR5_EXDINTE	(1<<6)	/* enable excessive deferral irq */
#define CSR5_MPPLBA	(1<<5)	/* magic packet physical logical broadcast */
#define CSR5_MPINT	(1<<4)	/* status: magic packet irq */
#define CSR5_MPINTE	(1<<3)	/* enable magic packet irq */
#define CSR5_MPEN	(1<<2)	/* enable magic packet */
#define CSR5_MPMODE	(1<<1)	/* magic packet mode */
#define CSR5_SPND	(1<<0)	/* request entrance into suspend mode */
#define CSR5_IRQ_STAT	(CSR5_SINT | CSR5_EXDINT | CSR5_MPINT)

#define CSR7_FASTSPNDE	(1<<15)	/* fast suspend enable */
#define CSR7_RXFRTG	(1<<14)	/* receive frame tag */
#define CSR7_RDMD	(1<<13)	/* receive demand (rx poll immediately) */
#define CSR7_RXDPOLL	(1<<12)	/* disable rx polling */
#define CSR7_STINT	(1<<11)	/* status: software timer interrupt */
#define CSR7_STINTE	(1<<10)	/* enable software timer irq */
#define CSR7_MREINT	(1<<9)	/* status: phy management read error irq */
#define CSR7_MREINTE	(1<<8)	/* enable phy management error irq */
#define CSR7_MAPINT	(1<<7)	/* status: phy management auto poll irq (status change) */
#define CSR7_MAPINTE	(1<<6)	/* enable phy management auto poll irq */
#define CSR7_MCCINT	(1<<5)	/* status: phy management command complete irq */
#define CSR7_MCCINTE	(1<<4)	/* enable MCCINT */
#define CSR7_MCCIINT	(1<<3)	/* status: phy management command complete internal irq */
#define CSR7_MCCIINTE	(1<<2)	/* enable MCCIINT */
#define CSR7_MIIPDTINT	(1<<1)	/* status: phy detect transition irq */
#define CSR7_MIIPDTINTE	(1<<0)	/* enable MIIPDTINT */
#define CSR7_IRQ_STAT   (CSR7_STINT | CSR7_MREINT | CSR7_MAPINT | CSR7_MCCINT | CSR7_MCCIINT | CSR7_MIIPDTINT)

#define CSR15_PROM	(1<<15) /* promiscuous mode */
#define CSR15_DRCVBC	(1<<14)	/* disable receive broadcast */
#define CSR15_DFRCVPA	(1<<13)	/* disable receive physical address */
#define CSR15_PORTSEL	(3<<7)	/* select network medium (must be 11b) */
#define CSR15_INTL	(1<<6)	/* internal loopback */
#define CSR15_DRTY	(1<<5)	/* disable retry */
#define CSR15_FCOLL	(1<<4)	/* force collision */
#define CSR15_DXMTFCS	(1<<3)	/* disable transmit crc (FCS) */
#define CSR15_LOOP	(1<<2)	/* loopback enable */
#define CSR15_DTX	(1<<1)	/* disable transmitter */
#define CSR15_DRX	(1<<0)	/* disable receiver */

#define CSR80_RCVFW_16	(0<<12)	/* rxc fifo watermark = 16 */
#define CSR80_RCVFW_64	(1<<12)	/* rxc fifo watermark = 64 */
#define CSR80_RCVFW_112	(2<<12)	/* rxc fifo watermark = 112 */
#define CSR80_XMTSP_MIN	(0<<10) /* tx start point 20 (36 if SRAM_SIZE > 0) */
#define CSR80_XMTSP_64	(1<<10) /* tx start point 64 */
#define CSR80_XMTSP_128	(2<<10) /* tx start point 128 */
#define CSR80_XMTSP_MAX	(3<<10) /* tx start point 220 (full packet if NOUFLO is set) */
#define CSR80_XMTFW_16	(0<<8)	/* tx fifo watermark (for DMA request) = 16 */
#define CSR80_XMTFW_64	(1<<8)	/* tx fifo watermark (for DMA request) = 64 */
#define CSR80_XMTFW_108	(2<<8)	/* tx fifo watermark (for DMA request) = 108 */

#define CSR100_MERRTO_MASK ((1<<16)-1)	/* PCI latency timer in .1us */


typedef struct LanceRegs32Rec_ {
	volatile unsigned char aprom[16];
	volatile unsigned long rdp;
	volatile unsigned long rap;
	volatile unsigned long reset;
	volatile unsigned long bdp;
} __attribute__ (( aligned(16), packed)) LanceRegs32Rec, *LanceRegs32;

/* RX buffer descriptor
 * Note that the Am79C973 uses this area LITTLE ENDIAN
 * and that it's different, depending on the software
 * style used!
 * The descriptor ring must be 16byte aligned! 
 */
typedef union RxBufDescU_ {
	struct {
		unsigned long	rbadrLE;/* buffer address */
		unsigned long	mcntLE;	/* message count and frame tag */
		unsigned long	statLE; /* status bits */
		unsigned long	user;	/* user space */
	} style2;
	struct {
		unsigned long	mcntLE;	/* message count and frame tag */
		unsigned long	statLE; /* status bits */
		unsigned long	rbadrLE;/* buffer address */
		unsigned long	user;	/* user space */
	} style3;
} __attribute__ (( aligned(16),packed )) RxBufDescU, *RxBufDesc;

/* bit definitions for rx buffer descriptors */
#define RXDESC_MCNT_MASK	((1<<12)-1)	/* mask for the message count */
#define RXDESC_MCNT_FRTAG(mcnt)	((mcnt>>16) & ((1<<15)-1)) /* extract RX frame tag */
#define RXDESC_STAT_OWN		(1<<31)		/* descriptor owned by lance */
#define RXDESC_STAT_ERR		(1<<30) 	/* FRAM | OFLO | CRC | BUFF | BPE */
#define RXDESC_STAT_FRAM	(1<<29) 	/* framing error */
#define RXDESC_STAT_OFLO	(1<<28) 	/* overflow error (fifo overrun) */
#define RXDESC_STAT_CRC		(1<<27) 	/* CRC (FCS) error */
#define RXDESC_STAT_BUFF	(1<<26) 	/* buffer error (no empty buffer) */
#define RXDESC_STAT_STP		(1<<25) 	/* start of packet (first buffer in chain) */
#define RXDESC_STAT_ENP		(1<<24) 	/* end of packet (last buffer in chain) */
#define RXDESC_STAT_BPE		(1<<23) 	/* bus parity error */
#define RXDESC_STAT_PAM		(1<<22) 	/* physical address match */
#define RXDESC_STAT_LAPM	(1<<21) 	/* logical address filter match */
#define RXDESC_STAT_BAM		(1<<20)		/* broadcast address match */
#define RXDESC_STAT_ONES	(15<<12)	/* host MUST write/set these */
#define RXDESC_STAT_BCNT_MSK	((1<<12)-1)	/* 2s COMPLEMENT of buffer length */

#define NumberOf(arr) (sizeof(arr)/sizeof(arr[0]))

/* TX buffer descriptor
 * Note that the Am79C973 uses this area LITTLE ENDIAN
 * and that it's different, depending on the software
 * style used!
 * The descriptor ring must be 16byte aligned! 
 */
typedef union TxBufDescU_ {
	struct {
		unsigned long	tbadrLE;/* buffer address */
		unsigned long	csr1LE;	/* status and byte count */
		unsigned long	csr2LE;	/* status bits */
		unsigned long	user;	/* user space */
	} style2;
	struct {
		unsigned long	csr2LE;	/* status bits */
		unsigned long	csr1LE;	/* status and byte count */
		unsigned long	tbadrLE;/* buffer address */
		unsigned long	user;	/* user space */
	} style3;
} __attribute__ (( aligned(16),packed )) TxBufDescU, *TxBufDesc;

/* bit definitions for tx buffer descriptors */
#define TXDESC_CSR1_OWN		(1<<31)		/* descriptor owned by lance */
#define TXDESC_CSR1_ERR		(1<<30)		/* UFLO|LCOL|LCAR|RTRY|BPE */
#define TXDESC_CSR1_ADD_FCS	(1<<29)		/* activate TX FCS generation */
#define TXDESC_CSR1_MORE	(1<<28)		/* more than 1 rexmit needed */
#define TXDESC_CSR1_LTINT	(1<<28)		/* suppress irq after successful xmit (if 0) */
#define TXDESC_CSR1_ONE		(1<<27)		/* one retry needed for xmission */
#define TXDESC_CSR1_DEF		(1<<26)		/* deferral detected */
#define TXDESC_CSR1_STP		(1<<25)		/* first buffer in packet (chain) */
#define TXDESC_CSR1_ENP		(1<<24)		/* last buffer in packet (chain) */
#define TXDESC_CSR1_BPE		(1<<23)		/* bus parity error detected */
#define TXDESC_CSR1_ONES	(15<<12)	/* MUST be written as ones by host */
#define TXDESC_CSR1_BCNT_MSK	((1<<12)-1)	/* buffer byte count (2s COMPLEMENT) */

#define TXDESC_CSR2_BUFF	(1<<31)		/* buffer error (no free buffer) */
#define TXDESC_CSR2_UFLO	(1<<30)		/* fifo underflow error */
#define TXDESC_CSR2_EXDEF	(1<<29)		/* excessive deferral */
#define TXDESC_CSR2_LCOL	(1<<28)		/* late collision */
#define TXDESC_CSR2_LCAR	(1<<27)		/* loss of carrier */
#define TXDESC_CSR2_RTRY	(1<<26)		/* retry error (more than 16 attempts) */
#define TXDESC_CSR2_TRC_MSK	(15)		/* mask for the retry count */

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


/* TODO add mutex for driver access */
typedef struct AmdEthDevRec_ {
	rtems_irq_connect_data	brokenbydesign;
	struct AmdEthDevRec_	*next;			/* devices sharing a common interrupt */
	LanceRegs32	baseAddr;
	int			irqLine;
	RxBufDescU	rdesc[1];	/* one dummy descriptor */
	TxBufDescU	tdesc[2];	/* two descriptors; one for the header, one for the data */
	EtherHeaderRec	header;
	PSemaId		sync;
	int			flags;
	struct		{
		unsigned long	txPackets;	/* # packets sent */
		unsigned long	rxPackets;	/* # packets received */
		unsigned long	irqs;
		/* all below here are error counts */
#ifdef RT_DRIVER
		unsigned long	txDeferred;	/* deferrals are errors */
#endif
		unsigned long	txFifoUnderflow;
		unsigned long	txLateCollision;
		unsigned long	txCarrierLoss;
		unsigned long	txRetry;
		unsigned long	txBusParity;

		unsigned long	rxMemory;
		unsigned long	rxFifoOverflow;
		unsigned long	rxFraming;
		unsigned long	rxCrc;
		unsigned long	rxBusParity;
		unsigned long	rxOverrunHi;
		unsigned long	rxCollisionHi;
		unsigned long	sysError;
	}		stats;
} AmdEthDevRec;

static int amdEthBogusIrqs = 0;

/*TSILL*/
unsigned long roundtrip;

/* Some setup bits */
#ifdef RT_DRIVER
#define TXDESC_ERRS	(TXDESC_CSR1_DEF | TXDESC_CSR1_ERR)
#define CSR4_RTFLAGS	(CSR4_TXDPOLL)	/* transmit on demand only */
#define CSR7_RTFLAGS	(CSR7_RXDPOLL)	/* poll RX buffers on demand only */
#define CSR15_RTFLAGS	(CSR15_DRTY)	/* don't retry transmission - there should be no collisions
					 * and we'd rather know when something goes wrong
					 */
#define CSR100_SETUP	200		/* allow 20 us max bus latency */
#else
#define TXDESC_ERRS	(TXDESC_CSR1_ERR)
#define CSR4_RTFLAGS	(0)
#define CSR7_RTFLAGS	(0)
#define CSR15_RTFLAGS	(0)
#define CSR100_SETUP	1500		/* allow 150 us max bus latency */
#endif

#define CSR0_SETUP	(0)
#define CSR3_SETUP	((CSR3_IRQ_MSK & ~(CSR3_MISSM | CSR3_MERRM | CSR3_TINTM | CSR3_RINTM)) | CSR3_DXSUFLO)
#define CSR4_SETUP	((CSR4_IRQ_MSK & ~(CSR4_MFCOM | CSR4_RCVCCOM)) | CSR4_RTFLAGS)
#define CSR5_SETUP	(CSR5_TOKINTD)
#define CSR7_SETUP	(CSR7_RTFLAGS)
#define CSR15_SETUP	(CSR15_RTFLAGS)

static int  amdEthIntIsOn();
static void amdEthIntEnable();
static void amdEthIntDisable();
static void amdEthIsr();
int amdEthUpdateTxStats(AmdEthDev d, int i);
static unsigned long long read_counter(AmdEthDev d, int csr, volatile unsigned long *pHi);

static AmdEthDevRec devices[NUM_ETH_DEVICES]={{{0},0},};

/* Because we cannot pass an argument to the ISR, we must
 * work and hack :-(
 */

typedef struct InsaneApiMapRec_ {
	void		(*isr)(void);
	AmdEthDev	device;
} InsaneApiMapRec, InsaneApiMap;

static void isrA();
static void isrB();
static void isrC();
static void isrD();

/* A table to map (CPU/board/system dependent) interrupt lines to wrappers */
static InsaneApiMapRec insaneApiMap[4] = {{isrA,0},{isrB,0},{isrC,0},{isrD,0}};

/* Need a wrapper for each of the 4 PCI interrupts;
 * several devices could share one of them
 */
static void isrA() { amdEthIsr(insaneApiMap[0].device); }
static void isrB() { amdEthIsr(insaneApiMap[1].device); }
static void isrC() { amdEthIsr(insaneApiMap[2].device); }
static void isrD() { amdEthIsr(insaneApiMap[3].device); }

#ifdef RXTEST
static unsigned char rxbuffer[NumberOf(devices[0].rdesc)][2048];
#endif

/* read and write a data / bus registers in the lance.  */
/* NOTE: registers should be mapped to guarded memory. Hence, 
 *       stores are inherently in order. We must enforce ordering
 *       of the lwbrx instruction below with respect to the stwbrx, however.
 */
static inline void
WriteIndReg(
	unsigned long value,
	unsigned long idx,
	volatile unsigned long *areg,
	volatile unsigned long *dreg)
{
	__asm__ __volatile__(
		"stwbrx %1, 0, %2; stwbrx %0, 0, %3": :"r"(value),"r"(idx),"r"(areg),"r"(dreg));
}

static inline unsigned long
ReadIndReg(
	unsigned long idx, 
	volatile unsigned long *areg, 
	volatile unsigned long *dreg)
{
	__asm__ __volatile__(
		"stwbrx %0, 0, %2; eieio; lwbrx %0, 0, %3"
		:"=r"(idx) 
		:"0"(idx),"r"(areg),"r"(dreg));
	return idx;
}

/* when using these macros, the 'rap', 'rdp' and 'bdp' variables
 * must be set up appropriately.
 */
#define WBCR(idx, val) WriteIndReg(val,idx,rap,bdp)
#define RBCR(idx) ReadIndReg(idx,rap,bdp)
#define WCSR(idx, val) WriteIndReg(val,idx,rap,rdp)
#define RCSR(idx) ReadIndReg(idx,rap,rdp)

static char *dn="Amd Ethernet:";

static void
amdEthPrintAddr(AmdEthDev d, FILE *f, char *header)
{
int i;

	if (header)
		fprintf(f,header);

	for (i=0; i<6; i++) {
		fprintf(f,"%02x", d->baseAddr->aprom[i]);
		if ( i < 5)
			fputc(':',f);
	}
}

void
amdEthSetSrc(EtherHeader h, AmdEthDev d)
{
	memcpy(h->src, (void*)d->baseAddr->aprom, sizeof(h->src));
}

void
amdEthHeaderInit(EtherHeader h, char *dst, AmdEthDev d)
{
	/* Only initialize the addresses if they specify a dst/src */
	if (dst) {
		memcpy(h->dst,dst,sizeof(h->dst));
	}
	if (d)
		amdEthSetSrc(h,d);
	h->len=htons(0);
	h->llc8022.dsap = 0xAA;
	h->llc8022.ssap = 0xAA;
	h->llc8022.ctrl = 0x3;
	h->snap.org[0]  = 0x08; /* Stanford OUI */
	h->snap.org[1]  = 0x00;
	h->snap.org[2]  = 0x56;
	h->snap.type    = htons(0x805b); /* stanford V kernel ethernet type */
}


static void
amdEthIsr(AmdEthDev	d)
{
register volatile unsigned long *rap, *rdp;
register unsigned long	tmp;
unsigned long			savedAR;
int						bogusIrq = 1;

	for ( ; d; d=d->next ) {

		/* save the rap before changing it (don't care about endianness) */
		savedAR = d->baseAddr->rap;

		rap = &d->baseAddr->rap;
		rdp = &d->baseAddr->rdp;

		if ( ((tmp=RCSR(0)) & CSR0_INTR) ) {
			/* to check: EXDINT, IDON, MERR, MISS, MFCO, RCVCCO, RINT,
			 * done (lowercase - x: unused):
			 *                x   x    merr  miss  mfco  rcvcco  rint
			 * register:
			 *             csr5  csr0  csr0  csr0   csr4   csr4  csr0 
			 *           SINT, TINT, TXSTRT, UINT, STINT, MREINT, MCCINT,
			 *           sint     x       x     x      x       x       x
			 *           csr5  csr0    csr4  csr4   csr7   csr7   csr7 
			 *           MIIPDTINT, MAPINT, MCCIINT, MPINT
			 *                   x       x       x     x
			 *                 csr7    csr7    csr7  csr5
			 */
			d->stats.irqs++;
			bogusIrq = 0;
#if 0
			if ( tmp & CSR0_MISS ) {
				/* missed frame - we use the hardware counter but action
				 * could be taken here also, maybe post a special event...
				 */
			}
#endif
			if ( tmp & CSR0_MERR ) {
				/* memory / PCI bus error */
				d->stats.rxMemory++;
			}
			if ( tmp & CSR0_TINT ) {
				/* TS interrupt not used */
				if (AMDETH_FLG_AUTO_TX_STATS & d->flags) {
					int i;
					for ( i = 0; i < NumberOf(d->tdesc); i++)
						amdEthUpdateTxStats(d,i);

				}
			}
#if 0	
		/* We post to the receiver in any case the controller has
		 * released the descriptor. (NO)
		 */
#endif
			if ( tmp & CSR0_RINT ) {
				/* received a packet, post an event */
				if ( ! (rdle(&d->rdesc[0].STYLE.statLE) & RXDESC_STAT_OWN) ) {
					/* TSILL */
					unsigned long now;
					__asm__ __volatile__("mftb %0":"=r"(now));
					roundtrip = now-roundtrip;
					pSemPost(&d->sync);
				}
			}
			/* clear raised flags */
	 		WCSR(0,tmp);

			/* handle user and the counter rollover interrupts */
			tmp=RCSR(4);
			if (tmp & CSR4_MFCO)
				d->stats.rxOverrunHi++;
			if (tmp & CSR4_RCVCCO)
				d->stats.rxCollisionHi++;
			/* clear raised flags */
			WCSR(4,tmp);

			tmp=RCSR(5);
			if (tmp & CSR5_SINT) {
				/* this is probably really bad */
				d->stats.sysError++;
				/* TODO we should do something here */
			}
			WCSR(5,tmp);

		}

		/* restore address register contents */
		d->baseAddr->rap=savedAR;

	}
	amdEthBogusIrqs += bogusIrq;
}

static int
amdEthIntIsOn(const rtems_irq_connect_data *arg)
{
int			i;
AmdEthDev	d;
register volatile unsigned long *rap;
register volatile unsigned long *rdp;
	for (i = 0; i < NumberOf(insaneApiMap); i++) {
		if ( (d = insaneApiMap[i].device) &&
			  d->brokenbydesign.name == arg->name) {
			rap = &d->baseAddr->rap;
			rdp = &d->baseAddr->rdp;
			return	(RCSR(0) & CSR0_IENA) || (RCSR(5) & CSR5_SINTE);
		}
	}
	return 0;
}

static inline void
intEnable(AmdEthDev d)
{
register volatile unsigned long *rap;
register volatile unsigned long *rdp;
	rap = &d->baseAddr->rap;
	rdp = &d->baseAddr->rdp;
	WCSR( 0, RCSR(0) | CSR0_IENA);
	WCSR( 5, RCSR(5) | CSR5_SINTE);
}

static void
amdEthIntEnable(const rtems_irq_connect_data *arg)
{
int			i;
AmdEthDev	d;

	for (i = 0; i < NumberOf(insaneApiMap); i++) {
		if ( (d = insaneApiMap[i].device) &&
			  d->brokenbydesign.name == arg->name) {
			do {
				intEnable(d);
				d = d->next;
			} while (d);
			return;
		}
	}
}

static inline void
intDisable(AmdEthDev d)
{
register volatile unsigned long *rap;
register volatile unsigned long *rdp;
	rap = &d->baseAddr->rap;
	rdp = &d->baseAddr->rdp;
	WCSR( 0, RCSR(0) & ~CSR0_IENA);
	WCSR( 5, RCSR(0) & ~CSR5_SINTE);
}

static void
amdEthIntDisable(const rtems_irq_connect_data *arg)
{
int			i;
AmdEthDev	d;

	for (i = 0; i < NumberOf(insaneApiMap); i++) {
		if ( (d = insaneApiMap[i].device) &&
			  d->brokenbydesign.name == arg->name) {
			do {
				intDisable(d);
				d = d->next;
			} while (d);
			return;
		}
	}
}

int
amdEthReceivePacket(AmdEthDev d, char *buf, int len)
{
register unsigned long rxstat;

	/* setup the RX descriptor */
	wrle(LOCAL2PCI(buf),&d->rdesc[0].STYLE.rbadrLE);
	/* make sure buffer address is written
	 * before yielding the descriptor
	 */
	__asm__ __volatile__("eieio");
	wrle(RXDESC_STAT_OWN | RXDESC_STAT_ONES | (-len & RXDESC_STAT_BCNT_MSK),
	     &d->rdesc[0].STYLE.statLE);
	/* yield the descriptor before enforcing a poll */
	__asm__ __volatile__("eieio");
	WriteIndReg(CSR7_SETUP|CSR7_RDMD, 7, &d->baseAddr->rap, &d->baseAddr->rdp);

	/* block on an event */
	pSemWait(&d->sync);

	rxstat=rdle(&d->rdesc[0].STYLE.statLE);

	if ( rxstat & RXDESC_STAT_ERR ) {
		/* do statistics */
		if ( rxstat & RXDESC_STAT_OFLO ) {
			d->stats.rxFifoOverflow++;
		} else {
			if ( rxstat & RXDESC_STAT_BUFF ) {
				/* count BUFF only if OFLO is not set also */
				d->stats.rxFifoOverflow++;
			}
			if ( rxstat & RXDESC_STAT_ENP ) {
				/* framing and crc error bits are only
				 * valid for ENP
				 */
				if ( rxstat & RXDESC_STAT_CRC ) {
					d->stats.rxCrc++;
					if ( rxstat & RXDESC_STAT_FRAM )
						d->stats.rxFraming++;
				}

			}
		}
		if ( (rxstat & RXDESC_STAT_BPE) &&
		     (rxstat & (RXDESC_STAT_ENP | RXDESC_STAT_OFLO | RXDESC_STAT_BUFF)) )
			d->stats.rxBusParity++;

		return AMDETH_ERROR;
	}
	d->stats.rxPackets++;
	/* get the real length */
	return ((int)rdle(&d->rdesc[0].STYLE.mcntLE)) & RXDESC_MCNT_MASK;
}

int
amdEthInit(AmdEthDev *pd, int instance, int flags)
{
int		bus,dev,fun,i;
pci_ulong	tmp;
pci_ubyte	tmpb;
AmdEthDev	d;

	/* for now, allow only one instance */
	assert(instance < NumberOf(devices));
	d=&devices[instance];
			
	if (devices[instance].baseAddr) {
		fprintf(stderr,"AmdEth #%i already initialized\n",instance);
		return AMDETH_ERROR;
	}

	memset(d, 0, sizeof(*d));

	/* scan PCI bus */
	if (pciFindDevice(
			PCI_VENDOR_ID_AMD,
			PCI_DEVICE_ID_AMD_LANCE,
			instance,
			&bus, &dev, &fun)) {
		fprintf(stderr,"%s unable to find an AMD Lance chip\n",dn);
		return AMDETH_ERROR;
	}

	pciConfigInLong(bus,dev,fun,PCI_BASE_ADDRESS_1, &tmp);
	if (tmp & 1 || !tmp) {
		fprintf(stderr,"%s base address not in memory space",dn);
		return AMDETH_ERROR;
	}
	d->baseAddr = (LanceRegs32)PCI2LOCAL(tmp);

	d->flags    = flags;

	pciConfigInByte(bus,dev,fun,PCI_INTERRUPT_LINE,&tmpb);
	d->irqLine=(unsigned char)tmpb;
	fprintf(stderr,"%s Lance PCI ethernet found at 0x%08x (IRQ %i),",
			dn, (unsigned int)d->baseAddr, d->irqLine);
	amdEthPrintAddr(d, stderr, " hardware address: ");
	fputc('\n',stderr);

	/* make sure we have bus master access */
	pciConfigInLong(bus,dev,fun,PCI_COMMAND,&tmp);
	tmp|=PCI_COMMAND_MASTER;
	pciConfigOutLong(bus,dev,fun,PCI_COMMAND,tmp);

	/* switch to 32bit io */
	d->baseAddr->rdp = 0x0; /* 0 is endian-safe :-) */
	__asm__ __volatile__("eieio");

	/* initialize BCR regs */
	{
		register volatile unsigned long *rap = &d->baseAddr->rap;
		register volatile unsigned long *bdp = &d->baseAddr->bdp;
		register volatile unsigned long *rdp = &d->baseAddr->rdp;

		WBCR(20, STYLEFLAGS);

		/* we really assume that there is an eeprom present providing some
		 * initialization.
		 * Note that we also assume that sufficient time has expired
		 * since HW reset until execution reaches this point for the
		 * controller to have read the EEPROM.
	 	 */
		{ register unsigned long lanceBCR19;
		lanceBCR19=ReadIndReg(19, rap, bdp);
		assert( lanceBCR19 & BCR19_PVALID );
		}

		/* clear interrupt and mask unneeded sources */
		WCSR(0,  CSR0_SETUP | CSR0_IRQ_STAT);

		WCSR(3,  CSR3_SETUP);

		WCSR(4,  CSR4_SETUP | CSR4_IRQ_STAT);

		WCSR(5,  CSR5_SETUP | CSR5_IRQ_STAT);

		WCSR(7,  CSR7_SETUP | CSR7_IRQ_STAT);

		/* switch off receiver ? */
		WCSR(15, CSR15_SETUP
					| (flags & AMDETH_FLG_USE_RX ? 0 : CSR15_DRX)
					| (flags & AMDETH_FLG_NOBCST ? CSR15_DRCVBC : 0)
			);

		/* clear logical address filters */
		WCSR(8, 0);
		WCSR(9, 0);
		WCSR(10, 0);
		WCSR(11, 0);
		/* physical address is read from EEPROM */
		/* receiver ring */
		WCSR(24, ((LOCAL2PCI(d->rdesc) &0xffff)));
		WCSR(25, ((LOCAL2PCI(d->rdesc) >> 16)&0xffff));
		WCSR(76, ((-(long)NumberOf(d->rdesc)) & 0xffff)); /* RX ring length */
		WCSR(49, 0); /* default RX polling interval */
		/* transmitter ring */
		printf("TSILL tdesc is at 0x%08x (PCI 0x%08x)\n",d->tdesc,LOCAL2PCI(d->tdesc));
		WCSR(30, ((LOCAL2PCI(d->tdesc) &0xffff)));
		WCSR(31, ((LOCAL2PCI(d->tdesc) >> 16)&0xffff));
		WCSR(78, ((-(long)NumberOf(d->tdesc)) & 0xffff)); /* TX ring length */
		WCSR(47, 0); /* default TX polling interval */

		WCSR(100,CSR100_MERRTO_MASK & CSR100_SETUP);

	/* initialize the buffer descriptors */
	{
		int i;
		/* fill the ethernet header */
		/* send to broadcast */
		amdEthHeaderInit(&d->header, 0, d);
		/* broadcast address */
		memset(d->header.dst, 0xff, sizeof(d->header.dst));
		/* clear ownership flags */
		for (i=0; i<NumberOf(d->tdesc); i++)
			wrle(0, &d->tdesc[i].STYLE.csr1LE);
#ifdef RXTEST
		memset(rxbuffer,0,sizeof(rxbuffer));
		for (i=0; i<NumberOf(d->rdesc); i++) {
			wrle(LOCAL2PCI(rxbuffer[i]),&d->rdesc[0].STYLE.rbadrLE);
			wrle(RXDESC_STAT_OWN |
			     RXDESC_STAT_ONES |
			     (-sizeof(rxbuffer[0]) & RXDESC_STAT_BCNT_MSK),
			     &d->rdesc[i].STYLE.statLE);
		}
#else
		for (i=0; i<NumberOf(d->rdesc); i++) {
			wrle(0, &d->rdesc[i].STYLE.statLE);
		}
#endif
	}

		WCSR(0, CSR0_SETUP | CSR0_STRT);
	}

	/* RTEMS binary semaphores allow nesting and are not
	 *       suited for task synchronization
	 */
	if (AMDETH_FLG_USE_RX & flags) 
		pSemCreate( 0/* binary */, 0 /* empty */, &d->sync);

	/* install ISR */

	d->brokenbydesign.on=amdEthIntEnable;
	d->brokenbydesign.off=amdEthIntDisable;
	d->brokenbydesign.isOn=amdEthIntIsOn;
	d->brokenbydesign.name=BSP_PCI_IRQ0 + d->irqLine;

	/* is an ISR for our line already installed ? */
	for (i = NumberOf(insaneApiMap) - 1; i >=0; i-- ) {
		AmdEthDev	d1;
		if ( (d1 = insaneApiMap[i].device) &&
			  d1->brokenbydesign.name == d->brokenbydesign.name) {
			/* link into existing entry */
			d->next = d1;
			insaneApiMap[i].device = d;
			d->brokenbydesign.hdl  = insaneApiMap[i].isr;
			intEnable(d);
			break;
		}
	}
	if ( i < 0 ) {
		/* create a new entry */
		for (i = NumberOf(insaneApiMap) - 1; i >=0 && insaneApiMap[i].device; i-- )
			/* do nothing else */;
		assert ( i >= 0 );
		insaneApiMap[i].device  = d;
		d->brokenbydesign.hdl   = insaneApiMap[i].isr;
		assert(BSP_install_rtems_irq_handler(&d->brokenbydesign));
	}


	if (pd) *pd=d;

	return AMDETH_OK;
}

/* update statistics and clear TX buffer errors
 * RETURNS: 0 if there were no errors, AMDETH_ERR
 *          if there were errors, AMDETH_BUSY if
 *          the descriptor pair is busy.
 */
int
amdEthUpdateTxStats(AmdEthDev d, int i)
{
unsigned long	csr1,csr2;

	csr1=rdle(&d->tdesc[i].STYLE.csr1LE);

	if ( (csr1 & TXDESC_CSR1_OWN) )
		return AMDETH_BUSY;

	if ( ! (csr1 & TXDESC_ERRS) )
		return 0;
	/* update statistics */
#ifdef RT_DRIVER
	if ( csr1 & TXDESC_CSR1_DEF )
		d->stats.txDeferred++;
#endif
	csr2=rdle(&d->tdesc[i].STYLE.csr2LE);

	if (csr2 & TXDESC_CSR2_UFLO)
		d->stats.txFifoUnderflow++;
	if (csr2 & TXDESC_CSR2_LCOL)
		d->stats.txLateCollision++;
	if (csr2 & TXDESC_CSR2_LCAR)
		d->stats.txCarrierLoss++;
	if (csr2 & TXDESC_CSR2_RTRY)
		d->stats.txRetry++;
	if (csr1 & TXDESC_CSR1_BPE)
		d->stats.txBusParity++;

	/* reset error bit */
	wrle( csr1 & ~ TXDESC_ERRS,
		&d->tdesc[i].STYLE.csr1LE );
	return AMDETH_ERROR;
}

/* send a packet; if no header is specified, use the default
 * broadcast header
 */

int
amdEthSendPacket(AmdEthDev d, EtherHeader h, void *data, int size)
{
register unsigned long csr1OR;
	if (!d) d=&devices[0];

	csr1OR=rdle(&d->tdesc[0].STYLE.csr1LE) | rdle(&d->tdesc[1].STYLE.csr1LE);
	if ( csr1OR & TXDESC_CSR1_OWN )
		return AMDETH_BUSY;

	/* previous transmission gave an error */
	if ( csr1OR & TXDESC_ERRS )
		return AMDETH_ERROR;
	
	/* do statistics */
	d->stats.txPackets++;

	if (!h)
		h=&d->header;
	h->len = htons(sizeof(*h) - 14  + size);
	/* clear csr2 */
	wrle(0,
		&d->tdesc[0].STYLE.csr2LE);
	wrle(0,
		&d->tdesc[1].STYLE.csr2LE);
	/* set buffer address of first bd */
	wrle(LOCAL2PCI(h),&d->tdesc[0].STYLE.tbadrLE);
	/* set buffer address of second bd */
	wrle(LOCAL2PCI(data), &d->tdesc[1].STYLE.tbadrLE);
	/* setup csr1 of second bd */
	/* NOTE: 2's complement of byte count! */
	wrle((-size & TXDESC_CSR1_BCNT_MSK) |
		TXDESC_CSR1_OWN | TXDESC_CSR1_ENP | TXDESC_CSR1_ONES,
		&d->tdesc[1].STYLE.csr1LE);
	/* make sure descriptors are written _before_ yielding the first one */
	__asm__ __volatile__("eieio");
	/* yield 1st descriptor */
	wrle((-sizeof(*h) & TXDESC_CSR1_BCNT_MSK) |
		TXDESC_CSR1_OWN | TXDESC_CSR1_STP | TXDESC_CSR1_ONES,
		&d->tdesc[0].STYLE.csr1LE);
	__asm__ __volatile__("eieio");
	{
	/* TSILL */
	unsigned long flags;
	/* demand transmission without changing status bits - we
	 * hope nobody (ISR or other thread) modifies the IENA flag while we are
	 * tampering with this register...
	 */
	register volatile unsigned long *rap = &d->baseAddr->rap;
	register volatile unsigned long *rdp = &d->baseAddr->rdp;
	rtems_interrupt_disable(flags);
	WCSR(0, (RCSR(0) & ~(CSR0_IRQ_STAT)) | CSR0_TDMD);
	__asm__ __volatile__("mftb %0":"=r"(roundtrip));
	rtems_interrupt_enable(flags);
	}
	return AMDETH_OK;
}

int
amdEthDumpStats(AmdEthDev d, FILE *f)
{
int	inc;

	if ( !f )
		f = stdout;

	if ( !d ) {
		d   = devices;
		inc = 1;
	} else {
		/* assert termination after dumping THE devices stats */
		inc = NumberOf(devices);
	}

	for ( ; d < devices + NumberOf(devices); d+=inc ) {

		if ( !d->baseAddr )
			continue;

		fprintf(f,"AMD Eth Interface #%i (", d - devices);
		amdEthPrintAddr(d, f, 0);
		fprintf(f,") statistics:\n");
		fprintf(f,"  # packets sent: %li\n",     d->stats.txPackets);
		fprintf(f,"  # pkt. received:%li\n",  d->stats.rxPackets);
		fprintf(f,"  # interrupts:   %li\n",     d->stats.irqs);
		fprintf(f,"TX Errors:\n");
		/* all below here are error counts */
#ifdef RT_DRIVER
		fprintf(f,"  deferred TX:    %li\n",     d->stats.txDeferred);	/* deferrals are errors */
#endif
		fprintf(f,"  fifo underflow: %li\n",    d->stats.txFifoUnderflow);
		fprintf(f,"  late collision: %li\n",    d->stats.txLateCollision);
		fprintf(f,"  carrier loss:   %li\n",    d->stats.txCarrierLoss);
		fprintf(f,"  retries:        %li\n",    d->stats.txRetry);
		fprintf(f,"  bus parity err.:%li\n",    d->stats.txBusParity);

		fprintf(f,"RX Errors:\n");
		fprintf(f,"  RX Memory err.: %li\n",    d->stats.rxMemory);
		fprintf(f,"  fifo overflow:  %li\n",    d->stats.rxFifoOverflow);
		fprintf(f,"  framing err.:   %li\n",    d->stats.rxFraming);
		fprintf(f,"  CRC err.:       %li\n",    d->stats.rxCrc);
		fprintf(f,"  bus parity err.:%li\n",    d->stats.rxBusParity);
		fprintf(f,"  missed frames:  %lli\n",   read_counter(d, 112, &d->stats.rxOverrunHi));
		fprintf(f,"  collision Hi:   %lli\n",   read_counter(d, 114, &d->stats.rxCollisionHi));
		fprintf(f,"  system err.:    %li\n",    d->stats.sysError);
	}

	fprintf(f,"%i bogus IRQs detected\n",       amdEthBogusIrqs);
	return 0;
}


/* routines for debugging / testing */
unsigned long
lance_read_csr(int offset)
{
	return ReadIndReg(offset,&devices[0].baseAddr->rap,&devices[0].baseAddr->rdp);

}

void
lance_write_csr(unsigned long val, int offset)
{
	WriteIndReg(val,offset,&devices[0].baseAddr->rap,&devices[0].baseAddr->rdp);
}

unsigned long
lance_read_bcr(int offset)
{
	return ReadIndReg(offset,&devices[0].baseAddr->rap,&devices[0].baseAddr->bdp);

}
void
lance_write_bcr(unsigned long val, int offset)
{
	WriteIndReg(val,offset,&devices[0].baseAddr->rap,&devices[0].baseAddr->bdp);
}

/* how to read a two-word counter without disabling interrupts
 *
 * Assumption: - thread executing this code is never suspended
 *               longer than it takes for the least significant
 *               word (LSW) to roll over.
 *             - ISR updates most significant word (MSW) when 
 *               LSW rolls over.
 *             - a 'rollover-pending' bit is available that is
 *               set by hardware and reset by the ISR updating
 *               the MSW.
 *             - ISR has higher priority than the thread executing
 *               the code (i.e. updating MSW and resetting the
 *               'pending' flag are atomic to the caller of
 *               read_counter()).
 *
 * e.g. assume hardware counts a short word, 1<<15 is used as
 * the 'rollover-pending' bit. Software uses a unsigned short
 * for counting the LSW rollovers. An interrupt is generated
 * when 1<<15 in the counter is set.
 *
 * unsigned short hw_counter; / * counts in hardware, issues IRQ * /
 * unsigned short hw_ovfl;    / * overflow bit, set by hardware when
 *                                hw_counter rolls over. Triggers IRQ * /
 *
 * unsigned short hi_count;   / * rollover area, maintained in sw * /
 *
 * isr()
 * {
 *   / * counter rolled over * /
 *   hi_count++;
 *   hw_ovfl = 0; / * reset rollover-pending * /
 * }
 *
 *
 * unsigned long read_counter()
 * {
 *   unsigned short lo_1, lo_2, hi;
 *
 *     lo_1 = hw_count;
 *       / * rollover could occur here (A) * /
 *     hi   = hi_count;
 *       / * or rollover could occur here (B) * /
 *     lo_2 = hw_count;
 *
 *     if ( lo_2 >= lo_1 ) {
 *       / * no overflow between lo_2 and lo_1 assignments * /
 *     } else {
 *       / * overflow; however, we don't know
 *          whether 'hi' contains the old or the new value * /
 *       if ( hw_ovfl ) {
 *         / * it is still pending - ISR has not executed yet * /
 *         hi++; / * adjust manually * /
 *       } else {
 *         / * ISR has run but we don't know if at A or B. In
 *             any case, the hi_count should be updated by _now_ * /
 *         hi = hi_count;
 *       }
 *     }
 *     return (hi << 16) | lo_2;
 * }
 *
 */

/* a simplified version who assumes the ISR always runs prior
 * to testing hw_ovfl
 */

static unsigned long long
read_counter(AmdEthDev d, int csr_no, volatile unsigned long *pHi)
{
unsigned short lo_1, lo_2;
unsigned long  hi;
register volatile unsigned long *rap = &d->baseAddr->rap;
register volatile unsigned long *rdp = &d->baseAddr->rdp;

		lo_1 = RCSR(csr_no);
		hi   = *pHi;
		lo_2 = RCSR(csr_no);

		if (lo_2 < lo_1) {
			/* overflow has just occurred; re-read Hi */
			hi = *pHi;
		}
		return (hi<<16)|lo_2;
}

int
_cexpModuleFinalize(void *mod)
{
int		 	i;
AmdEthDev	d;
register volatile unsigned long *rap;
register volatile unsigned long *rdp;
	
	for ( i=0; i < NumberOf(insaneApiMap); i++) {
		for ( d=insaneApiMap[i].device; d; d=d->next) {
			rap = &d->baseAddr->rap;
			rdp = &d->baseAddr->rdp;
			/* STOP DMA */
			WCSR( 0, CSR0_SETUP | CSR0_STOP);
			/* disable interrupts */
			intDisable(d);
			if (d->sync) {
				/* release sema */
				pSemPost(&d->sync);
				/* delete sema  */
				pSemDestroy(&d->sync);
			}
		}
		/* uninstall ISR */
		BSP_remove_rtems_irq_handler(&insaneApiMap[i].device->brokenbydesign);
	}
	return 0;
}
