/* $Id$ */

/* Author: Till Straumann, <strauman@slac.stanford.edu> 7/2001 */

/* THREAD SAFETY NOTE
 * ==================
 *
 * This driver is NOT thread safe. Any access to the device
 * involves setting a 'address' register followed by reading/writing
 * a 'data' register (indirect register access), a sequence which
 * is not atomic.
 * For sake of performance, no locking scheme is implemented, hence
 * it is left to the user to make sure multiple threads are not
 * accessing the same device.
 * The ISR which also performes register access *does* save/restore
 * the 'address' register setting across interrupts.
 */

#include <assert.h>
#include <stdio.h>

#include <netinet/in.h>	/* htons */

#include "amdeth.h"
#include "wrap/wrap.h"

#define NUM_ETH_DEVICES 4	/* how many devices our table supports */
#define RT_DRIVER			/* whether to set some flags for real-time specific application */
#define AUTOPOLL_BROKEN		/* I can't get media autopolling to work :-( */

#ifdef __rtems__
#include <rtems.h>
#include <bsp.h>
#include <bsp/pci.h>
#include <bsp/irq.h>
#include <libcpu/io.h>
#ifdef HAVE_LIBBSPEXT
#include <bsp/bspExt.h>
#endif
typedef unsigned int  pci_ulong;
typedef unsigned char pci_ubyte;
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr) + PCI_MEM_BASE)
#define LOCAL2PCI(memaddr) ((pci_ulong)(memaddr) + PCI_DRAM_OFFSET)
#define pciFindDevice BSP_pciFindDevice
#define pciConfigOutLong pci_write_config_dword
#define pciConfigOutByte pci_write_config_byte
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
static inline void wrle(unsigned long w, volatile void *addr)
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
#define BCR2_SMIUEN			(1<<15)	/* 975 only */
#define BCR2_DISSCR_SFEX	(1<<14)	/* disable scrambler; MUST SET FOR PECL (FIBER PORT) */
#define BCR2_PHYSELEN		(1<<13)	/* enable writes to BCR18[4:3] */
#define BCR2_LEDPE			(1<<12)	/* LED program enable; (enables writes to LED regs) */
#define BCR2_RESET_SFEX		(1<<11)	/* hold internal PHY in RESET */
#define BCR2_I2C_M3			(1<<10)	/* 975 only */
#define BCR2_I2C_M2			(1<<9)	/* 975 only */
#define BCR2_APROMWE		(1<<8)	/* enable writes to address prom (i.e., shadow ram) */
#define BCR2_INTLEVEL		(1<<7)	/* interrupt pin level(0) or edge (1) */
#define BCR2_I2C_M1			(1<<6)	/* 975 only */
#define BCR2_I2C_M0			(1<<5)	/* 975 only */
#define BCR2_I2C_N2			(1<<4)	/* 975 only */
#define BCR2_I2C_EADISEL	(1<<3)	/* EADI select; (external address select); muxes some pins */
#define BCR2_SLEEP_SFEX		(1<<2)	/* sleep PHY, reduce power comsumption */
#define BCR2_I2C_N1			(1<<1)	/* 975 only */
#define BCR2_ASEL			(1<<1)	/* Auto Select media interface port; */
									/* if 1 and MIIPD (BCR32[14]) is 1 -> MII port.
									 * in addition, if DANAS (BCR32[7]) is 0,
									 * external PHY is automatically configured.
									 * If ASEL==0, PORTSEL bits define port.
									 */
#define BCR2_I2C_N0			(1<<0)	/* 975 only */

#define BCR19_PVALID		(1<<15) /* status: EEPROM (detected and contents) valid */
#define BCR19_PREAD			(1<<14)	/* EEPROM read command bit */
#define BCR19_EEDET			(1<<13)	/* EEPROM detect */
#define BCR19_EEN			(1<<4)	/* EEPROM port enable */
#define BCR19_ECS			(1<<2)	/* EEPROM chip select */
#define BCR19_ESK			(1<<1)	/* EEPROM serial clock */
#define BCR19_EDIO			(1<<0)	/* EEPROM data in/out */

#define BCR20_SWSTYLE_3		0x0003

#define BCR25_SRAMSIZE		(0xff)

#define BCR27_LOLATRX		(1<<14)	/* low latency RX function; requires 
									 * SRAM > 0
									 * CSR127(RPA) = 1
									 */

#define BCR32_APEP			(1<<11) /* PHY auto-poll enable */

#define BCR32_APDW_MSK		(7<<8)  /* auto-poll dwell time (polling interval) */
#define BCR32_APDW_16us		(0<<8)
#define BCR32_APDW_103us	(1<<8)
#define BCR32_APDW_206us	(2<<8)
#define BCR32_APDW_410us	(3<<8)
#define BCR32_APDW_819us	(4<<8)
#define BCR32_APDW_1640us	(5<<8)

#define ANR24_LINK_UP		(1<<3)
#define ANR24_FULLDUP		(1<<2)
#define ANR24_ANEG_ALRT		(1<<1)
#define ANR24_SPEED_100		(1<<0)

#define CSR0_ERR			(1<<15)	/* status: cerr | miss | merr */
#define CSR0_CERR			(1<<13)	/* status: collision error */
#define CSR0_MISS			(1<<12)	/* status: missed frame interrupt */
#define CSR0_MERR			(1<<11)	/* status: memory error interrupt */
#define CSR0_RINT			(1<<10)	/* status: rx interrupt */
#define CSR0_TINT			(1<<9)	/* status: tx interrupt */
#define CSR0_IDON			(1<<8)	/* status: init done interrupt */
#define CSR0_INTR			(1<<7)	/* status: any interrupt */
#define CSR0_IENA			(1<<6)	/* enable interrupts */
#define CSR0_RXON			(1<<5)	/* receiver enabled (RO) */
#define CSR0_TXON			(1<<4)	/* transmitter enabled (RO) */
#define CSR0_TDMD			(1<<3)	/* demand transmission */
#define CSR0_STOP			(1<<2)	/* STOP all DMA activity */
#define CSR0_STRT			(1<<1)	/* START */
#define CSR0_INIT			(1<<0)	/* read init block */
#define CSR0_IRQ_STAT		( CSR0_MISS | CSR0_MERR | CSR0_RINT | CSR0_TINT | CSR0_IDON)


#define CSR3_MISSM			(1<<12)	/* mask missed frame irq */
#define CSR3_MERRM			(1<<11)	/* mask memory error irq */
#define CSR3_RINTM			(1<<10)	/* mask rx irq */
#define CSR3_TINTM			(1<<9)	/* mask tx irq */
#define CSR3_IDONM			(1<<8)	/* mask init done irq */
#define CSR3_IRQ_MSK		(CSR3_MISSM | CSR3_MERRM | CSR3_RINTM | CSR3_TINTM | CSR3_IDONM)
#define CSR3_DXSUFLO		(1<<6)	/* disable xmit stop on underflow error */
#define CSR3_LAPPEN			(1<<5)	/* enable look ahead packet procesing */
#define CSR3_DXMT2PD		(1<<4)	/* disable xmit two part deferral */
#define CSR3_EMBA			(1<<3)	/* enable modified backoff algorithm */
#define CSR3_BSWP			(1<<2)	/* byte swap (FIFO access only) */

#define CSR4_TXDPOLL		(1<<12)	/* disable tx polling */
#define CSR4_APAD_XMT		(1<<11)	/* autopad transmission */
#define CSR4_ASTRP_RCV		(1<<10) /* autostrip padded reception */
#define CSR4_MFCO			(1<<9)	/* status: missed frame counter overflow */
#define CSR4_MFCOM			(1<<8)	/* mask missed frame cnt ovfl irq */
#define CSR4_UINTCMD		(1<<7)	/* user irq command */
#define CSR4_UINT			(1<<6)	/* status: user command irq */
#define CSR4_RCVCCO			(1<<5)	/* status: rx collision counter overflow */
#define CSR4_RCVCCOM		(1<<4)	/* mask rx collision cnt ovfl irq */
#define CSR4_TXSTRT			(1<<3)	/* status: xmission started */
#define CSR4_TXSTRTM		(1<<2)	/* mask TXSTRT irq */
#define CSR4_IRQ_MSK		(CSR4_MFCOM | CSR4_RCVCCOM | CSR4_TXSTRTM)
#define CSR4_IRQ_STAT		(CSR4_MFCO | CSR4_UINT | CSR4_RCVCCO | CSR4_TXSTRT)

#define CSR5_TOKINTD		(1<<15)	/* disable xmit ok irq */
#define CSR5_LTINTEN		(1<<14)	/* enable last xmit irq */
#define CSR5_SINT			(1<<11)	/* status: system error (bus master) irq */
#define CSR5_SINTE			(1<<10)	/* enable system error irq */
#define CSR5_EXDINT			(1<<7)	/* status: excessive deferral irq */
#define CSR5_EXDINTE		(1<<6)	/* enable excessive deferral irq */
#define CSR5_MPPLBA			(1<<5)	/* magic packet physical logical broadcast */
#define CSR5_MPINT			(1<<4)	/* status: magic packet irq */
#define CSR5_MPINTE			(1<<3)	/* enable magic packet irq */
#define CSR5_MPEN			(1<<2)	/* enable magic packet */
#define CSR5_MPMODE			(1<<1)	/* magic packet mode */
#define CSR5_SPND			(1<<0)	/* request entrance into suspend mode */
#define CSR5_IRQ_STAT		(CSR5_SINT | CSR5_EXDINT | CSR5_MPINT)

#define CSR7_FASTSPNDE		(1<<15)	/* fast suspend enable */
#define CSR7_RXFRTG			(1<<14)	/* receive frame tag */
#define CSR7_RDMD			(1<<13)	/* receive demand (rx poll immediately) */
#define CSR7_RXDPOLL		(1<<12)	/* disable rx polling */
#define CSR7_STINT			(1<<11)	/* status: software timer interrupt */
#define CSR7_STINTE			(1<<10)	/* enable software timer irq */
#define CSR7_MREINT			(1<<9)	/* status: phy management read error irq */
#define CSR7_MREINTE		(1<<8)	/* enable phy management error irq */
#define CSR7_MAPINT			(1<<7)	/* status: phy management auto poll irq (status change) */
#define CSR7_MAPINTE		(1<<6)	/* enable phy management auto poll irq */
#define CSR7_MCCINT			(1<<5)	/* status: phy management command complete irq */
#define CSR7_MCCINTE		(1<<4)	/* enable MCCINT */
#define CSR7_MCCIINT		(1<<3)	/* status: phy management command complete internal irq */
#define CSR7_MCCIINTE		(1<<2)	/* enable MCCIINT */
#define CSR7_MIIPDTINT		(1<<1)	/* status: phy detect transition irq */
#define CSR7_MIIPDTINTE		(1<<0)	/* enable MIIPDTINT */
#define CSR7_IRQ_STAT  		(CSR7_STINT | CSR7_MREINT | CSR7_MAPINT | CSR7_MCCINT | CSR7_MCCIINT | CSR7_MIIPDTINT)

#define CSR15_PROM			(1<<15) /* promiscuous mode */
#define CSR15_DRCVBC		(1<<14)	/* disable receive broadcast */
#define CSR15_DFRCVPA		(1<<13)	/* disable receive physical address */
#define CSR15_PORTSEL		(3<<7)	/* select network medium (must be 11b) */
#define CSR15_INTL			(1<<6)	/* internal loopback */
#define CSR15_DRTY			(1<<5)	/* disable retry */
#define CSR15_FCOLL			(1<<4)	/* force collision */
#define CSR15_DXMTFCS		(1<<3)	/* disable transmit crc (FCS) */
#define CSR15_LOOP			(1<<2)	/* loopback enable */
#define CSR15_DTX			(1<<1)	/* disable transmitter */
#define CSR15_DRX			(1<<0)	/* disable receiver */

#define CSR80_RCVFW_16		(0<<12)	/* rxc fifo watermark = 16 */
#define CSR80_RCVFW_64		(1<<12)	/* rxc fifo watermark = 64 */
#define CSR80_RCVFW_112		(2<<12)	/* rxc fifo watermark = 112 */
#define CSR80_XMTSP_MIN		(0<<10) /* tx start point 20 (36 if SRAM_SIZE > 0) */
#define CSR80_XMTSP_64		(1<<10) /* tx start point 64 */
#define CSR80_XMTSP_128		(2<<10) /* tx start point 128 */
#define CSR80_XMTSP_MAX		(3<<10) /* tx start point 220 (full packet if NOUFLO is set) */
#define CSR80_XMTFW_16		(0<<8)	/* tx fifo watermark (for DMA request) = 16 */
#define CSR80_XMTFW_64		(1<<8)	/* tx fifo watermark (for DMA request) = 64 */
#define CSR80_XMTFW_108		(2<<8)	/* tx fifo watermark (for DMA request) = 108 */
	
#define CSR100_MERRTO_MASK	((1<<16)-1)	/* PCI latency timer in .1us */

#define CSR124_RPA			(1<<3)	/* runt packet accept */


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
#ifndef HAVE_LIBBSPEXT
	rtems_irq_connect_data	brokenbydesign;
	struct AmdEthDevRec_	*next;			/* devices sharing a common interrupt */
#endif
	LanceRegs32	baseAddr;
	int			irqLine;
	RxBufDescU	rdesc[1];	/* one descriptor */
	TxBufDescU	tdesc[2];	/* two descriptors; one for the header, one for the data */
	EtherHeaderRec	theader;
	PSemaId		sync;
	int			flags;
	unsigned	rmode, tmode;
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
		unsigned long	miiIrqs;
		unsigned long	linkStat;
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
#define CSR5_SETUP	(CSR5_TOKINTD | CSR5_EXDINTE)
#ifndef AUTOPOLL_BROKEN
#define CSR7_SETUP	(CSR7_RTFLAGS | CSR7_MAPINTE)
#else
#define CSR7_SETUP	(CSR7_RTFLAGS)
#endif
#define CSR15_SETUP	(CSR15_RTFLAGS | CSR15_PORTSEL)

#ifndef HAVE_LIBBSPEXT
static int  amdEthIntIsOn();
static void amdEthIntEnable();
static void amdEthIntDisable();
#endif

static void amdEthIsr();
int amdEthUpdateTxStats(AmdEthDev d, unsigned i);
static unsigned long long read_counter(AmdEthDev d, int csr, volatile unsigned long *pHi);

static AmdEthDevRec devices[NUM_ETH_DEVICES]={
{
#ifndef HAVE_LIBBSPEXT
  {0},
#endif
  0
},
};

#ifndef HAVE_LIBBSPEXT
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
#endif

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

#define RAPDECL register volatile unsigned long *rap = &d->baseAddr->rap
#define RDPDECL register volatile unsigned long *rdp = &d->baseAddr->rdp
#define BDPDECL register volatile unsigned long *bdp = &d->baseAddr->bdp

#define WBCR(idx, val) WriteIndReg(val,idx,rap,bdp)
#define WBCR1(val) wrle(val,bdp) /* write, using old rap */
#define RBCR(idx) ReadIndReg(idx,rap,bdp)
#define WCSR(idx, val) WriteIndReg(val,idx,rap,rdp)
#define WCSR1(val) wrle(val,rdp) /* write, using old rap */
#define RCSR(idx) ReadIndReg(idx,rap,rdp)

#define GET_LINK_STAT() (WBCR(33, ((0x1e<<5)|24)),RBCR(34))

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

static inline int
updateRxStats(AmdEthDev d, unsigned long rxstat)
{

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
	return 0;
}

/*static inline*/ void
yieldRx( AmdEthDev d, unsigned long rxstat )
{
RAPDECL;
RDPDECL;
	wrle(rxstat | RXDESC_STAT_OWN, &d->rdesc[0].STYLE.statLE);
	/* yield the descriptor before enforcing a poll */
	__asm__ __volatile__("eieio");
	WCSR1( (RCSR(7) & ~CSR7_IRQ_STAT) | CSR7_RDMD );
}

#ifdef __PPC__
#define ISR_PROF_DEPTH	10
unsigned long amdEthMaxIsrTicks = 0;
unsigned long amdEthIsrProfile[ISR_PROF_DEPTH]={};
#define ISR_PROF_INC()	asm volatile("mftb %0":"=r"(isrProf[i++]))
#endif

static void
amdEthInterruptWork(AmdEthDev d)
{
register unsigned long	csr0, csr5, tmp;
unsigned long			savedAR;
int						bogusIrq = 1;
#ifdef __PPC__
unsigned long			i=0;
unsigned long			isrProf[ISR_PROF_DEPTH];

	ISR_PROF_INC();
#endif


#ifndef HAVE_LIBBSPEXT
	for ( ; d; d=d->next )
#endif
	{
	RAPDECL;
	RDPDECL;
		/* save the rap before changing it (don't care about endianness) */
		ISR_PROF_INC();
		savedAR = *rap;

		ISR_PROF_INC();
		if ( ((csr0=RCSR(0)) & CSR0_INTR) ) {
			/* to check: EXDINT, IDON, MERR, MISS, MFCO, RCVCCO, RINT,
			 * done (lowercase - x: unused):
			 *              def   x    merr  miss  mfco  rcvcco  rint
			 * register:
			 *             csr5  csr0  csr0  csr0   csr4   csr4  csr0 
			 *           SINT, TINT, TXSTRT, UINT, STINT, MREINT, MCCINT,
			 *           sint     x       x     x      x       x       x
			 *           csr5  csr0    csr4  csr4   csr7   csr7   csr7 
			 *           MIIPDTINT, MAPINT, MCCIINT, MPINT
			 *                   x     map       x     x
			 *                 csr7    csr7    csr7  csr5
			 */
			ISR_PROF_INC();
			d->stats.irqs++;
			bogusIrq = 0;
#if 0
			if ( csr0 & CSR0_MISS ) {
				/* missed frame - we use the hardware counter but action
				 * could be taken here also, maybe post a special event...
				 */
			}
#endif
			if ( csr0 & CSR0_MERR ) {
				/* memory / PCI bus error */
				d->stats.rxMemory++;
			}

			/* TS interrupt not used */
#if 0	
		/* We post to the receiver in any case the controller has
		 * released the descriptor. (NO)
		 */
#endif
			if ( csr0 & CSR0_RINT ) {
				/* received a packet, post an event */
				register unsigned long dstat = rdle(&d->rdesc[0].STYLE.statLE);
				if ( ! (dstat & RXDESC_STAT_OWN) ) {
					/* TSILL */
					unsigned long now;
					__asm__ __volatile__("mftb %0":"=r"(now));
					roundtrip = now-roundtrip;

					switch ( d->rmode ) {
						default:
						break;
						case AMDETH_FLG_RX_MODE_AUTO:
							/* probably cheaper to do this from ISR context
							 * than posting a semaphore...
							 */
							updateRxStats(d, dstat);
							yieldRx(d,dstat);
						break;
						case AMDETH_FLG_RX_MODE_SYNC:
							pSemPost(&d->sync);
						break;
					}
				}
			}
			ISR_PROF_INC();
			/* clear raised flags */
	 		WCSR(0,csr0);

			/* handle user and the counter rollover interrupts */
			tmp=RCSR(4);
			if (tmp & CSR4_MFCO)
				d->stats.rxOverrunHi++;
			if (tmp & CSR4_RCVCCO)
				d->stats.rxCollisionHi++;
			/* clear raised flags */
			WCSR(4,tmp);
			ISR_PROF_INC();

			csr5=RCSR(5);
			if (csr5 & CSR5_SINT) {
				/* this is probably really bad */
				d->stats.sysError++;
				/* TODO we should do something here */
			}

			if (   ((CSR0_TINT & csr0) || (CSR5_EXDINT & csr5))
			    && (AMDETH_FLG_TX_MODE_AUTO == AMDETH_FLG_TX_MODE(d->flags)) ) {
					amdEthUpdateTxStats(d,(unsigned)-1);
			}
			WCSR(5,csr5);
			ISR_PROF_INC();

#ifndef AUTOPOLL_BROKEN
			tmp = RCSR(7);
			if ( tmp & CSR7_MAPINT ) {
				unsigned long	savedMIIAR;
				BDPDECL;
				d->stats.miiIrqs++;
				savedMIIAR = RBCR(33);
				/* get summary status */
				d->stats.linkStat = GET_LINK_STAT();
				WBCR(33, savedMIIAR);
				/* clear raised flag */
				WCSR(7, tmp);
			}
#endif
		}

		/* restore address register contents */
		d->baseAddr->rap=savedAR;
	}
	amdEthBogusIrqs += bogusIrq;
#ifdef __PPC__
	/* do profiling */
	ISR_PROF_INC();
	tmp=isrProf[i-1] - isrProf[0];
	if ( amdEthMaxIsrTicks < tmp ) {
		amdEthMaxIsrTicks = tmp;
		for ( tmp=0; tmp<i-1; tmp++ ) {
			amdEthIsrProfile[tmp] = isrProf[tmp+1]-isrProf[tmp];
		}
		amdEthIsrProfile[tmp]=0xdeadbeef;
	}
#endif
}

static void
amdEthIsr(AmdEthDev	d)
{
amdEthInterruptWork(d);
}

static inline void
intDoEnable(AmdEthDev d)
{
RAPDECL;
RDPDECL;
	WCSR1( (RCSR(0) &~CSR0_IRQ_STAT) | CSR0_IENA);
	WCSR1( (RCSR(5) &~CSR5_IRQ_STAT) | CSR5_SINTE);
}

static inline void
intDoDisable(AmdEthDev d)
{
RAPDECL;
RDPDECL;
	WCSR1( (RCSR(0) &~CSR0_IRQ_STAT) & ~CSR0_IENA);
	WCSR1( (RCSR(5) &~CSR5_IRQ_STAT) & ~CSR5_SINTE);
}


#ifndef HAVE_LIBBSPEXT
static int
amdEthIntIsOn(const rtems_irq_connect_data *arg)
{
int			i;
AmdEthDev	d;
	for (i = 0; i < NumberOf(insaneApiMap); i++) {
		if ( (d = insaneApiMap[i].device) &&
			  d->brokenbydesign.name == arg->name) {
			RAPDECL;
			RDPDECL;
			return	(RCSR(0) & CSR0_IENA) || (RCSR(5) & CSR5_SINTE);
		}
	}
	return 0;
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
				intDoEnable(d);
				d = d->next;
			} while (d);
			return;
		}
	}
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
				intDoDisable(d);
				d = d->next;
			} while (d);
			return;
		}
	}
}
#endif

int
amdEthReceivePacket(AmdEthDev d, char *buf, int len)
{
int						rval;
unsigned long			mode;

	if ( ! (mode=AMDETH_FLG_RX_MODE(d->flags)) )
		return AMDETH_ERROR;

	/* can't call this again if in AUTO_RX mode */
	if ( (rdle(&d->rdesc[0].STYLE.statLE) & RXDESC_STAT_OWN) ||
	     ((AMDETH_FLG_RX_MODE_AUTO == mode) && d->rdesc[0].STYLE.rbadrLE )  )
		return AMDETH_BUSY;

	/* setup the RX descriptor */
	wrle(LOCAL2PCI(buf),&d->rdesc[0].STYLE.rbadrLE);

	/* make sure buffer address is written
	 * before yielding the descriptor
	 */
	__asm__ __volatile__("eieio");


	switch ( mode ) {
		case AMDETH_FLG_RX_MODE_SYNC:

			yieldRx(d, RXDESC_STAT_ONES | (-len & RXDESC_STAT_BCNT_MSK));

			/* block on an event */
			if ( pSemWait(&d->sync) ) {
				/* semaphore destroyed; device was closed */
				return -1;
			}

			/* update stats and return length */
			if ( ! (rval = updateRxStats(d, rdle(&d->rdesc[0].STYLE.statLE))) )
				rval = ((int)rdle(&d->rdesc[0].STYLE.mcntLE)) & RXDESC_MCNT_MASK;
			return rval;

		case AMDETH_FLG_RX_MODE_POLL:

			if ( ! (rval = updateRxStats(d, rdle(&d->rdesc[0].STYLE.statLE))) )
				rval = ((int)rdle(&d->rdesc[0].STYLE.mcntLE)) & RXDESC_MCNT_MASK;

			yieldRx(d, RXDESC_STAT_ONES | (-len & RXDESC_STAT_BCNT_MSK));

			return rval;

		case AMDETH_FLG_RX_MODE_AUTO:
			{
			RAPDECL;
			RDPDECL;
			/* activate RX auto polling ? */
			WCSR1(RCSR(7) & ~ (CSR7_RXDPOLL|CSR7_IRQ_STAT));
			yieldRx(d, RXDESC_STAT_ONES | (-len & RXDESC_STAT_BCNT_MSK));
			}
			return 0;

		default:
			break;
	}
	/* should never get here */
	return AMDETH_ERROR;
}

int
amdEthInit(AmdEthDev *pd, int instance, int flags)
{
int		bus,dev,fun,i,irqLine=-1;
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

	if ( (d->tmode=AMDETH_FLG_TX_MODE(flags)) > AMDETH_FLG_TX_MODE_POLL ) {
		fprintf(stderr,"Invalid TX mode\n");
		return AMDETH_ERROR;
	}
	if ( (d->rmode=AMDETH_FLG_RX_MODE(flags)) > AMDETH_FLG_RX_MODE_SYNC ) {
		fprintf(stderr,"Invalid RX mode\n");
		return AMDETH_ERROR;
	}

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
	irqLine=(unsigned char)tmpb;
	fprintf(stderr,"%s Lance PCI ethernet found at 0x%08x (IRQ %i),",
			dn, (unsigned int)d->baseAddr, irqLine);
	amdEthPrintAddr(d, stderr, " hardware address: ");
	fputc('\n',stderr);

	/* make sure we have bus master access */
	pciConfigInLong(bus,dev,fun,PCI_COMMAND,&tmp);
	tmp|=PCI_COMMAND_MASTER;
	pciConfigOutLong(bus,dev,fun,PCI_COMMAND,tmp);

	/* program the PCI latency timer - we're not nice
	 * but hang on to the bus as long as possible
	 * for sake of determinism
	 */
	pciConfigOutByte(bus,dev,fun,PCI_LATENCY_TIMER,0xff);

	/* switch to 32bit io */
	d->baseAddr->rdp = 0x0; /* 0 is endian-safe :-) */
	__asm__ __volatile__("eieio");

	/* initialize BCR regs */
	{
		RAPDECL;
		RDPDECL;
		BDPDECL;

		WBCR(20, STYLEFLAGS);

		/* we really assume that there is an eeprom present providing some
		 * initialization.
		 * Note that we also assume that sufficient time has expired
		 * since HW reset until execution reaches this point for the
		 * controller to have read the EEPROM.
		 * Also, we assume that the EEPROM contains reasonable defaults
		 * for some BCRs (e.g., BCR18 enabling PCI burst modes)...
	 	 */
		{ register unsigned long lanceBCR19;
		lanceBCR19=RBCR(19);
		assert( lanceBCR19 & BCR19_PVALID );
		}

#if 0 /* not yet ready - unsure if really needed */
		/* set LOLATRX (requires SRAM > 0, CSR127(RPA) ) */
		WBCR1(RBCR(27) | BCR27_LOLATRX);
#endif

		/* clear interrupt and mask unneeded sources */
		WCSR(0,  CSR0_SETUP | CSR0_IRQ_STAT);

		WCSR(3,  CSR3_SETUP | (AMDETH_FLG_RX_MODE_POLL == d->rmode ? CSR3_RINTM : 0 ));

		WCSR(4,  CSR4_SETUP | CSR4_IRQ_STAT);

		WCSR(5,  CSR5_SETUP | CSR5_IRQ_STAT);

		WCSR(7, CSR7_SETUP | CSR7_IRQ_STAT);

		/* switch off receiver ? */
		tmp = CSR15_SETUP;
		if ( d->tmode == AMDETH_FLG_TX_MODE_OFF ) {
			tmp |= CSR15_DTX;
		}
		if ( d->rmode == AMDETH_FLG_RX_MODE_OFF ) {
			tmp |= CSR15_DRX;
		}
		if ( flags & AMDETH_FLG_NOBCST ) {
			tmp |= CSR15_DRCVBC;
		}
		WCSR(15, tmp);

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
		printf("TSILL tdesc is at %8p (PCI 0x%08x)\n",d->tdesc,LOCAL2PCI(d->tdesc));
		WCSR(30, ((LOCAL2PCI(d->tdesc) &0xffff)));
		WCSR(31, ((LOCAL2PCI(d->tdesc) >> 16)&0xffff));
		WCSR(78, ((-(long)NumberOf(d->tdesc)) & 0xffff)); /* TX ring length */
		WCSR(47, 0); /* default TX polling interval */

		WCSR(100,CSR100_MERRTO_MASK & CSR100_SETUP);

	/* initialize the buffer descriptors */
	{
		/* fill the ethernet header */
		/* send to broadcast */
		amdEthHeaderInit(&d->theader, 0, d);
		/* broadcast address */
		memset(d->theader.dst, 0xff, sizeof(d->theader.dst));
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

		/* I don't know of a way how to detect if the card has
		 * a fiber or copper interface. Unfortunately, some
 		 * settings need to be different and hence YOU tell ME
 		 */
		if ( AMDETH_FLG_FIBER & flags ) {
			/* Scrambler/descrambler needs to be disabled (datasheet).
			 * If I don't do this, the link still works but experiences
			 * quite high bit errors (CRC errors)!
			 */
			WBCR1( RBCR(2) | BCR2_DISSCR_SFEX );
		} else {
			WBCR1( RBCR(2) & ~BCR2_DISSCR_SFEX );
		}

#ifndef AUTOPOLL_BROKEN
		/* Enable MII auto polling */
		WBCR1( RBCR(32) | BCR32_APEP );
#endif
		/* obtain initial link status */
		d->stats.linkStat = GET_LINK_STAT();

		/* finally, start the device */
		WCSR(0, CSR0_SETUP | CSR0_STRT);
	}

	/* RTEMS binary semaphores allow nesting and are not
	 *       suited for task synchronization
	 */
	if ( AMDETH_FLG_RX_MODE_SYNC==d->rmode )
		pSemCreate( 0/* binary */, 0 /* empty */, &d->sync);

	/* install ISR */
#ifdef HAVE_LIBBSPEXT
	if ( bspExtInstallSharedISR(BSP_PCI_IRQ_LOWEST_OFFSET + irqLine, (void (*)(void*))amdEthIsr, (void*)d, 0) ) {
		fprintf(stderr,"Unable to connect to interrupt\n");
		return -1;
	}
	d->irqLine = irqLine;
	intDoEnable(d);
#else

	d->brokenbydesign.on=amdEthIntEnable;
	d->brokenbydesign.off=amdEthIntDisable;
	d->brokenbydesign.isOn=amdEthIntIsOn;
	d->brokenbydesign.name=BSP_PCI_IRQ0 + (d->irqLine = irqLine);

	/* is an ISR for our line already installed ? */
	for (i = NumberOf(insaneApiMap) - 1; i >=0; i-- ) {
		AmdEthDev	d1;
		if ( (d1 = insaneApiMap[i].device) &&
			  d1->brokenbydesign.name == d->brokenbydesign.name) {
			/* link into existing entry */
			d->next = d1;
			insaneApiMap[i].device = d;
			d->brokenbydesign.hdl  = insaneApiMap[i].isr;
			intDoEnable(d);
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
#endif


	if (pd) *pd=d;

	return AMDETH_OK;
}

/* update statistics and clear TX buffer errors
 * RETURNS: 0 if there were no errors, AMDETH_ERR
 *          if there were errors, AMDETH_BUSY if
 *          the descriptor pair is busy.
 */
int
amdEthUpdateTxStats(AmdEthDev d, unsigned i)
{
unsigned long	csr1,csr2,end;
int		rval = 0;

	if ( i >= NumberOf(d->tdesc) ) {
		i = 0; end = NumberOf(d->tdesc);
	} else {
		end = i+1;
	}

	for ( ; i<end; i++) {

	csr1=rdle(&d->tdesc[i].STYLE.csr1LE);

	if ( (csr1 & TXDESC_CSR1_OWN) ) {
		rval = AMDETH_BUSY;
		continue;
	}

	if ( ! (csr1 & TXDESC_ERRS) )
		continue;

	/* update statistics */
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

	if (
#ifdef RT_DRIVER
		(csr1 & TXDESC_CSR1_DEF) ||
#endif
		(csr2 & TXDESC_CSR2_EXDEF) ) {
		
		d->stats.txDeferred++;
	}

	/* reset error bit */
	wrle( csr1 & ~ TXDESC_ERRS,
		&d->tdesc[i].STYLE.csr1LE );
	rval = AMDETH_ERROR;
	}

	return rval;
}

/* send a packet; if no header is specified, use the default
 * broadcast header
 */

int
amdEthSendPacket(AmdEthDev d, EtherHeader h, void *data, int size)
{
register unsigned long csr1OR;
int l = sizeof(*h);

	if (!d) d=&devices[0];

	if ( ! d->tmode )
		return AMDETH_ERROR;

	csr1OR=rdle(&d->tdesc[0].STYLE.csr1LE) | rdle(&d->tdesc[1].STYLE.csr1LE);
	if ( csr1OR & TXDESC_CSR1_OWN )
		return AMDETH_BUSY;

	/* previous transmission gave an error */
	if ( csr1OR & TXDESC_ERRS )
		return AMDETH_ERROR;
	
	/* do statistics */
	d->stats.txPackets++;

	if (!h) {
		h=&d->theader;
		l=sizeof(d->theader);
	}
	((EtherHeader)h)->len = htons(l - 14  + size);
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
	wrle((-l & TXDESC_CSR1_BCNT_MSK) |
		TXDESC_CSR1_OWN | TXDESC_CSR1_STP | TXDESC_CSR1_ONES,
		&d->tdesc[0].STYLE.csr1LE);
	__asm__ __volatile__("eieio");
	{
	RAPDECL;
	RDPDECL;
	/* TSILL */
	unsigned long flags;
	/* demand transmission without changing status bits - we
	 * hope nobody (ISR or other thread) modifies the IENA flag while we are
	 * tampering with this register...
	 */
	rtems_interrupt_disable(flags);
	WCSR1((RCSR(0) & ~(CSR0_IRQ_STAT)) | CSR0_TDMD);
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
		fprintf(f,"PHY status:\n");
		fprintf(f,"  interrupts:     %li\n",    d->stats.miiIrqs);
		fprintf(f,"        link:     %s, 10%s-%s\n",
				d->stats.linkStat & ANR24_LINK_UP   ? "up"   : "DOWN",
				d->stats.linkStat & ANR24_SPEED_100 ? "0"    : "",
				d->stats.linkStat & ANR24_FULLDUP   ? "full" : "half");
	}

	fprintf(f,"%i bogus IRQs detected\n",       amdEthBogusIrqs);
	return 0;
}


/* routines for debugging / testing */
unsigned long
lance_read_csr(int offset,AmdEthDev d)
{
unsigned long v, flags;
if ( !d ) d = &devices[0];
	rtems_interrupt_disable(flags);
	v = ReadIndReg(offset,&d->baseAddr->rap,&d->baseAddr->rdp);
	rtems_interrupt_enable(flags);
	return v;
}

void
lance_write_csr(unsigned long val, int offset, AmdEthDev d)
{
unsigned long flags;
if ( !d ) d = &devices[0];
	rtems_interrupt_disable(flags);
	{
	RAPDECL;
	unsigned long saved=*rap;
	WriteIndReg(val,offset,rap,&d->baseAddr->rdp);
	*rap = saved;
	}
	rtems_interrupt_enable(flags);
}

unsigned long
lance_read_bcr(int offset, AmdEthDev d)
{
unsigned long v, flags;
if ( !d ) d = &devices[0];
	rtems_interrupt_disable(flags);
	v = ReadIndReg(offset,&d->baseAddr->rap,&d->baseAddr->bdp);
	rtems_interrupt_enable(flags);
	return v;

}

void
lance_write_bcr(unsigned long val, int offset, AmdEthDev d)
{
unsigned long flags;
if ( !d ) d = &devices[0];
	rtems_interrupt_disable(flags);
	WriteIndReg(val,offset,&d->baseAddr->rap,&d->baseAddr->bdp);
	rtems_interrupt_enable(flags);
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
RAPDECL;
RDPDECL;

		lo_1 = RCSR(csr_no);
		hi   = *pHi;
		lo_2 = RCSR(csr_no);

		if (lo_2 < lo_1) {
			/* overflow has just occurred; re-read Hi */
			hi = *pHi;
		}
		return (hi<<16)|lo_2;
}

void
_cexpModuleInitialize(void *mod)
{
int i;
	for ( i=0; i<NumberOf(devices); i++ ) {
		devices[i].irqLine = -1;
	}
}

int amdEthCloseDev(AmdEthDev d)
{
	if ( d < devices || d >= devices + NumberOf(devices) ) {
		fprintf(stderr,"Invalid device pointer\n");
		return -1;
	}
	if ( !d->baseAddr ) {
		fprintf(stderr,"Device not open\n");
		return -1;
	}
{
RAPDECL;
RDPDECL;
	/* STOP DMA */
	WCSR( 0, CSR0_SETUP | CSR0_STOP);
	/* disable interrupts */
}
	intDoDisable(d);
	if (d->sync) {
		/* delete sema; release blocked task with error  */
		pSemDestroy(&d->sync);
	}
#ifdef HAVE_LIBBSPEXT
	if ( d->irqLine >= 0 ) {
		bspExtRemoveSharedISR(BSP_PCI_IRQ_LOWEST_OFFSET + d->irqLine, (void (*)(void*))amdEthIsr, d);
	}
#endif
	memset(d,0,sizeof(*d)); /* mark slot free */
	return 0;
}


int
_cexpModuleFinalize(void *mod)
{
int		 	i;
AmdEthDev	d;
	
#ifdef HAVE_LIBBSPEXT
	for ( d=devices, i=0; i<NumberOf(devices); i++, d++ ) {
		if ( d->baseAddr ) {
			amdEthCloseDev(d);
		}
	}
#else
	for ( i=0; i < NumberOf(insaneApiMap); i++) {
		for ( d=insaneApiMap[i].device; d; d=d->next) {
			amdEthCloseDev(d);
		}
		/* uninstall ISR */
		BSP_remove_rtems_irq_handler(&insaneApiMap[i].device->brokenbydesign);
	}
#endif
	/* hack; allow released tasks to finish */
	sleep(1);
	return 0;
}

int
amdEthGetHeaderSize()
{
	return sizeof(EtherHeaderRec);
}
