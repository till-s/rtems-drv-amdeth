/* $Id$ */

#include <assert.h>
#include <stdio.h>

#include "amdeth.h"

#ifdef __rtems
#include <rtems.h>
#include <bsp/pci.h>
#include <libcpu/io.h>
/* let bsp.h fix the _ISA_MEM_BASE & friends symbols */
#warning "TODO fix _ISA_MEM_BASE hack"
#include <bsp.h>
#include "bspExt.h"
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr) + _ISA_MEM_BASE)
#define LOCAL2PCI(memaddr) ((pci_ulong)(memaddr) + PCI_DRAM_OFFSET)
#define pciFindDevice BSP_pciFindDevice
#define pciConfigInLong pci_read_config_dword
#define pciConfigInByte pci_read_config_byte

#elif defined(__vxworks)
#include <vxWorks.h>
//#define PCI_VENDOR_ID_AMD	0x1022
//#define PCI_DEVICE_ID_AMD_LANCE	0x2000
#define PCI2LOCAL(pciaddr) ((pci_ulong)(pciaddr))
#else
#error "Unknown Architecture"
#endif

#ifdef CPU_BIG_ENDIAN
#ifdef __PPC
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
#define CSR0_MISS	(1<<12)	/* status: missed frame */
#define CSR0_MERR	(1<<11)	/* status: memory error */
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
#define CSR4_MFCO	(1<<9)	/* missed frame counter overflow */
#define CSR4_MFCOM	(1<<8)	/* mask missed frame cnt ovfl irq */
#define CSR4_UINTCMD	(1<<7)	/* user irq command */
#define CSR4_UINT	(1<<6)	/* status: user command irq */
#define CSR4_RCVCCO	(1<<5)	/* rx collision counter overflow */
#define CSR4_RCVCCOM	(1<<4)	/* mask rx collision cnt ovfl irq */
#define CSR4_TXSTRT	(1<<3)	/* status: xmission started */
#define CSR4_TXSTRTM	(1<<2)	/* mask TXSTRT irq */
#define CSR4_IRQ_MSK	(CSR4_MFCOM | CSR4_RCVCCOM | CSR4_TXSTRTM)

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


typedef struct LanceRegs32Rec_ {
	volatile unsigned char aprom[16];
	volatile unsigned long rdp;
	volatile unsigned long rap;
	volatile unsigned long reset;
	volatile unsigned long bdp;
} LanceRegs32Rec, *LanceRegs32;


/* RX buffer descriptor
 * Note that the Am79C973 uses this area LITTLE ENDIAN
 * and that it's different, depending on the software
 * style used!
 * The descriptor ring must be 16byte aligned! 
 */
typedef union RxBufDescU_ {
	struct {
		unsigned long	rbadrLE __attribute__ ((aligned(16)));/* buffer address */
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
} RxBufDescU, *RxBufDesc;

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
		unsigned long	tbadrLE __attribute__ ((aligned(16)));/* buffer address */
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
} TxBufDescU, *TxBufDesc;

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
	unsigned char	src[2];
	unsigned short	len;	/* network byte order */
	struct	{
		unsigned char dsap;
		unsigned char ssap;
		unsigned char ctrl;
	} llc8022;
	struct  {
		unsigned char org[3];
		unsigned short type;
	} snap;
} EtherHeaderRec, *EtherHeader;

/* TODO add mutex for driver access */
typedef struct AmdEthDevRec_ {
	LanceRegs32	baseAddr;
	int		irqLine;
	RxBufDescU	rdesc[1];	/* one dummy descriptor */
	TxBufDescU	tdesc[2];	/* two descriptors; one for the header, one for the data */
	EtherHeaderRec	header;
	struct		{
	}		stats;
} AmdEthDevRec;

static AmdEthDevRec defDev={0};

#ifdef RXTEST
static unsigned char rxbuffer[NumberOf(defDev.rdesc)][2048];
#endif

/* read and write a data / bus registers in the lance.  */
/* NOTE: registers should be mapped to guarded memory. Hence, 
 *       stores are inherently in order. We must enforce ordering
 *       of the lwbrx instruction below with respect to the stwbrx, however.
 */
#define WriteIndReg(value, idx, areg, dreg) __asm__ __volatile__(\
		"stwbrx %1, 0, %2; stwbrx %0, 0, %3": :"r"(value),"r"(idx),"r"(areg),"r"(dreg))
#define ReadIndReg(value, idx, areg, dreg) __asm__ __volatile__(\
		"stwbrx %1, 0, %2; eieio; lwbrx %0, 0, %3":"=r"(value) :"r"(idx),"r"(areg),"r"(dreg))


static char *dn="Amd Ethernet:";


int
amdEthInit(AmdEthDev *pd, int instance)
{
int		bus,dev,fun,i;
pci_ulong	tmp;
pci_ubyte	tmpb;
AmdEthDev	d;


	/* for now, allow only one instance */
	assert(instance==1 && !defDev.baseAddr);
	d=&defDev;

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
	pciConfigInByte(bus,dev,fun,PCI_INTERRUPT_LINE,&tmpb);
	d->irqLine=(unsigned char)tmpb;
	fprintf(stderr,"%s Lance PCI ethernet found at 0x%08x (IRQ %i),",
			dn, (unsigned int)d->baseAddr, d->irqLine);
	fprintf(stderr," hardware address: ");

	for (i=0; i<6; i++) {
		fprintf(stderr,"%02x%c",
				((unsigned char *)d->baseAddr)[i],
				i==5 ? '\n' : ':');
	}

	/* switch to 32bit io */
	d->baseAddr->rdp = 0x0; /* 0 is endian-safe :-) */

	/* initialize BCR regs */
	{
		register unsigned long rap = (unsigned long)&d->baseAddr->rap;
		register unsigned long bdp = (unsigned long)&d->baseAddr->bdp;
		register unsigned long rdp = (unsigned long)&d->baseAddr->rdp;
#define WBCR(idx, val) WriteIndReg(val,idx,rap,bdp)
#define WCSR(idx, val) WriteIndReg(val,idx,rap,rdp)
		WBCR(20, STYLEFLAGS);

		/* we really assume that there is an eeprom present providing some
		 * initialization
	 	 */
		{ register unsigned long lanceBCR19;
		ReadIndReg(lanceBCR19, 19, rap, bdp);
		assert( lanceBCR19 & BCR19_PVALID );
		}

		/* mask all possible irq sources */
		WCSR(3,  CSR3_IRQ_MSK);

		/* transmit on demand only */
		WCSR(4,  CSR4_IRQ_MSK | CSR4_TXDPOLL);
		WCSR(5,  CSR5_TOKINTD);
		/* switch off receiver */
#ifndef RXTEST
		WCSR(7,  CSR7_RXDPOLL);
		WCSR(15, CSR15_DRX);
#endif

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
		 WCSR(30, ((LOCAL2PCI(d->tdesc) &0xffff)));
		 WCSR(31, ((LOCAL2PCI(d->tdesc) >> 16)&0xffff));
		 WCSR(78, ((-(long)NumberOf(d->tdesc)) & 0xffff)); /* TX ring length */
		 WCSR(47, 0); /* default TX polling interval */

	/* initialize the buffer descriptors */
	{
		int i;
		/* fill the ethernet header */
		/* send to broadcast */
		memset(d->header.dst, 0xff, sizeof(d->header.dst));
		memcpy(d->header.src, (void*)d->baseAddr->aprom, sizeof(d->header.src));
		d->header.llc8022.dsap = 0xAA;
		d->header.llc8022.ssap = 0xAA;
		d->header.llc8022.ctrl = 0x3;
		d->header.snap.org[2]  = 0xca;
		d->header.snap.type    = htons(0x805b); /* stanford */
		wrle(LOCAL2PCI(&d->header),&d->tdesc[0].STYLE.tbadrLE);
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
#endif
	}

		/* start the device */
		WCSR(0, CSR0_STRT);
	}
#undef WBCR
#undef WCSR
	if (pd) *pd=d;

	return AMDETH_OK;
}

int
amdEthBroadcast(AmdEthDev d,void *data, int size)
{
	if (!d) d=&defDev;

	if ( rdle(&d->tdesc[0].STYLE.csr1LE) & TXDESC_CSR1_OWN ||
	     rdle(&d->tdesc[1].STYLE.csr1LE) & TXDESC_CSR1_OWN)
		return AMDETH_BUSY;
	/* TODO do statistics */
	d->header.len          = sizeof(d->header) - 14  + size;
	/* clear csr2 */
	wrle(0,
		&d->tdesc[0].STYLE.csr2LE);
	wrle(0,
		&d->tdesc[1].STYLE.csr2LE);
	/* set buffer address of second bd */
	wrle(LOCAL2PCI(data), &d->tdesc[1].STYLE.tbadrLE);
	/* setup csr1 of second bd */
	/* NOTE: 2's complement of byte count! */
	wrle((-size & TXDESC_CSR1_BCNT_MSK) |
		TXDESC_CSR1_OWN | TXDESC_CSR1_ENP | TXDESC_CSR1_ONES,
		&d->tdesc[1].STYLE.csr1LE);
	/* yield 1st descriptor */
	wrle((-sizeof(d->header) & TXDESC_CSR1_BCNT_MSK) |
		TXDESC_CSR1_OWN | TXDESC_CSR1_STP | TXDESC_CSR1_ONES,
		&d->tdesc[0].STYLE.csr1LE);
	__asm__ __volatile__("eieio");
	/* demand transmission */
	WriteIndReg( CSR0_TDMD, 0, &d->baseAddr->rap, &d->baseAddr->rdp);
	return AMDETH_OK;
}

/* routines for debugging / testing */
unsigned long
lance_read_csr(int offset)
{
	ReadIndReg(offset,offset,&defDev.baseAddr->rap,&defDev.baseAddr->rdp);
	return offset;

}
void
lance_write_csr(unsigned long val, int offset)
{
	WriteIndReg(val,offset,&defDev.baseAddr->rap,&defDev.baseAddr->rdp);
}

unsigned long
lance_read_bcr(int offset)
{
	ReadIndReg(offset,offset,&defDev.baseAddr->rap,&defDev.baseAddr->bdp);
	return offset;

}
void
lance_write_bcr(unsigned long val, int offset)
{
	WriteIndReg(val,offset,&defDev.baseAddr->rap,&defDev.baseAddr->bdp);
}
