//-------------------------------------------------------------------
//	nic2.c	 (An enhanced version of our 'nic.c' device-driver)
//
//	This module implements a character-mode Linux device-driver 
//	for the Intel 82573L gigabit ethernet controller which uses
//	VLAN filtering to assist multiple developers in sharing the
//	concurrent access to our 'anchor-cluster' network stations. 
//
//	NOTE: Written and tested for Linux kernel version 2.6.22.5.
//
//	programmer: ALLAN CRUSE
//	written on: 20 MAR 2008
//-------------------------------------------------------------------

#include <linux/module.h>	// for init_module() 
#include <linux/proc_fs.h>	// for create_proc_info_entry() 
#include <linux/pci.h>		// for pci_get_device()
#include <linux/interrupt.h>	// for request_irq()
#include <linux/delay.h>	// for request_irq()
#include <asm/uaccess.h>	// for copy_from_user()

#define VENDOR_ID	0x8086	// Intel Corporation
//#define DEVICE_ID	0x109A	// 82573L controller
//#define DEVICE_ID	0x10B9	// 82572EI controller
#define DEVICE_ID	0x100F	// 82545EM controller
#define N_RX_DESC	16	// number of RX descriptors
#define N_TX_DESC	16	// number of TX descriptors
#define RX_BUFSIZ	2048	// size of RX packet-buffer
#define TX_BUFSIZ	1536	// size of TX packet-buffer
#define RX_MEMLEN	(RX_BUFSIZ + 16)*N_RX_DESC
#define TX_MEMLEN	(TX_BUFSIZ + 16)*N_TX_DESC
#define KMEM_SIZE	(RX_MEMLEN + TX_MEMLEN)
#define HDR_BYTES	(14+2)	// our packet-data preamble	
#define USUAL_MTU	1500	// standard ethernet MTU 
#define INTR_MASK   0xFFFFFFFF	// interrupt mask 

#define PROC_NAME "nicspy"
#define PROC_RX_NAME "nicrx"
#define PROC_TX_NAME "nictx"
#define RAL_MAC_ADDR_LEN 4
#define RAH_MAC_ADDR_LEN 2
#define MAX_ETH_ADDR_LEN (RAL_MAC_ADDR_LEN + RAH_MAC_ADDR_LEN)

typedef struct	{
		unsigned long long	base_address;
		unsigned short		packet_length;
		unsigned short		packet_chksum;
		unsigned char		desc_status;
		unsigned char		desc_errors;
		unsigned short		vlan_tag;
		}__attribute__((packed)) RX_DESCRIPTOR;

typedef struct	{
		unsigned long long	base_address;
		unsigned short		packet_length;
		unsigned char		cksum_offset;
		unsigned char		desc_command;
		unsigned char		desc_status;
		unsigned char		cksum_origin;
		unsigned short		special_info;
		}__attribute__((packed)) TX_DESCRIPTOR;





enum	{
	E1000_CTRL	= 0x0000,	// Device Control
	E1000_STATUS	= 0x0008,	// Device Status
	E1000_CTRL_EXT	= 0x0018,	// Device Control Extended
	E1000_VET	= 0x0038,	// VLAN Ether Type
	E1000_ICR	= 0x00C0,	// Interrupt Cause Read
	E1000_ICS	= 0x00C8,	// Interrupt Cause Set
	E1000_IMS	= 0x00D0,	// Interrupt Mask Set
	E1000_IMC	= 0x00D8,	// Interrupt Mask Clear
	E1000_RCTL	= 0x0100,	// Receive Control
	E1000_TCTL	= 0x0400,	// Transmit Control
	E1000_RDBAL	= 0x2800,	// Receive Descriptor Base Addr Low
	E1000_RDBAH	= 0x2804,	// Receive Descriptor Base Addr High
	E1000_RDLEN	= 0x2808,	// Receive Descriptor Length
	E1000_RDH	= 0x2810,	// Receive Descriptor Head
	E1000_RDT	= 0x2818,	// Receive Descriptor Tail
	E1000_RXDCTL	= 0x2828,	// Receive Descriptor Control
	E1000_TDBAL	= 0x3800,	// Transmit Descriptor Base Addr Low
	E1000_TDBAH	= 0x3804,	// Transmit Descriptor Base Addr High
	E1000_TDLEN	= 0x3808,	// Transmit Descriptor Length
	E1000_TDH	= 0x3810,	// Transmit Descriptor Head
	E1000_TDT	= 0x3818,	// Transmit Descriptor Tail
	E1000_TXDCTL	= 0x3828,	// Transmit Descriptor Control
	E1000_CRCERRS	= 0x4000,	// CRC Errors
	E1000_TPR	= 0x40D0,	// Total Packets Received
	E1000_TPT	= 0x40D4,	// Total Packets Transmitted
	E1000_RA	= 0x5400,	// Receive-filter Array
    E1000_RAL	= 0x5400,	// Receive-address Array
    E1000_RAH	= 0x5404,	// Receive-address Array
	E1000_VFTA	= 0x5600,	// VLAN Filter Table Array
	};



char modname[] = "nic2";
char devname[] = "nic";
char info_rx[] = "nicrx";
char info_tx[] = "nictx";
int	my_major = 97;
struct pci_dev	*devp;
unsigned int	mmio_base;
unsigned int	mmio_size;
void		*io, *kmem;
RX_DESCRIPTOR	*rxring;
TX_DESCRIPTOR	*txring;
unsigned int	kmem_phys;
unsigned char	mac[6], dstn[6];
unsigned short	vlan_id = 0x456;
wait_queue_head_t	wq_recv;
wait_queue_head_t	wq_xmit;




static ssize_t my_get_info_rx(struct file *filp,char *buf,size_t count,loff_t *offp );
static ssize_t my_get_info_tx(struct file *filp,char *buf,size_t count,loff_t *offp );
ssize_t my_read( struct file *file, char *buf, size_t len, loff_t *pos );
ssize_t my_write( struct file *file, const char *buf, size_t len, loff_t *pos );
static ssize_t my_ioctl(struct file *filp, unsigned int cmd, unsigned long args);
static int my_get_mac(void);



static struct file_operations my_proc_rxinfo = {
    .owner = THIS_MODULE,
    .read= my_get_info_rx
};

static struct file_operations my_proc_txinfo = {
    .owner = THIS_MODULE,
    .read= my_get_info_tx
};

struct file_operations my_fops= {
				  .owner=THIS_MODULE,
				  .read=my_read,
				  .write=my_write,
				  .unlocked_ioctl=my_ioctl,
				  };
ssize_t my_write( struct file *file, const char *buf, size_t len, loff_t *pos )
{
	int	txtail = ioread32( io + E1000_TDT );
	char	*cp = phys_to_virt( txring[ txtail ].base_address );

	// sleep until controller has finished with the 'tail' descriptor 
	if ( txring[ txtail ].desc_status == 0 )
		{
		if ( file->f_flags & O_NONBLOCK ) return 0;
		printk( "NIC2 WRITE: going to sleep \n" );
		if ( wait_event_interruptible( wq_xmit,
			txring[ txtail ].desc_status ) ) return -EINTR;
		printk( "NIC2 WRITE: waking up\n" );
		}
	
	// Here we send no more bytes than will fit in a single packet-buffer
	if ( len + HDR_BYTES+4 > TX_BUFSIZ ) len = TX_BUFSIZ - (HDR_BYTES+4);  
	else if ( len > USUAL_MTU ) len = USUAL_MTU;

	// copy the user's data to current descriptor's packet-buffer
	if ( copy_from_user( cp+HDR_BYTES, buf, len ) ) return -EFAULT;	

	// setup the ethernet header and our count of the data-bytes 
	memcpy( cp+0, dstn, 6 );			// DA
	memcpy( cp+6,  mac, 6 );			// SA
	*(unsigned short*)(cp+12) = htons( 0x0800 );	// TYPE/LEN
	*(unsigned short*)(cp+14) = len;		// byte-count

	// prepare the transmit-descriptor
	txring[ txtail ].packet_length = HDR_BYTES+len;
	txring[ txtail ].desc_status = 0;
	txring[ txtail ].desc_command = 0;		 
	txring[ txtail ].desc_command |= (1<<3);	// RS 
	txring[ txtail ].desc_command |= (1<<0);	// EOP
	txring[ txtail ].desc_command |= (1<<6);	// VLE
	txring[ txtail ].special_info = vlan_id;

	// give ownership of this descriptor to the network controller
	txtail = (1 + txtail) % N_TX_DESC;
	iowrite32( txtail, io + E1000_TDT );
	return	len;
}













static int my_get_mac(){

	u32 rar_low;
	u32 rar_high;
	u8 dev_addr[MAX_ETH_ADDR_LEN];
	int i =0;
	rar_high = ioread32( io + E1000_RAH );
	rar_low = ioread32( io + E1000_RAL );


	for(i=0; i<RAL_MAC_ADDR_LEN;i++){
		printk("%02x ", (u8)(rar_low >> (8*i )&0xff )) ;
		dev_addr[i]= (u8)(rar_low >> (8*i )&0xff ) ;
	}
	for(i=0; i<RAH_MAC_ADDR_LEN;i++){
		dev_addr[i+RAL_MAC_ADDR_LEN]= (u8)(rar_high >> (8*i )&0xff ) ;
	}

	memcpy(mac,dev_addr,MAX_ETH_ADDR_LEN);
	
	return 0;
} 

ssize_t my_read( struct file *file, char *buf, size_t len, loff_t *pos )
{
	static int	rxhead = 0, pickup = 0; 
	unsigned char	*cp = phys_to_virt( rxring[ rxhead ].base_address );
	int		count, nbytes;

	// sleep until the current descriptor has nonzero status
	if (( pickup == 0 )&&( rxring[ rxhead ].desc_status == 0 ))
		{
		if ( file->f_flags & O_NONBLOCK ) return 0; 
		printk( "NIC2 READ: going to sleep \n" );
		if ( wait_event_interruptible( wq_recv, 
			rxring[ rxhead ].desc_status ) ) return -EINTR;
		printk( "NIC2 READ: waking up \n" );
		}

	// Here we do not handle packets that exceed packet-buffer size
	if ( ( rxring[ rxhead ].desc_status & 3 ) == 1 ) // DD, ~EOP
		{
		printk( "NIC2 READ: Oversized packet dropped\n" );
		while ( ( rxring[ rxhead ].desc_status & 3 ) == 1 )
			{
			rxring[ rxhead ].desc_status = 0;
			rxhead = (1 + rxhead) % N_RX_DESC;
			}
		rxring[ rxhead ].desc_status = 0;
		rxhead = (1 + rxhead) % N_RX_DESC;
		pickup = 0;
		return -EMSGSIZE;
		}

	// get the number of actual data-bytes in this packet
	count = *(unsigned short*)(cp+14); 

	// now we try to copy these data-bytes to the user's buffer
	nbytes = (count > len + pickup) ? len : count - pickup;
	if ( copy_to_user( buf, cp+HDR_BYTES+pickup, nbytes ) ) return -EFAULT;
	pickup += nbytes;

	// if all packet-data was transferred, advance 'rxhead' index
	if ( pickup >= count )	
		{
		pickup = 0;
		rxring[ rxhead ].desc_status = 0;
		rxhead = (1 + rxhead) % N_RX_DESC;
		}

	// tell the kernel how many bytes were transferred
	return	nbytes;	
}






static ssize_t my_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	unsigned char	*usr = (unsigned char*)args;	
	unsigned int 	vid, shift, index, mask;

	switch ( cmd )
		{
		case 0:	// Set this driver's current destination MAC-address
			if ( copy_from_user( dstn, usr, 6 ) ) return -EFAULT;	
			return	0;	// SUCCESS

		case 1: // Get this driver's current destination MAC-address
			if ( copy_to_user( usr, dstn, 6 ) ) return -EFAULT;
			return	0;	// SUCCESS

		case 2:	// Set this driver's current VLAN identification
			vid = (vlan_id & 0x0FFF);
			shift = vid % 32;
			index = vid / 32;
			if ( copy_from_user( &vid, usr, 2 ) ) return -EFAULT;
			// invalidate former VLAN Filter-Table entry
			mask = ioread32( io + E1000_VFTA + 4*index );
			mask &= ~(1 << shift);
			iowrite32( mask, io + E1000_VFTA + 4*index ); 
			// install modified VLAN Filter-Table entry
			vid &= 0xFFF;
			shift = vid % 32;
			index = vid / 32;
			mask = ioread32( io + E1000_VFTA + 4*index );
			mask |= (1 << shift);
			iowrite32( mask, io + E1000_VFTA + 4*index ); 
			vlan_id = (vlan_id & 0xF000)|(vid & 0x0FFF); 		
			return	0;	// SUCCESS
	
		case 3:	// Get this driver's current VLAN identification
			vid = (vlan_id & 0x0FFF);
			if ( copy_to_user( usr, &vid, 2 ) ) return -EFAULT; 
			return	0;	// SUCCESS
		}
	return	-EINVAL;
}















irqreturn_t my_isr( int irq, void *dev_id )
{
	static int	reps = 0;
	int		intr_cause = ioread32( io + E1000_ICR );
	if ( intr_cause == 0 ) return IRQ_NONE;

	printk( "NIC2 %-2d  cause=%08X  ", ++reps, intr_cause );
	if ( intr_cause & (1<<0) ) printk( "TXDW " );
	if ( intr_cause & (1<<1) ) printk( "TXQE " );
	if ( intr_cause & (1<<2) ) printk( "LC " );
	if ( intr_cause & (1<<4) ) printk( "RXDMT0 " );
	if ( intr_cause & (1<<6) ) printk( "RXO " );
	if ( intr_cause & (1<<7) ) printk( "RXT0 " );
	if ( intr_cause & (1<<9) ) printk( "MDAC " );
	if ( intr_cause & (1<<15) ) printk( "TXDLOW " );
	if ( intr_cause & (1<<16) ) printk( "SRPD " );
	if ( intr_cause & (1<<17) ) printk( "ACK " );
	printk( "\n" );	

	if ( intr_cause & (1<<4) ) 	// Rx-Descriptors Low
		{
		int	rxtail = ioread32( io + E1000_RDT );
		rxtail = (8 + rxtail) % N_RX_DESC;
		iowrite32( rxtail, io + E1000_RDT );
		}

	if ( intr_cause & (1<<0) )	// Tx-descriptor Written back
		wake_up_interruptible( &wq_xmit );

	if ( intr_cause & (1<<7) )	// Rx-descriptor Timer expired
		wake_up_interruptible( &wq_recv );

	iowrite32( intr_cause, io + E1000_ICR );
	return	IRQ_HANDLED;
}











static ssize_t my_get_info_rx(struct file *filp,char *buf,size_t count,loff_t *offp )
{
	static int	n_recv_packets = 0;
	int		i, head, tail, len = 0;

	n_recv_packets += ioread32( io + E1000_TPR );
	head = ioread32( io + E1000_RDH );
	tail = ioread32( io + E1000_RDT );
	
	len += sprintf( buf+len, "\n Receive-Descriptor Buffer-Area " );
	len += sprintf( buf+len, "(head=%d, tail=%d) \n\n", head, tail );
	for (i = 0; i < N_RX_DESC; i++)
		{
		int	status = rxring[ i ].desc_status;
		int	errors = rxring[ i ].desc_errors;
		len += sprintf( buf+len, " #%-2d ", i );
		len += sprintf( buf+len, "%08lX: ", (long)(rxring + i) );
		len += sprintf( buf+len, "%016llX ", rxring[i].base_address );
		len += sprintf( buf+len, "%04X ", rxring[i].packet_length );
		len += sprintf( buf+len, "%04X ", rxring[i].packet_chksum );
		len += sprintf( buf+len, "%02X ", rxring[i].desc_status );
		len += sprintf( buf+len, "%02X ", rxring[i].desc_errors );
		len += sprintf( buf+len, "%04X ", rxring[i].vlan_tag );
		if ( status & (1<<0) ) len += sprintf( buf+len, "DD " );
		if ( status & (1<<1) ) len += sprintf( buf+len, "EOP " );
		if ( status & (1<<2) ) len += sprintf( buf+len, "IXSM " );
		if ( status & (1<<3) ) len += sprintf( buf+len, "VP " );
		if ( status & (1<<5) ) len += sprintf( buf+len, "TCPCS " );
		if ( status & (1<<6) ) len += sprintf( buf+len, "IPCS " );
		if ( status & (1<<7) ) len += sprintf( buf+len, "PIF " );
		len += sprintf( buf+len, " " );
		if ( errors & (1<<0) ) len += sprintf( buf+len, "CE " );
		if ( errors & (1<<2) ) len += sprintf( buf+len, "FE " );
		if ( errors & (1<<5) ) len += sprintf( buf+len, "TCPE " );
		if ( errors & (1<<6) ) len += sprintf( buf+len, "IPE " );
		if ( errors & (1<<7) ) len += sprintf( buf+len, "RXE " );
		len += sprintf( buf+len, "\n" );
		}
	len += sprintf( buf+len, "\n" );
	len += sprintf( buf+len, " packets_received = %d ", n_recv_packets );
	len += sprintf( buf+len, "\n\n" );
	return	len;
}














static ssize_t my_get_info_tx(struct file *filp,char *buf,size_t count,loff_t *offp )
{
	static int	n_xmit_packets = 0;
	int		i, head, tail, len = 0;

	n_xmit_packets += ioread32( io + E1000_TPT );
	head = ioread32( io + E1000_TDH );
	tail = ioread32( io + E1000_TDT );
	
	len += sprintf( buf+len, "\n Transmit-Descriptor Buffer-Area " );
	len += sprintf( buf+len, "(head=%d, tail=%d) \n\n", head, tail );
	for (i = 0; i < N_TX_DESC; i++)
		{
		int	command = txring[ i ].desc_command;
		int	status = txring[ i ].desc_status;
		len += sprintf( buf+len, " #%-2d ", i );
		len += sprintf( buf+len, "%08lX: ", (long)(txring + i) );
		len += sprintf( buf+len, "%016llX ", txring[i].base_address );
		len += sprintf( buf+len, "%04X ", txring[i].packet_length );
		len += sprintf( buf+len, "%02X ", txring[i].cksum_offset );
		len += sprintf( buf+len, "%02X ", txring[i].desc_command );
		len += sprintf( buf+len, "%02X ", txring[i].desc_status );
		len += sprintf( buf+len, "%02X ", txring[i].cksum_origin );
		len += sprintf( buf+len, "%04X ", txring[i].special_info );
		len += sprintf( buf+len, " " );
		if ( status & (1<<0) ) len += sprintf( buf+len, "DD " );
		if ( status & (1<<1) ) len += sprintf( buf+len, "EC " );
		if ( status & (1<<2) ) len += sprintf( buf+len, "LC " );
		len += sprintf( buf+len, " " );
		if ( command & (1<<0) ) len += sprintf( buf+len, "EOP " );
		if ( command & (1<<1) ) len += sprintf( buf+len, "IFCS " );
		if ( command & (1<<2) ) len += sprintf( buf+len, "IC " );
		if ( command & (1<<3) ) len += sprintf( buf+len, "RS " );
		if ( command & (1<<6) ) len += sprintf( buf+len, "VLE " );
		if ( command & (1<<7) ) len += sprintf( buf+len, "IDE " );
		len += sprintf( buf+len, "\n" );
		}
	len += sprintf( buf+len, "\n" );
	len += sprintf( buf+len, " packets_sent = %d ", n_xmit_packets );
	len += sprintf( buf+len, "\n\n" );
	return	len;
}














static int __init nicsplit_init( void )
{
	int	i, dev_control, rx_control, tx_control;
	unsigned int	rx_buf, tx_buf;
	u16	pci_cmd;

	printk( "<1>\nInstalling \'%s\' module\n", modname );

	devp = pci_get_device( VENDOR_ID, DEVICE_ID, NULL );
	if ( !devp ) return -ENODEV;

	mmio_base = pci_resource_start( devp, 0 );
	mmio_size = pci_resource_len( devp, 0 );
	io = ioremap_nocache( mmio_base, mmio_size );
	if ( !io ) return -ENOSPC;

	kmem = kzalloc( KMEM_SIZE, GFP_KERNEL );
	if ( !kmem ) { iounmap( io ); return -ENOMEM; }
	kmem_phys = virt_to_phys( kmem );

	memset( dstn, 0xFF, 6 );
	//memcpy( mac, io + E1000_RA, 6 );
	my_get_mac();
	for (i = 0; i < 0x200; i+=4) ioread32( io + E1000_CRCERRS + i );	
	for (i = 0; i < 0x200; i+=4) iowrite32( 0, io + E1000_VFTA + i );

	pci_read_config_word( devp, 4, &pci_cmd );
	pci_cmd |= (1<<2);
	pci_write_config_word( devp, 4, pci_cmd );

	rxring = phys_to_virt( kmem_phys );
	rx_buf = virt_to_phys( rxring ) + (16 * N_RX_DESC);
	for (i = 0; i < N_RX_DESC; i++)
		{
		rxring[ i ].base_address = rx_buf + i*RX_BUFSIZ;
		rxring[ i ].packet_length = 0;
		rxring[ i ].packet_chksum = 0;
		rxring[ i ].desc_status = 0;
		rxring[ i ].desc_errors = 0;
		rxring[ i ].vlan_tag = 0;	
		}

	txring = phys_to_virt( kmem_phys + RX_MEMLEN );
	tx_buf = virt_to_phys( txring ) + (16 * N_TX_DESC);
	for (i = 0; i < N_TX_DESC; i++)
		{
		txring[ i ].base_address = tx_buf + i*TX_BUFSIZ;
		txring[ i ].packet_length = 0;
		txring[ i ].cksum_offset = 0;
		txring[ i ].cksum_origin = 0;
		txring[ i ].desc_status = (1<<0);	// DD
		txring[ i ].desc_command = 0; //(1<<3);	// RS
		txring[ i ].special_info = 0;
		}

	init_waitqueue_head( &wq_recv );
	init_waitqueue_head( &wq_xmit );

	dev_control = 0;
	dev_control |= (1<<0);	// FD-bit (Full Duplex)
	dev_control |= (0<<2);	// GIOMD-bit (GIO Master Disable)
	dev_control |= (1<<3);	// LRST-bit (Link Reset)
	dev_control |= (1<<6);	// SLU-bit (Set Link Up)	
	dev_control |= (2<<8);	// SPEED=2 (1000Mbps)
	dev_control |= (1<<11);	// FRCSPD-bit (Force Speed)
	dev_control |= (0<<12);	// FRCDPLX-bit (Force Duplex)
	dev_control |= (0<<20);	// ADVD3WUC-bit (Advertise D3 Wake Up Cap)
	dev_control |= (1<<26);	// RST-bit (Device Reset)
	dev_control |= (0<<27);	// RFCE-bit (Receive Flow Control Enable)
	dev_control |= (0<<28);	// TFCE-bit (Transmit Flow Control Enable) 
	dev_control |= (1<<30);	// VME-bit (VLAN Mode Enable) 
	dev_control |= (0<<31);	// PHY_RST-bit (PHY Reset)

	iowrite32( 0x00000000, io + E1000_STATUS );	// Device Status 
	iowrite32( 0xFFFFFFFF, io + E1000_IMC );	// Interrupt Mask Clear
	iowrite32( dev_control, io + E1000_CTRL );	// Device Control
	dev_control &= ~(1<<26);  	// clear RST-bit (Device Reset)
	iowrite32( dev_control, io + E1000_CTRL );	// Device Control
	udelay( 10000 );
	while ( (ioread32( io + E1000_STATUS ) & 3) != 3 );

	iowrite32( 0x00008100, io + E1000_VET );
	iowrite32( 1 << (vlan_id & 0x1F), io + E1000_VFTA + (vlan_id>>5)*4 );

	rx_control = 0;
	rx_control |= (0<<1);	// EN-bit (Enable)
	rx_control |= (1<<2);	// SBP-bit (Store Bad Packets)
	rx_control |= (1<<3);	// UPE-bit (Unicast Promiscuous Enable)
	rx_control |= (1<<4);	// MPE-bit (Multicase Promiscuous Enable)
	rx_control |= (0<<5);	// LPE-bit (Long Packet Enable)
	rx_control |= (0<<6);	// LBM=0 (LoopBack Mode off)
	rx_control |= (3<<8);	// RDMTS=3 (Rx-Descriptor Min Thresh Size)
	rx_control |= (0<<10);	// DTYPE=0 (Descriptor Type)
	rx_control |= (0<<12);	// MO=0 (Multicast Offset)
	rx_control |= (1<<15);	// BAM-bit (Broadcast Address Enable)
	rx_control |= (0<<16);	// BSIZE=0 (Receive Buffer Size = 2048)
	rx_control |= (1<<18);	// VLE-bit (VLAN Filter Enable)
	rx_control |= (0<<19);	// CFIEN=0 (Canonical Form Indicator Enable)
	rx_control |= (0<<20);	// CFI=0 (Canonical Form Indicator bit-value)
	rx_control |= (0<<22);	// DPF-bit (Discard Pause Frames)
	rx_control |= (1<<23);	// PMCF-bit (Pass MAC Control Frames)
	rx_control |= (0<<25);	// BSEX-bit (Buffer Size Extension)
	rx_control |= (1<<26);	// SECRC-bit (Strip Ethernet CRC)
	rx_control |= (0<<27);	// FLEXBUF=0 (Flexible Buffer Size)	
	iowrite32( rx_control, io + E1000_RCTL );	// Receive Control

	tx_control = 0;
	tx_control |= (0<<1);	// EN-bit (Enable )
	tx_control |= (1<<3);	// PSP-bit (Pad Short Packets) 
	tx_control |= (15<<4);	// CT=15 (Collision Threshold)
	tx_control |= (63<<12);	// COLD=63 (Collision Distance)
	tx_control |= (0<<22);	// SWXOFF-bit (Software XOFF Transmit)
	tx_control |= (1<<24);	// RTLC-bit (Re-Transmit on Late Collision)
	tx_control |= (0<<25);	// UNORTX-bit (Underrun No Re-Transmit)
	tx_control |= (0<<26);	// TXCSCMT=0 (TxDesc Minimum Threshold)
	tx_control |= (0<<28);	// MULR-bit (Multiple Request Support)
	iowrite32( tx_control, io + E1000_TCTL );	// Trandmit Control

	iowrite32( 0x001401C0, io + E1000_CTRL_EXT );

	i = devp->irq;
	if ( request_irq( i, my_isr, IRQF_SHARED, modname, &modname ) < 0 )
		{ kfree( kmem ); iounmap( io ); return -EBUSY; }
	iowrite32(  INTR_MASK, io + E1000_IMS );

	tx_buf = virt_to_phys( txring );
	iowrite32(     tx_buf, io + E1000_TDBAL );
	iowrite32( 0x00000000, io + E1000_TDBAH );
	iowrite32( 16 * N_TX_DESC, io + E1000_TDLEN );
	iowrite32( 0x01010000, io + E1000_TXDCTL );
	tx_control |= (1<<1);
	iowrite32( tx_control, io + E1000_TCTL );

	rx_buf = virt_to_phys( rxring );
	iowrite32(     rx_buf, io + E1000_RDBAL );
	iowrite32( 0x00000000, io + E1000_RDBAH );
	iowrite32( 16 * N_RX_DESC, io + E1000_RDLEN );
	iowrite32( 0x01010000, io + E1000_RXDCTL );
	rx_control |= (1<<1);
	iowrite32( rx_control, io + E1000_RCTL );

	// trigger interrupt to give some RX-descriptors to the NIC
	iowrite32( (1<<4), io + E1000_ICS );	// RXDMT0 

    proc_create(PROC_RX_NAME, 0, NULL, &my_proc_rxinfo);
    proc_create(PROC_TX_NAME, 0, NULL, &my_proc_txinfo);
	return	register_chrdev( my_major, devname, &my_fops );
}























static void __exit nicsplit_exit(void )
{
	int	rx_control, tx_control;

	rx_control = ioread32( io + E1000_RCTL );
	rx_control &= ~(1<<1);
	iowrite32( rx_control, io + E1000_RCTL );

	tx_control = ioread32( io + E1000_TCTL );
	tx_control &= ~(1<<1);
	iowrite32( tx_control, io + E1000_TCTL );

	iowrite32( 0xFFFFFFFF, io + E1000_IMC );
	free_irq( devp->irq, modname );

	unregister_chrdev( my_major, devname );
	remove_proc_entry( info_tx, NULL );
	remove_proc_entry( info_rx, NULL );

	kfree( kmem );
	iounmap( io );

	printk( "<1>Removing \'%s\' module\n", modname );
}

module_init( nicsplit_init );
module_exit( nicsplit_exit );
MODULE_LICENSE("GPL"); 

