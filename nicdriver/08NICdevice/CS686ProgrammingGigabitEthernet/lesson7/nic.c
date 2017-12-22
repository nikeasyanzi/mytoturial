//-------------------------------------------------------------------
//	nic.c
//
//	This Linux device-driver module provides a platform for some
//	future investigations with our Intel 82573L Gigabit Ethernet 
//	Controllers.  It implements the character-mode 'write()' and
//	'read()' methods, used here to transmit and receive packets,
//	and an 'ioctl()' method, which lets an application determine 
//	a packet's destination MAC-address; in addition, two pseudo-
//	files display the status of the TX and RX descriptor queues.
//
//	NOTE:  Written and tested for Linux kernel version 2.6.22.5.
//
//	programmer: ALLAN CRUSE
//	date begun: 09 FEB 2008
//	completion: 11 FEB 2008
//-------------------------------------------------------------------

#include <linux/module.h>	// for init_module() 
#include <linux/proc_fs.h>	// for create_proc_info_entry() 
#include <linux/pci.h>		// for pci_get_device()
#include <linux/interrupt.h>	// for request_irq()
#include <linux/delay.h>
#include <asm/uaccess.h>	// for copy_from_user()


#define	VENDOR_ID	0x8086	// Intel Corporation
#define DEVICE_ID	0x109A	// 82573L network controller
//#define DEVICE_ID	0x10B9	// 82572EI network controller
//#define DEVICE_ID	0x107C	// 82541PI network controller
#define N_TX_DESC	16	// number of TX Descriptors
#define N_RX_DESC	16	// number of RX Descriptors
#define BUFF_SIZE	0x600	// length of packet buffers
#define KMEM_SIZE	0x10000	// kernel memory allocation
#define INTR_MASK    0xFFFFFFFF	// device-interrupt mask


typedef struct	{
		unsigned long long	base_address;
		unsigned short		packet_length;
		unsigned char		cksum_offset;
		unsigned char		desc_command;
		unsigned char		desc_status;
		unsigned char		cksum_origin;
		unsigned short		special_info;
		} TX_DESCRIPTOR;


typedef struct	{
		unsigned long long	base_address;
		unsigned short		packet_length;
		unsigned short		packet_cksum;
		unsigned char		desc_status;
		unsigned char		desc_errors;
		unsigned short		vlan_tag;
		} RX_DESCRIPTOR;



enum	{
	E1000_CTRL	= 0x0000,	// Device Control
	E1000_STATUS	= 0x0008,	// Device Status
	E1000_CTRL_EXT	= 0x0018,	// Device Control Extension
	E1000_ICR	= 0x00C0,	// Interrupt Cause Read
	E1000_ICS	= 0x00C8,	// Interrupt Cause Set
	E1000_IMS	= 0x00D0,	// Interrupt Mask Set
	E1000_IMC	= 0x00D8,	// Interrupt Mask Clear
	E1000_RCTL	= 0x0100,	// Receive Control
	E1000_TCTL	= 0x0400,	// Transmit Control
	E1000_RDBAL	= 0x2800,	// Rx Descriptor Base Address Low	
	E1000_RDBAH	= 0x2804,	// Rx Descriptor Base Address High
	E1000_RDLEN	= 0x2808,	// Rx Descriptor Length
	E1000_RDH	= 0x2810,	// Rx Descriptor Head	
	E1000_RDT	= 0x2818,	// Rx Descriptor Tail	
	E1000_RXDCTL	= 0x2828,	// Rx Descriptor Control	
	E1000_TDBAL	= 0x3800,	// Tx Descriptor Base Address Low	
	E1000_TDBAH	= 0x3804,	// Tx Descriptor Base Address High
	E1000_TDLEN	= 0x3808,	// Tx Descriptor Length
	E1000_TDH	= 0x3810,	// Tx Descriptor Head	
	E1000_TDT	= 0x3818,	// Tx Descriptor Tail	
	E1000_TXDCTL	= 0x3828,	// Tx Descriptor Control	
	E1000_TPR	= 0x40D0,	// Total Packets Received
	E1000_TPT	= 0x40D4,	// Total Packets Transmitted
	E1000_RA	= 0x5400,	// Receive-filter Array
	};



char modname[] = "nic";
char devname[] = "nic";
char info_rx[] = "nicrx";
char info_tx[] = "nictx";
int	my_major = 97;
struct pci_dev	*devp;
unsigned char	mac[ 6 ];
unsigned char	dstn[ 6 ];
unsigned int	irq;
unsigned int	mmio_base;
unsigned int	mmio_size;
void		*io, *kmem;
unsigned int	kmem_phys;
RX_DESCRIPTOR	*rxring;
TX_DESCRIPTOR	*txring;
wait_queue_head_t  wq_recv;









irqreturn_t my_isr( int irq, void *dev_id )
{
	static int	reps = 0;
	int		intr_cause = ioread32( io + E1000_ICR );
	if ( intr_cause == 0 ) return IRQ_NONE;
	
	printk( " NIC: #%d CAUSE=%08X ", ++reps, intr_cause );
	if ( intr_cause & (1<<0) ) printk( "TXDW " );
	if ( intr_cause & (1<<1) ) printk( "TXQE " );
	if ( intr_cause & (1<<2) ) printk( "LSC " );
	if ( intr_cause & (1<<4) ) printk( "RXDMT0 " );
	if ( intr_cause & (1<<6) ) printk( "RXO (Receiver Overrun)" );
	if ( intr_cause & (1<<7) ) printk( "RXT0 (Receiver Timeout)" );
	if ( intr_cause & (1<<9) ) printk( "MDAC " );
	if ( intr_cause & (1<<15) ) printk( "TXDLOW " );
	if ( intr_cause & (1<<16) ) printk( "SRPD " );
	if ( intr_cause & (1<<17) ) printk( "ACK " );
	printk( "\n" );	

	// wake up a sleeping reader if new packets have arrived
	if ( intr_cause & (1<<7) ) wake_up_interruptible( &wq_recv );
	
	// clear the pertinent interrupt-cause bits and resume the task 
	iowrite32( intr_cause, io + E1000_ICR );
	return	IRQ_HANDLED;
}


ssize_t my_read( struct file *file, char *buf, size_t len, loff_t *pos )
{
	static int	rxhead = 0;
	unsigned char	*from = phys_to_virt( rxring[ rxhead ].base_address );
	unsigned int	count;

	// sleep if no new packet has been received	
	if ( ioread32( io + E1000_RDH ) == rxhead )
		if ( wait_event_interruptible( wq_recv, 
			ioread32( io + E1000_RDH ) != rxhead ) ) return -EINTR;

	// get the number of actual data-bytes in this packet
	count = *(unsigned short*)(from + 14);

	// here we try to copy these bytes to the user's buffer 
	if ( copy_to_user( buf, from + 16, count ) ) return -EFAULT;
	
	// update our ring-buffer index-value to the next descriptor
	rxhead = (1 + rxhead) % N_RX_DESC;

	// tell the kernel how many bytes were transferred
	return	count;
}




ssize_t my_write( struct file *file, const char *buf, size_t len, loff_t *pos )
{
	int	txtail = ioread32( io + E1000_TDT );
	char	*packet = (char*)phys_to_virt( txring[ txtail ].base_address );
	
	// we cannot transmit more than 1500 bytes in an ethernet packet
	if ( len > 1500 ) len = 1500;	

	// copy user's data into our packet-buffer (following its header)
	if ( copy_from_user( packet + 16, buf, len ) ) return -EFAULT;

	// setup this packet's header
	memcpy( packet + 0, dstn, 6 );	// destination MAC-address
	memcpy( packet + 6,  mac, 6 );	// source MAC-address
	packet[ 12 ] = 0x08;		// Type/Length LSB
	packet[ 13 ] = 0x00;		// Type/Length MSB 
	memcpy( packet + 14, &len, 2 );	// setup actual data-length

	// setup this packet's Tx descriptor
	txring[ txtail ].packet_length = 16 + len;
	txring[ txtail ].desc_status = 0;
	
	// give ownership of this descriptor to the controller
	txtail = ( 1 + txtail ) % N_TX_DESC;
	iowrite32( txtail, io + E1000_TDT );

	// tell the kernel how many bytes were transferred
	return	len;
}




static ssize_t my_ioctl(struct file *filp, unsigned int cmd, unsigned long args){
	unsigned char	*from = (unsigned char *)args;

	switch ( cmd )
		{
		case 0:	// set driver's current packet-destination address 
			if ( copy_from_user( dstn, from, 6 ) ) return -EFAULT;
			return	0;  // SUCCESS

		case 1:	// get driver's current packet-destination address 
			if ( copy_to_user( from, dstn, 6 ) ) return -EFAULT;
			return	0;  // SUCCESS
		}

	return	-EINVAL;
}



static ssize_t my_get_info_tx(struct file *filp,char *buf,size_t count,loff_t *offp ){

	int	i, head, tail, n_xmit_packets, len = 0;

	// read the Tx descriptor queue's current head and tail 
	head = ioread32( io + E1000_TDH );
	tail = ioread32( io + E1000_TDT );

	// read selected controller statistics registers
	n_xmit_packets = ioread32( io + E1000_TPT );

	// display the current state of the Tx Descriptor queue	
	len += sprintf( buf+len, "\n  Transmit-Descriptor Buffer-Area " );
	len += sprintf( buf+len, "(head=%d, tail=%d) \n\n", head, tail );
	for (i = 0; i < N_TX_DESC; i++)
		{
		int	status = txring[i].desc_status;
		len += sprintf( buf+len, "  #%-2d ", i );
		len += sprintf( buf+len, "%08lX: ", (long)(txring+i) );
		len += sprintf( buf+len, "%016llX ", txring[i].base_address );
		len += sprintf( buf+len, "%04X ", txring[i].packet_length );
		len += sprintf( buf+len, "%02X ", txring[i].cksum_offset );
		len += sprintf( buf+len, "%02X ", txring[i].desc_command );
		len += sprintf( buf+len, "%02X ", txring[i].desc_status  );
		len += sprintf( buf+len, "%02X ", txring[i].cksum_origin );
		len += sprintf( buf+len, "%04X ", txring[i].special_info );
		if ( status & (1<<0) ) 	len += sprintf( buf+len, "DD " );
		if ( status & (1<<1) ) 	len += sprintf( buf+len, "EC " );
		if ( status & (1<<2) ) 	len += sprintf( buf+len, "LC " );
		len += sprintf( buf+len, "\n" );
		}
	len += sprintf( buf+len, "\n" );

	// show the transmit-related statistics counts
	len += sprintf( buf+len, "  packets_sent = %d ", n_xmit_packets );
	len += sprintf( buf+len, "\n\n" );
	return	len;
} 


static ssize_t my_get_info_rx(struct file *filp,char *buf,size_t count,loff_t *offp ){
	int	i, head, tail, n_recv_packets, len = 0;

	// read the Rx descriptor queue's current head and tail 
	head = ioread32( io + E1000_RDH );
	tail = ioread32( io + E1000_RDT );

	// read selected controller statistics registers
	n_recv_packets = ioread32( io + E1000_TPR );
	
	// display the current state of the Rx Descriptor queue	
	len += sprintf( buf+len, "\n  Receive-Descriptor Buffer-Area " );
	len += sprintf( buf+len, "(head=%d, tail=%d) \n\n", head, tail );
	for (i = 0; i < N_RX_DESC; i++)
		{
		int	status = rxring[i].desc_status;
		int	errors = rxring[i].desc_errors;
		len += sprintf( buf+len, "  #%-2d ", i );
		len += sprintf( buf+len, "%08lX: ", (long)(rxring+i) );
		len += sprintf( buf+len, "%016llX ", rxring[i].base_address );
		len += sprintf( buf+len, "%04X ", rxring[i].packet_length );
		len += sprintf( buf+len, "%04X ", rxring[i].packet_cksum );
		len += sprintf( buf+len, "%02X ", rxring[i].desc_status );
		len += sprintf( buf+len, "%02X ", rxring[i].desc_errors );
		len += sprintf( buf+len, "%04X ", rxring[i].vlan_tag );
		if ( status & (1<<0) ) 	len += sprintf( buf+len, "DD " );
		if ( status & (1<<1) ) 	len += sprintf( buf+len, "EOP " );
		if ( status & (1<<2) ) 	len += sprintf( buf+len, "IXSM " );
		if ( status & (1<<3) ) 	len += sprintf( buf+len, "VP " );
		if ( status & (1<<5) ) 	len += sprintf( buf+len, "TCPCS " );
		if ( status & (1<<6) ) 	len += sprintf( buf+len, "IPCS " );
		if ( status & (1<<7) ) 	len += sprintf( buf+len, "PIF " );
		len += sprintf( buf+len, " " );
		if ( errors & (1<<0) ) 	len += sprintf( buf+len, "CE " );
		if ( errors & (1<<2) ) 	len += sprintf( buf+len, "FE " );
		if ( errors & (1<<5) ) 	len += sprintf( buf+len, "TCPE " );
		if ( errors & (1<<6) ) 	len += sprintf( buf+len, "IPE " );
		if ( errors & (1<<7) ) 	len += sprintf( buf+len, "RXE " );
		len += sprintf( buf+len, "\n" );
		}
	len += sprintf( buf+len, "\n" );

	// show the receive-related statistics counts
	len += sprintf( buf+len, "  packets_received = %d ", n_recv_packets );
	len += sprintf( buf+len, "\n\n" );
	return	len;
}



struct file_operations	my_fops = {
				  .owner=	THIS_MODULE,
				  .read=	my_read,
				  .write=	my_write,
				  .unlocked_ioctl=my_ioctl,
				  };


static struct file_operations my_proc_ops_tx = {
    .owner = THIS_MODULE,
    .read= my_get_info_tx,
};

static struct file_operations my_proc_ops_rx = {
    .owner = THIS_MODULE,
    .read= my_get_info_rx,
};






static int __init nic_init( void )
{
	int	rx_control, tx_control, rdba_phys, tdba_phys, i;
	u16	pci_cmd;
    struct proc_dir_entry *entry;
	
	// write confirmation-message to the kernel's log-file
	printk( "<1>\nInstalling \'%s\' module ", modname );
	printk( "(major=%d) \n", my_major );

	// detect the presence of the Intel Pro1000 network controller
	devp = pci_get_device( VENDOR_ID, DEVICE_ID, NULL );
	if ( !devp ) return -ENODEV;

	// remap the controller's i/o-memory into kernel's address-space
	mmio_base = pci_resource_start( devp, 0 );
	mmio_size = pci_resource_len( devp, 0 );
	io = ioremap_nocache( mmio_base, mmio_size );
	if ( !io ) return -ENOSPC;

	// allocate memory for this driver's buffers and descriptors
	kmem = kzalloc( KMEM_SIZE, GFP_KERNEL );
	if ( !kmem ) { iounmap( io ); return -ENOMEM; }
	kmem_phys = virt_to_phys( kmem );

	// initialize this driver's Rx descriptors
	rdba_phys = kmem_phys + 0x0000;
	rxring = (RX_DESCRIPTOR *)phys_to_virt( rdba_phys );
	for (i = 0; i < N_RX_DESC; i++)
		{
		rxring[ i ].base_address = rdba_phys + 0x1000 + i*0x600;
		rxring[ i ].packet_length = 0;
		rxring[ i ].packet_cksum = 0;
		rxring[ i ].desc_status = 0;
		rxring[ i ].desc_errors = 0;
		rxring[ i ].vlan_tag = 0;
		}

	// initialize this driver's Tx descriptors
	tdba_phys = kmem_phys + 0x8000;
	txring = (TX_DESCRIPTOR *)phys_to_virt( tdba_phys );
	for (i = 0; i < N_TX_DESC; i++)
		{
		txring[ i ].base_address = tdba_phys + 0x1000 + i*0x600;
		txring[ i ].packet_length = 0;
		txring[ i ].cksum_offset = 0;
		txring[ i ].cksum_origin = 0;
		txring[ i ].desc_status = 0;
		txring[ i ].desc_command = (1<<0)|(1<<1)|(1<<3);
		txring[ i ].special_info = 0;
		}

	// make sure the controller's Bus Master capability is enabled
	pci_read_config_word( devp, 4, &pci_cmd );
	pci_cmd |= (1<<2);
	pci_write_config_word( devp, 4, pci_cmd );	


	// Reset the network controller hardware
	iowrite32( 0xFFFFFFFF, io + E1000_IMC );
	iowrite32( 0x00000000, io + E1000_STATUS );
	iowrite32( 0x040C0241, io + E1000_CTRL );	
	iowrite32( 0x000C0241, io + E1000_CTRL );	
	udelay( 10000 );

	// configure the controller's 'receive' engine
	rx_control = 0;
	rx_control |= (0<<1);	// EN-bit (Enable)
	rx_control |= (1<<2);	// SPB-bit (Store Bad Packets) 	
	rx_control |= (1<<3);	// UPE-bit (Unicast Promiscuous Mode)
	rx_control |= (1<<4);	// MPE-bit (Multicast Promiscuous Mode)
	rx_control |= (0<<5);	// LPE-bit (Long Packet Enable)
	rx_control |= (0<<6);	// LBM=0 (Loop-Back Mode)
	rx_control |= (0<<8);	// RDMTS=0 (Rx Descriptor Min Threshold Size)
	rx_control |= (0<<10);	// DTYPE=0 (Descriptor Type)
	rx_control |= (0<<12);	// MO=0 (Multicast Offset)
	rx_control |= (1<<15);	// BAM-bit (Broadcast Address Mode)
	rx_control |= (0<<16);	// BSIZE=0 (Buffer Size = 2048) 	
	rx_control |= (0<<18);	// VLE-bit (VLAN filter Enable)
	rx_control |= (0<<19);	// CFIEN-bit (Canonical Form Indicator Enable)	
	rx_control |= (0<<20);	// CFI-bit (Canonical Form Indicator)
	rx_control |= (0<<22);	// DPF-bit (Discard Pause Frames)	
	rx_control |= (0<<23);	// PMCF-bit (Pass MAC Control Frames)
	rx_control |= (0<<25);	// BSEX=0 (Buffer Size EXtension)
	rx_control |= (1<<26);	// SECRC-bit (Strip Ethernet CRC)
	rx_control |= (0<<27);	// FLEXBUF=0 (Flexible Buffer size)	
	iowrite32( rx_control, io + E1000_RCTL );

	// tell controller the location, size, and fetch-policy for Rx queue
	iowrite32(      rdba_phys, io + E1000_RDBAL );
	iowrite32(     0x00000000, io + E1000_RDBAH );
	iowrite32( N_RX_DESC * 16, io + E1000_RDLEN );
	iowrite32(     0x01010000, io + E1000_RXDCTL );

	// configure the controller's 'transmit' engine
	tx_control = 0;
	tx_control |= (0<<1);	// EN-bit (Enable)
	tx_control |= (1<<3);	// PSP-bit (Pad Short Packets)
	tx_control |= (15<<4);	// CT=15 (Collision Threshold)
	tx_control |= (63<<12);	// COLD=63 (Collision Distance)
	tx_control |= (0<<22);	// SWXOFF-bit (Software XOFF)
	tx_control |= (1<<24);	// RTLC-bit (Re-Transmit on Late Collision)
	tx_control |= (0<<25);	// UNORTX-bit (Underrun No Re-Transmit)
	tx_control |= (0<<26);	// TXCSCMT=0 (TxDesc Mininum Threshold)
	tx_control |= (0<<28);	// MULR-bit (Multiple Request Support)
	iowrite32( tx_control, io + E1000_TCTL );

	// tell controller the location, size, and fetch-policy for Tx queue
	iowrite32(      tdba_phys, io + E1000_TDBAL );
	iowrite32(     0x00000000, io + E1000_TDBAH );
	iowrite32( N_TX_DESC * 16, io + E1000_TDLEN );
	iowrite32(     0x01010000, io + E1000_TXDCTL );


	// initialize this driver's wait-queue for receive-packets
	init_waitqueue_head( &wq_recv );

	// initialize this driver's destination and source MAC-addresses
	memset( dstn, 0xFF, 6 );
	memcpy(  mac, io + E1000_RA, 6 );

	// install this driver's Interrupt Service Routine
	irq = devp->irq;
	if ( request_irq( irq, my_isr, IRQF_SHARED, devname, &devname ) < 0 )
		{ kfree( kmem ); iounmap( io ); return -EBUSY; }

	// enable the controller's interrupts
	iowrite32( 0xFFFFFFFF, io + E1000_ICR );
	iowrite32(  INTR_MASK, io + E1000_IMS );
	iowrite32( 0xFFFFFFFF, io + E1000_ICR );

	// turn on the controller's transmit engine 
	tx_control |= (1<<1);
	iowrite32( tx_control, io + E1000_TCTL );

	// turn on the controller's receive engine 
	rx_control |= (1<<1);
	iowrite32( rx_control, io + E1000_RCTL );

	// give the controller ownership of all receive descriptors
	iowrite32( N_RX_DESC, io + E1000_RDT );

	// install our driver's pseudo-files in the '/proc' directory
    entry = proc_create(info_rx, 0, NULL, &my_proc_ops_rx);
    if (!entry) {
        return -ENOMEM;
    }
    entry = proc_create(info_tx, 0, NULL, &my_proc_ops_tx);
    if (!entry) {
        return -ENOMEM;
    }
	
	// register this driver's file-operations with the kernel
	return	register_chrdev( my_major, devname, &my_fops );
}






















static void __exit nic_exit(void )
{
	int	tx_control, rx_control;

	// unregister this driver's file-operations
	unregister_chrdev( my_major, devname );

	// delete this driver's pseudo-files from the '/proc' directory
	remove_proc_entry( info_rx, NULL );
	remove_proc_entry( info_tx, NULL );

	// turn off the controller's receive engine
	rx_control = ioread32( io + E1000_RCTL );
	rx_control &= ~(1<<1);
	iowrite32( rx_control, io + E1000_RCTL );	

	// turn off the controller's transmit engine
	tx_control = ioread32( io + E1000_TCTL );
	tx_control &= ~(1<<1);
	iowrite32( tx_control, io + E1000_TCTL );	

	// turn off the controller's interrupts
	iowrite32( 0xFFFFFFFF, io + E1000_IMC );

	// remove this driver's interrupt-handler
	free_irq( irq, devname );

	// release this driver's memory-allocation
	kfree( kmem );

	// unmap the controller's i/o-memory from kernel-space
	iounmap( io );

	// write confirmation-message to the kernel's log-file
	printk( "<1>Removing \'%s\' module\n", modname );
}

module_init( nic_init );
module_exit( nic_exit );
MODULE_LICENSE("GPL"); 

