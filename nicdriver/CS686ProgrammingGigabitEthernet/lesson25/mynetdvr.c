//-------------------------------------------------------------------
//	mynetdvr.c
//
//	This module implements a minimal Linux network device-driver
//	for our Intel PRO1000 gibabit ethernet interface controller.
//
//	NOTE: Written and tested with Linux kernel version 2.6.22.5.
//
//	programmer: ALLAN CRUSE
//	written on: 22 FEB 2008
//-------------------------------------------------------------------

#include <linux/module.h>	// for init_module()
#include <linux/etherdevice.h>	// for alloc_etherdev() 
#include <linux/netdevice.h>	// 
#include <linux/interrupt.h>	// for request_irq()
#include <linux/proc_fs.h>	// for create_proc_info_entry()
#include <linux/pci.h>		// for pci_get_device()

#define VENDOR_ID	0x8086	// Intel Corporation
//#define DEVICE_ID	0x109A	// 82573L controller
//#define DEVICE_ID	0x10B9	// 82572EI controller
#define DEVICE_ID	0x100F	// 82572EM controller
#define INTR_MASK   0xFFFFFFFF	// nic interrupt mask
#define N_RX_DESC	   32	// number of RX Descriptors
#define N_TX_DESC	   32	// number of TX Descriptors
#define RX_BUFSIZ	  2048	// size of RX packet-buffer
#define TX_BUFSIZ	  1536	// size of TX packet-buffer
#define RX_MEMLEN	(RX_BUFSIZ + 16)*N_RX_DESC
#define TX_MEMLEN	(TX_BUFSIZ + 16)*N_TX_DESC
#define KMEM_SIZE	(RX_MEMLEN + TX_MEMLEN)


#define RAL_MAC_ADDR_LEN 4
#define RAH_MAC_ADDR_LEN 2
#define MAX_ETH_ADDR_LEN (RAL_MAC_ADDR_LEN + RAH_MAC_ADDR_LEN)

//#define USE_NAPI
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


typedef struct	{
		RX_DESCRIPTOR	rxring[ N_RX_DESC ];		
		TX_DESCRIPTOR	txring[ N_TX_DESC ];
		RX_DESCRIPTOR 	**rx_virt_addr;
		unsigned char	rxbuff[ N_RX_DESC * RX_BUFSIZ ];
		unsigned char	txbuff[ N_TX_DESC * TX_BUFSIZ ];
		unsigned int	rxnext;
		struct tasklet_struct	rx_tasklet;
		struct napi_struct mynapi; 
		spinlock_t mylock;
		} MY_DRIVERDATA;



enum 	{ 
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
	//E1000_RA	= 0x5400,	// Receive-filter Array
    E1000_RAL	= 0x5400,	// Receive-address Array
    E1000_RAH	= 0x5404,	// Receive-address Array
	E1000_VFTA	= 0x5600,	// VLAN Filter Table Array
	};



// function prototypes
static int __init my_init( void );
static void __exit my_exit( void );
irqreturn_t my_isr( int, void * );
void my_rx_handler( unsigned long );
int my_open( struct net_device * );
int my_stop( struct net_device * );
int my_hard_start_xmit( struct sk_buff *, struct net_device * );
static ssize_t my_get_info(struct file *filp,char *buf,size_t count,loff_t *offp );
int my_get_mac(void);
static int my_napipoll(struct napi_struct *napi, int budget);
module_init( my_init );
module_exit( my_exit );
MODULE_LICENSE("GPL");


static struct file_operations my_proc= {
    .owner = THIS_MODULE,
    .read= my_get_info
 }; 

// global variables
char modname[] = "mynetdvr";
struct pci_dev	*devp;
unsigned int	mmio_base;
unsigned int	mmio_size;
void		*io;
struct net_device  *my_netdev;


struct net_device_ops	my_ops = {
				 ndo_open:		my_open,
				 ndo_stop:		my_stop,
				 ndo_start_xmit:	my_hard_start_xmit,
				 };


int my_get_mac(void){

	u32 rar_low;
	u32 rar_high;
	u8 dev_addr[MAX_ETH_ADDR_LEN];
	int i =0;
	rar_high = ioread32( io + E1000_RAH );
	rar_low = ioread32( io + E1000_RAL );

//	mac_dbg_printk("rar_high=%02x ", rar_high);
//	mac_dbg_printk("rar_low=%02x\n", rar_low);

	for(i=0; i<RAL_MAC_ADDR_LEN;i++){
		dev_addr[i]= (u8)(rar_low >> (8*i )&0xff ) ;
	}
	for(i=0; i<RAH_MAC_ADDR_LEN;i++){
		dev_addr[i+RAL_MAC_ADDR_LEN]= (u8)(rar_high >> (8*i )&0xff ) ;
	}

	memcpy(my_netdev->perm_addr,dev_addr,MAX_ETH_ADDR_LEN);
	my_netdev->dev_addr=my_netdev->perm_addr;

	//	printk("\n");

//	for(i=0; i<MAX_ETH_ADDR_LEN;i++){
//		printk("%02x ",dev_addr[i] );
//	}	 
	return 0;
}


static ssize_t my_get_info(struct file *filp,char *buf,size_t count,loff_t *offp ){
	MY_DRIVERDATA	*mdp = netdev_priv (my_netdev);	
	RX_DESCRIPTOR		*rdq = mdp->rxring;
	TX_DESCRIPTOR		*tdq = mdp->txring;
	unsigned int		rxhead = ioread32( io + E1000_RDH );
	unsigned int		rxtail = ioread32( io + E1000_RDT );
	unsigned int		txhead = ioread32( io + E1000_TDH );
	unsigned int		txtail = ioread32( io + E1000_TDT );
	int			i, len = 0;

	len += sprintf( buf+len, "\n Receive-Descriptor Queue " );
	len += sprintf( buf+len, "(head=%d, tail=%d) \n\n", rxhead, rxtail );
	for (i = 0; i < N_RX_DESC; i++)
		{
		unsigned int	ba = virt_to_phys( rdq+i );
		len += sprintf( buf+len, " #%-2d ", i );
		len += sprintf( buf+len, "%08X: ", ba );
		len += sprintf( buf+len, "%016llX ", rdq[i].base_address ); 	
		len += sprintf( buf+len, "%04X ", rdq[i].packet_length ); 	
		len += sprintf( buf+len, "%04X ", rdq[i].packet_chksum ); 	
		len += sprintf( buf+len, "%02X ", rdq[i].desc_status ); 	
		len += sprintf( buf+len, "%02X ", rdq[i].desc_errors ); 	
		len += sprintf( buf+len, "%04X ", rdq[i].vlan_tag ); 	
		len += sprintf( buf+len, "\n" );
		}
	/*
	len += sprintf( buf+len, "\n Transmit-Descriptor Queue " );
	len += sprintf( buf+len, "(head=%d, tail=%d) \n\n", txhead, txtail );
	for (i = 0; i < N_TX_DESC; i++)
		{
		unsigned int	ba = virt_to_phys( tdq+i );
		len += sprintf( buf+len, " #%-2d ", i );
		len += sprintf( buf+len, "%08X: ", ba );
		len += sprintf( buf+len, "%016llX ", tdq[i].base_address ); 	
		len += sprintf( buf+len, "%04X ", tdq[i].packet_length ); 	
		len += sprintf( buf+len, "%02X ", tdq[i].cksum_offset ); 	
		len += sprintf( buf+len, "%02X ", tdq[i].desc_command ); 	
		len += sprintf( buf+len, "%02X ", tdq[i].desc_status ); 	
		len += sprintf( buf+len, "%02X ", tdq[i].cksum_origin ); 	
		len += sprintf( buf+len, "%04X ", tdq[i].special_info ); 	
		len += sprintf( buf+len, "\n" );
		}

	len += sprintf( buf+len, "\n" );*/
	return	len;
}


static int __init my_init( void )
{
	u16	pci_cmd;

	printk( "<1>\nInstalling \'%s\' module\n", modname );

	// detect presence of the Intel Pro1000 controller
	devp = pci_get_device( VENDOR_ID, DEVICE_ID, NULL );
	if ( !devp ) return -ENODEV;

	// map the controller's i/o-memory into kernel space
	mmio_base = pci_resource_start( devp, 0 );
	mmio_size = pci_resource_len( devp, 0 );
	io = ioremap_nocache( mmio_base, mmio_size );
	if ( !io ) return -ENOSPC;




	// insure the controller's Bus Master capability is enabled 
	pci_read_config_word( devp, 4, &pci_cmd );
	pci_cmd |= (1<<2);
	pci_write_config_word( devp, 4, pci_cmd );

	// allocate kernel memory for the 'net_device' structure
	my_netdev = alloc_etherdev( sizeof( MY_DRIVERDATA ) );
	if ( !my_netdev ) { iounmap( io ); return -ENOMEM; }

	// initialize essential fields in the 'net_device' structure
//	memcpy( my_netdev->perm_addr, io + E1000_RA, 6 );
//	memcpy( my_netdev->dev_addr,  io + E1000_RA, 6 );
	my_get_mac();
	my_netdev->netdev_ops = &my_ops;
	my_netdev->mem_start		= mmio_base;
	my_netdev->mem_end		= mmio_base + mmio_size;
	my_netdev->irq			= devp->irq;
	my_netdev->flags		|= IFF_PROMISC;
	my_netdev->ml_priv		= netdev_priv( my_netdev );


	// create our driver's pseudo-file (for debugging)
    proc_create(modname, 0, NULL, &my_proc);




	// register this driver's 'net_device' structure
	return	register_netdev( my_netdev );
}


static void __exit my_exit( void )
{
	remove_proc_entry( modname, NULL );

	unregister_netdev( my_netdev );
	iounmap( io );
	free_netdev( my_netdev );

	printk( "<1>Removing \'%s\' module\n", modname );
}

int my_open( struct net_device *dev )
{
	//--------------------------------------------------------
	// The kernel calls this function whenever the 'ifconfig'
	// command is executed to bring up the device interface.
	// This function needs to call 'netif_start_queue()'.
	//--------------------------------------------------------

	MY_DRIVERDATA	*mdp = netdev_priv (dev);	
	RX_DESCRIPTOR	*rxq = mdp->rxring;
	TX_DESCRIPTOR	*txq = mdp->txring;
	unsigned long	rbuf = virt_to_phys( mdp->rxbuff );
	unsigned long	tbuf = virt_to_phys( mdp->txbuff );
	unsigned long	rxdescaddr = virt_to_phys( rxq );
	unsigned long	txdescaddr = virt_to_phys( txq );
	int		i;
	int rx_control=0;

//#ifdef USE_NAPI
//	netif_napi_add(my_netdev, & (mdp->mynapi),my_napipoll, 64); 
//#endif
	
	printk( " %s: opening the \'%s\' device \n", modname, dev->name );

	// reset the network controller
	iowrite32( 0xFFFFFFFF, io + E1000_IMC );	
	iowrite32( 0x00000000, io + E1000_STATUS );
	iowrite32( 0x040C0241, io + E1000_CTRL );
	iowrite32( 0x000C0241, io + E1000_CTRL );
	while ( ( ioread32( io + E1000_STATUS )&3 ) != 3 );

	// initialize the RX Descriptor-queues
	for (i = 0; i < N_RX_DESC; i++ )
	{
		rxq[ i ].base_address = rbuf + i * RX_BUFSIZ;
		rxq[ i ].packet_length = 0;
		rxq[ i ].packet_chksum = 0;
		rxq[ i ].desc_status = 0;
		rxq[ i ].desc_errors = 0;
		rxq[ i ].vlan_tag = 0;
 	} 

	// initialize the TX Descriptor-queues
	for (i = 0; i < N_TX_DESC; i++ )
	{
		txq[ i ].base_address = tbuf + i * TX_BUFSIZ;
		txq[ i ].packet_length = 0;
		txq[ i ].cksum_offset = 0;
		txq[ i ].desc_command = (1<<0)|(1<<1)|(1<<3);	// EOP/IFCS/RS
		txq[ i ].desc_status = 0;
		txq[ i ].cksum_origin = 0;
		txq[ i ].special_info = 0;
 	}


	/*
	   rx_control = 0;
	   rx_control |= (0<<1);	// EN-bit (Enable)
	   rx_control |= (1<<2);	// SBP-bit (Store Bad Packets)
	   rx_control |= (1<<3);	// UPE-bit (Unicast Promiscuous Enable)
	   rx_control |= (1<<4);	// MPE-bit (Multicase Promiscuous Enable)
	   rx_control |= (0<<5);	// LPE-bit (Long Packet Enable)
	   rx_control |= (0<<6);	// LBM=0 (LoopBack Mode off)
	   rx_control |= (2<<8);	// RDMTS=3 (Rx-Descriptor Min Thresh Size)
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
 	   */	// configure the controller's Receive Engine

	iowrite32( 0x0400801C, io + E1000_RCTL );
	rx_control=0x0400801C;
	iowrite32( rxdescaddr, io + E1000_RDBAL );
	iowrite32( 0x00000000, io + E1000_RDBAH );
	iowrite32( N_RX_DESC * 16, io + E1000_RDLEN );
	iowrite32( 0x01010000, io + E1000_RXDCTL );


	// configure the controller's Transmit Engine
	iowrite32( 0x0103F0F8, io + E1000_TCTL );
	iowrite32( txdescaddr, io + E1000_TDBAL );
	iowrite32( 0x00000000, io + E1000_TDBAH );
	iowrite32( N_TX_DESC * 16, io + E1000_TDLEN );
	iowrite32( 0x01010000, io + E1000_TXDCTL );

	// initialize our tasklet
	mdp->rxnext = 0;
	tasklet_init( &mdp->rx_tasklet, my_rx_handler, (unsigned long)dev );
	// install our driver's interrupt-handler
	i = dev->irq;
	if ( request_irq( i, my_isr, IRQF_SHARED, dev->name, dev ) < 0 )
		return -EBUSY;

	// unmask interrupts
	ioread32( io + E1000_ICR );
	iowrite32( INTR_MASK, io + E1000_IMS );

	// start the receive engine
	iowrite32( N_RX_DESC, io + E1000_RDT );
	iowrite32( ioread32( io + E1000_RCTL ) | (1<<1), io + E1000_RCTL );

	// start the transmit engine
	iowrite32( ioread32( io + E1000_TCTL ) | (1<<1), io + E1000_TCTL );
	
	printk("txdescaddr=%x\n ",txdescaddr);
	printk("rxdescaddr=%x\n ",rxdescaddr);
	

	//printk("txdescaddr=%x\n ",txdescaddr);
	//printk("rxdescaddr=%x\n ",rxdescaddr);


//#ifdef USE_NAPI 
//	napi_enable(&mdp->mynapi); 
//#endif
	netif_start_queue( dev );



	return	0;  // SUCCESS
}


int my_stop( struct net_device *dev )
{  
	//--------------------------------------------------------
	// The kernel calls this function whenever the 'ifconfig'
	// command is executed to shut down the device interface.
	// This function needs to call 'netif_stop_queue()'.
	//--------------------------------------------------------

	MY_DRIVERDATA	*mdp = netdev_priv (dev);	

	printk( " %s: stopping the \'%s\' device \n", modname, dev->name );

	// stop the controller's transmit and receive engines
	iowrite32( ioread32( io + E1000_RCTL ) & ~(1<<1), io + E1000_RCTL );
	iowrite32( ioread32( io + E1000_TCTL ) & ~(1<<1), io + E1000_TCTL );

	// disable controller interrupts and remove interrupt-handler
	iowrite32( 0x00000000, io + E1000_IMC );
	free_irq( dev->irq, dev );


#ifdef USE_NAPI 
	napi_disable(& mdp->mynapi); 
#else
	// stop our tasklet and the network interface queue
	tasklet_kill( &mdp->rx_tasklet );

#endif 
	netif_stop_queue( dev );

	return	0;  // SUCCESS
} 


int my_hard_start_xmit( struct sk_buff *skb, struct net_device *dev )
{  
	//------------------------------------------------------------
	// The kernel calls this function whenever its protocol layer
	// has a packet that it wants the controller to transmit.  It
	// must either call the 'netif_free_skb()' function itself or
	// else arrange for it to be called by our interrupt-handler.  	
	//------------------------------------------------------------


	MY_DRIVERDATA	*mdp = netdev_priv (dev);	
	TX_DESCRIPTOR	*txq ;
	unsigned int	curr ;

	unsigned int	next;
	unsigned char	*src ;
	unsigned char	*dst ;

	unsigned short	len;
	unsigned long flags;

	spin_lock_irqsave(&mdp->mylock, flags);

	txq= mdp->txring;
	curr= ioread32( io + E1000_TDT );
	next= (1 + curr) % N_TX_DESC;
	src= skb->data;
	dst = phys_to_virt( txq[ curr ].base_address );
	len= skb->len;

	// save the timestamp
	//dev->trans_start = jiffies;
	netdev_get_tx_queue(dev,0)->trans_start=jiffies;

	// copy the socket-buffer's data into the next packet-buffer
	if ( len > TX_BUFSIZ ) len = TX_BUFSIZ;
	memcpy( dst, src, len );

	// setup the next TX Descriptor
	txq[ curr ].packet_length = len;
	txq[ curr ].cksum_offset = 0;
	txq[ curr ].cksum_origin = 0;
	txq[ curr ].special_info = 0;
	printk("curr=%d, status=%d\n",curr, txq[curr].desc_status);
	txq[ curr ].desc_status = 0;
	txq[ curr ].desc_command = (1<<0)|(1<<1)|(1<<3);  // EOP/IFCS/RS

	// initiate the transmission
	iowrite32( next, io + E1000_TDT );

	// update the 'net_device' statistics
	dev->stats.tx_packets += 1;
	dev->stats.tx_bytes += len;

	// it is essential to free the socket-buffer structure
	dev_kfree_skb( skb );

	return	NETDEV_TX_OK;  // SUCCESS
} 


void my_rx_handler( unsigned long data )
{   
	//--------------------------------------------------------------
	// This function is scheduled by our driver's interrupt-handler
	// whenever the controller has received some new packets. 
	//-------------------------------------------------------------- 

	struct net_device	*dev = (struct net_device*)data;
	MY_DRIVERDATA	*mdp = netdev_priv (dev);	
	RX_DESCRIPTOR		*rdq = (RX_DESCRIPTOR*)mdp->rxring;
	unsigned int		curr = mdp->rxnext;
	void			*src = phys_to_virt( rdq[ curr ].base_address );
	int			len = rdq[ curr ].packet_length;
	struct sk_buff		*skb = dev_alloc_skb( len + NET_IP_ALIGN );

	int rxtail;

	unsigned int RDT= ioread32(io+E1000_RDT);
	unsigned int RDH= ioread32(io+E1000_RDH);
	//printk("rx_handler is called,status=%d, curr=%d,rdh=%d, rdt=%d\n",rdq[curr].desc_status,curr ,RDH,RDT);

 	while(curr!=RDH){
		src = phys_to_virt( rdq[ curr ].base_address );
		len = rdq[ curr ].packet_length;
		skb = dev_alloc_skb( len + NET_IP_ALIGN );

		// clear the current descriptor's status 
		rdq[ curr ].desc_status = 0;
		rdq[ curr ].desc_errors = 0;

		// allocate a new socket-buffer
		if ( !skb ) { dev->stats.rx_dropped += 1; return; }

		// copy received packet-data into this socket-buffer
		memcpy( skb_put( skb, len ), src, len );

		// adjust the socket-buffer's parameters
		skb->dev = dev;
		skb->protocol = eth_type_trans( skb, dev );
		skb->ip_summed = CHECKSUM_NONE;

		// advance our driver's 'rxnext' index 
		curr = (++curr) % N_RX_DESC;
		mdp->rxnext=curr;

		// update the 'net_device' statistics
		dev->stats.rx_packets += 1;
		dev->stats.rx_bytes += len;

		// record the timestamp
		dev->last_rx = jiffies;

		// now hand over the socket-buffer to the kernel
		netif_rx( skb );
}  


} 


static int my_napipoll(struct napi_struct *napi,int budget){
	
	MY_DRIVERDATA	*mdp = container_of(napi, MY_DRIVERDATA,mynapi);	
	struct net_device *dev=my_netdev;
	RX_DESCRIPTOR		*rdq = (RX_DESCRIPTOR*)mdp->rxring;
	unsigned int		curr = mdp->rxnext;
	void			*src;
	int			len;
	struct sk_buff		*skb ; 

	int rxtail;

	unsigned int RDT= ioread32(io+E1000_RDT);
	unsigned int RDH= ioread32(io+E1000_RDH);
	//printk("rx_handler is called,status=%d, curr=%d,rdh=%d, rdt=%d\n",rdq[curr].desc_status,curr ,RDH,RDT);


 	while(curr!=RDH){
		src = phys_to_virt( rdq[ curr ].base_address );
		len = rdq[ curr ].packet_length;
		skb = dev_alloc_skb( len + NET_IP_ALIGN );

		// clear the current descriptor's status 
		rdq[ curr ].desc_status = 0;
		rdq[ curr ].desc_errors = 0;

		// allocate a new socket-buffer
		if ( !skb ) { dev->stats.rx_dropped += 1; return; }

		// copy received packet-data into this socket-buffer
		memcpy( skb_put( skb, len ), src, len );

		// adjust the socket-buffer's parameters
		skb->dev = dev;
		skb->protocol = eth_type_trans( skb, dev );
		skb->ip_summed = CHECKSUM_NONE;

		// advance our driver's 'rxnext' index 
		curr = (++curr) % N_RX_DESC;
		mdp->rxnext=curr;

		// update the 'net_device' statistics
		dev->stats.rx_packets += 1;
		dev->stats.rx_bytes += len;

		// record the timestamp
		dev->last_rx = jiffies;

		// now hand over the socket-buffer to the kernel
		netif_receive_skb(skb);
	}

	//stop the polling 
	//	napi_complete(struct napi_struct *napi);

}  



irqreturn_t my_isr( int irq, void *dev_id )
{
	struct net_device	*dev = (struct net_device*)dev_id;
	MY_DRIVERDATA	*mdp = netdev_priv (dev);	
	static int	reps = 0;
	unsigned long flags;
	unsigned int		intr_cause = ioread32( io + E1000_ICR );
	
	if ( intr_cause == 0 ) return IRQ_NONE;

	//	printk( "NIC2 %-2d  cause=%08X  ", ++reps, intr_cause );
	//	if ( intr_cause & (1<<0) ) printk( "TXDW " );
	//	if ( intr_cause & (1<<1) ) printk( "TXQE " );
	//	if ( intr_cause & (1<<2) ) printk( "LC " );
	if ( intr_cause & (1<<4) ) printk( "RXDMT0 " );
	//	if ( intr_cause & (1<<6) ) printk( "RXO " );
	if ( intr_cause & (1<<7) ) printk( "RXT0 " );

	//disable interrupt
	//iowrite32( 0xFFFFFFFF, io + E1000_IMC );

	spin_lock_irqsave(&mdp->mylock, flags);

	if ( intr_cause == 0 ) return IRQ_NONE;

	//		printk( "NIC2 %-2d  cause=%08X  ", ++reps, intr_cause );
	//	if ( intr_cause & (1<<0) ) printk( "TXDW " );
	//	if ( intr_cause & (1<<1) ) printk( "TXQE " );
	//	if ( intr_cause & (1<<2) ) printk( "LC " );
	//	if ( intr_cause & (1<<4) ) printk( "RXDMT0 " );
	//	if ( intr_cause & (1<<6) ) printk( "RXO " );
	//	if ( intr_cause & (1<<7) ) printk( "RXT0 " );
	//	if ( intr_cause & (1<<9) ) printk( "MDAC " );
	//	if ( intr_cause & (1<<15) ) printk( "TXDLOW " );
	//	if ( intr_cause & (1<<16) ) printk( "SRPD " );
	//	if ( intr_cause & (1<<17) ) printk( "ACK " );
	//	printk( "\n" );	

	if ( intr_cause & (1<<4) ) 	// Rx-Descriptors Low
	{
		int	rxtail = ioread32( io + E1000_RDT );
		rxtail = ( N_RX_DESC/8 + rxtail) % N_RX_DESC;
		iowrite32( rxtail, io + E1000_RDT );
 	// schedule our interrupt-handler's 'bottom-half'
 	if ( intr_cause & (1<<7) ){


#ifdef USE_NAPI 
		//disable RXT0 interrupt
		iowrite32(0x0000000080,io+E1000_IMC);
		napi_schedule( &(mdp->mynapi)); 
		//enable RXT0 interrupt
		iowrite32(0x0000000080,io+E1000_IMS);
#else
		tasklet_schedule( &mdp->rx_tasklet );
#endif

	}


	// clear these interrupts
	intr_cause=0;
	iowrite32( intr_cause, io + E1000_ICR );
	
	return	IRQ_HANDLED;
}   

	//enable interrupt
	//iowrite32( 0xFFFFFFFF, io + E1000_IMS );

	spin_unlock_irqrestore(&mdp->mylock, flags);

	return	IRQ_HANDLED;
 }    



