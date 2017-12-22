//-------------------------------------------------------------------
//	nicmap.c
//
//	This quickly-written module lets us investigate the behavior
//	of a range of registers used by the Intel Pro1000 controller 
//	to hold its most recently fetched set of RX Descriptors. 
//
//	NOTE: Written and tested with Linux kernel version 2.6.22.5.
//
//	programmer: ALLAN CRUSE
//	written on: 21 FEB 2008
//-------------------------------------------------------------------

#include <linux/module.h>	// for init_module() 
#include <linux/proc_fs.h>	// for create_proc_info_entry() 
#include <linux/pci.h>		// for pci_get_device()

#define VENDOR_ID	0x8086	// Intel Corporation
#define DEVICE_ID	0x109A	// 82573L controller
#define KMEM_SIZE	0x1000	// kernel memory size

enum	{
	E1000_CTRL	= 0x0000,
	E1000_STATUS	= 0x0008,
	E1000_IMC	= 0x00D8,
	E1000_RCTL	= 0x0100,
	E1000_RDBAL	= 0x2800,
	E1000_RDBAH	= 0x2804,
	E1000_RDLEN	= 0x2808,
	E1000_RDH	= 0x2810,
	E1000_RDT	= 0x2818,
	E1000_RXDCTL	= 0x2828,
	};

char modname[] = "nicmap";
struct pci_dev	*devp;
unsigned int	mmio_base;
unsigned int	mmio_size;
void		*io, *kmem;
unsigned int	kmem_phys;

int my_get_info( char *buf, char **start, off_t off, int count )
{
	int	len = 0;

	iowrite32( 0x000001E, io + E1000_RDT );
	iowrite32( ioread32( io + E1000_RCTL ) | (1<<1), io + E1000_RCTL );
	len += sprintf( buf+len, "\n " );
	len += sprintf( buf+len, "RDBAL=%08X  ", ioread32( io + E1000_RDBAL ) );
	len += sprintf( buf+len, "RDLEN=%08X  ", ioread32( io + E1000_RDLEN ) );
	len += sprintf( buf+len, "RDH=%08X  ", ioread32( io + E1000_RDH ) );
	len += sprintf( buf+len, "RDT=%08X  ", ioread32( io + E1000_RDT ) );
	len += sprintf( buf+len, "RXDCTL=%08X", ioread32( io + E1000_RXDCTL ) );
	len += sprintf( buf+len, "\n\n" );
	return	len;
}

static int __init nicmap_init( void )
{
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
	printk( " kmem physical address = %08X \n", kmem_phys );

	memset( kmem, 0xAA, 0x400 );

	iowrite32( 0x00000000, io + E1000_STATUS );
	iowrite32( 0xFFFFFFFF, io + E1000_IMC );
	iowrite32( 0x040C0241, io + E1000_CTRL );
	iowrite32( 0x000C0241, io + E1000_CTRL );
	while ( ( ioread32( io + E1000_STATUS ) & 3 ) != 3 );

	iowrite32( 0x0000801C, io + E1000_RCTL );
	iowrite32(  kmem_phys, io + E1000_RDBAL );
	iowrite32( 0x00000000, io + E1000_RDBAH ); 
	iowrite32( 0x00000400, io + E1000_RDLEN ); 
	iowrite32( 0x01010000, io + E1000_RXDCTL ); 

	pci_read_config_word( devp, 4, &pci_cmd );
	pci_cmd |= (1<<2);
	pci_write_config_word( devp, 4, pci_cmd );

	create_proc_info_entry( modname, 0, NULL, my_get_info );
	return	0;  //SUCCESS
}

static void __exit nicmap_exit(void )
{
	remove_proc_entry( modname, NULL );
	iowrite32( ioread32( io + E1000_RCTL ) & ~(1<<1), io + E1000_RCTL );
	kfree( kmem );
	iounmap( io );
	printk( "<1>Removing \'%s\' module\n", modname );
}

module_init( nicmap_init );
module_exit( nicmap_exit );
MODULE_LICENSE("GPL"); 

