
#include "mymemdev.h"
#include "mymemdev_ioctl.h"
//#include "proc.c"
struct MYMEMDEV *mymem_dev;
static int mymem_major = 0;

//static int mymem_ioctl(struct inode *inode,struct files *files, unsigned int cmd,unsigned long arg) {
static int mymem_ioctl(struct files *files, unsigned int cmd,unsigned long arg) {
	static int test_var=0;
	int err=0,retval=0;
	if(_IOC_TYPE(cmd)!=MYMEMDEV_IOC_MAGIC)//please refer to ioctl_magic.txt for the magic number
	{
		MYDEBUG("IOC_TYPE %d !=MAGIC %d\n",_IOC_TYPE(cmd),MYMEMDEV_IOC_MAGIC);
		return -EINVAL;
	}	
	
	if(_IOC_NR(cmd)>MYMEMDEV_IOC_MAXNR)
 	{
		MYDEBUG("IOC_NR %d !=MAXNR %d\n",_IOC_NR(cmd),MYMEMDEV_IOC_MAXNR);
		return -EINVAL;
	}
	// Parameter check	
	if(_IOC_DIR(cmd)& _IOC_READ){
		err=!access_ok(VERIFY_WRITE,(void __user*) arg,_IOC_SIZE(cmd));
//		MYDEBUG("access_ok(VERIFY_WRITE)=%d\n",err);
	} 
 	else{ 
 		if(_IOC_DIR(cmd)& _IOC_WRITE){
		err=!access_ok(VERIFY_READ,(void __user*) arg,_IOC_SIZE(cmd));
//		MYDEBUG("access_ok(VERIFY_READ)=%d\n",err);
		}
	} 
	if(err){ 
		MYDEBUG("<---Command error--->\n");
		return -EFAULT;
	}
	MYDEBUG("<---Switch Cmd =%x,arg=%d--->\n",cmd,arg);

	switch(cmd){
		case MYMEMDEV_IOC_PRINT:
			//printk("<---ORG IOCPRINT :test_var=%d--->\n",test_var);
			retval=__get_user(test_var,(int __user *)arg);		
			printk("<---IOCPRINT complete:test_var=%d--->\n",test_var);
			//printk("<---IOCPRINT complete:retval=%d--->\n",retval);
			break;

		case MYMEMDEV_IOC_GETDATA:
			//MYDEBUG("<---IOCGETDATA--->\n");
			retval=__put_user(test_var,(int __user *)arg);		
			MYDEBUG("<---IOCGETDATA complete:test_var=%d--->\n",test_var);
			break;

		case MYMEMDEV_IOC_SETDATA:
			//printk("<---IOCSETDATA--->\n");
			retval=__get_user(test_var,(int __user *)arg);		
			printk("<---IOCSETDATA complete:test_var=%d--->\n",test_var);
			break;

		default:
			printk("Going to default\n");
			return -EINVAL;
	}
	return retval;
}

static int mymem_open(struct inode *inode,struct file *filp){
	struct MYMEMDEV *dev;
	// is it ok without check minor number
	dev=container_of(inode->i_cdev,struct MYMEMDEV,mycdev);
	filp->private_data=dev;
	return 0;
}   

static int mymem_release(struct inode *inode, struct file *filp){
	return 0;
}  

static ssize_t mymem_read(struct file *filp,
		char __user *buf,  /* The buffer in user space to fill with data */
		size_t size, /* The length of the buffer     */
		loff_t *ppos) /* Our offset in the file       */
{

	unsigned long p=*ppos;
	unsigned int count=size;
	MYDEBUG("unsigned p=%lu, size=%u, count=%u =============\n",p,size,count);
	MYDEBUG("signed p=%d, size=%d, count=%d =============\n",p,size,count);
	int ret=0;
	MYDEBUG("=========read initial============\n");
	struct MYMEMDEV *dev=filp->private_data;
	down_interruptible(&(dev -> sem));
	if(dev==NULL){
		MYDEBUG("==========dev = NULL=============\n");
	} 

	if(p>=DATA_SIZE)
		return 0;
	MYDEBUG(" p>data_size  p=%d, size=%d, count=%d =============\n",p,size,count);
	if(count > DATA_SIZE-p)
		count=DATA_SIZE-p;

	MYDEBUG("count>data_size p=%d, size=%d, count=%d, DATA_SIZE-p=%d =============\n",p,size,count,DATA_SIZE-p);
	//MYDEBUG("=========read 1============\n");
	count=4;
	if(copy_to_user(buf,(void *)(dev->data+p),count)){
		ret=-EFAULT;
		goto out;
	}else{
		MYDEBUG(KERN_INFO "read %d bytes from %p\n",count, ppos);
		*ppos+=count;
		ret = count;
		MYDEBUG(KERN_INFO "read %d bytes from %p\n",count, ppos);
	}
out:
	//up(&dev->sem);
	//MYDEBUG("=========read 2============\n");
	up(&(dev -> sem));
	return ret;

}  

static loff_t mymem_llseek(struct file *filp, loff_t offset, int whence){
	loff_t newpos;

	MYDEBUG("in llseek %d\n",offset);
	switch(whence){ 
		case 0://move to begin
			newpos=offset;
			break;
		case 1://move to curr
			newpos=filp->f_pos+offset;
			break;
		case 2://move to the end
			newpos=DATA_SIZE-1+offset;
			break;
		default:
			return -EINVAL;
	}
	if((newpos<0)||(newpos>DATA_SIZE))//out of range
		return -EINVAL;
	filp->f_pos=newpos;
	return newpos;
} 

static ssize_t mymem_write(struct file *filp,const char __user *buf,size_t size,loff_t *ppos){
	unsigned long p=*ppos;

	unsigned int count=size;
	int ret=0;
	struct MYMEMDEV *dev=filp->private_data;
	if(dev==NULL){ 
		MYDEBUG("==========dev NULL=============\n");
	}

	down_interruptible(&(dev -> sem));
	if(p>=DATA_SIZE)
		return 0;
	if(count > DATA_SIZE-p)
		count=DATA_SIZE-p;
	if(copy_from_user(dev->data + p,buf,count)){
		ret=-EFAULT;
	}else{
		MYDEBUG(KERN_INFO "write %d bytes from %p\n",count,ppos );
		*ppos+=count;
		ret = count;
		MYDEBUG(KERN_INFO "write %d bytes from %p\n",count,ppos );
	} 
	MYDEBUG("ret=%d\n",ret);

	up(&(dev -> sem));
	return ret;
}

static const struct file_operations mymem_fops={
	.owner=THIS_MODULE,
	.llseek=mymem_llseek,
	.read=mymem_read,
	.write=mymem_write,
	.release=mymem_release,
	.open=mymem_open,
	.unlocked_ioctl=mymem_ioctl

};

static int mymemdev_init (void){
	int result,i,err,minor;
	//	myproc_init();
	MYDEBUG("=====mymemdev_init========\n");
	dev_t mydev=MKDEV(mymem_major,0);
	result = alloc_chrdev_region(&mydev,0,MEMDEV_NR_DEVS,DRIVER_NAME);
	MYDEBUG("alloc_chrdev_region result=%d\n",result);
	if(result<0){
		goto error;//register or alloc error
	}
	mymem_major=MAJOR(mydev);

	printk("=====mymem_major=%d========\n",mymem_major);

	MYDEBUG("=====kmalloc for mymem_dev========\n");
	mymem_dev=kmalloc((sizeof(struct MYMEMDEV))*MEMDEV_NR_DEVS,GFP_KERNEL);

	if(!mymem_dev){ 
		result=-ENOMEM;
		goto error;
	}

	MYDEBUG("=====initial multiple devices========\n");
	// for multiple device, we need multiple memory and multiple cdev
	for  (i = 0; i < MEMDEV_NR_DEVS; i++) {
		minor=i;
		mymem_dev[i].data=kmalloc(sizeof(char)*DATA_SIZE,GFP_KERNEL);
		MYDEBUG("=====cdev_init========\n");
		cdev_init(&(mymem_dev[i].mycdev),&mymem_fops);
		MYDEBUG("=====cdev_add========\n");
		err=cdev_add(&(mymem_dev[i].mycdev),MKDEV(mymem_major,minor),1); //add many cdev
		if(err){ 
			MYDEBUG("cdev[%d] adding error\n",i);
		}
		sema_init(&mymem_dev[i].sem,1);	
	}

	MYDEBUG("=====init finish========\n");

error:	//error redo recycle
	return result;
}

static void mymemdev_exit(void){

	MYDEBUG("=====mymemdev exit========\n");
	cdev_del(&(mymem_dev->mycdev));
	MYDEBUG("=====free mymemdev->data========\n");
	kfree(mymem_dev->data);
	MYDEBUG("=====free mymemdev========\n");
	kfree(mymem_dev);

	MYDEBUG("=====unregister mymemdev========\n");
	//MYDEBUG("=====mymemdev=%d ,MEMDEV_MAJOR=%d,========\n",mymem_major,MEMDEV_MAJOR);

	unregister_chrdev_region(MKDEV(mymem_major,0),MEMDEV_NR_DEVS);
	//myproc_exit();

	MYDEBUG("=====exit finish========\n");
}

MODULE_LICENSE("GPL");
module_init(mymemdev_init);
module_exit(mymemdev_exit);


