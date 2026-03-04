/***********************************************************************
*
* Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
* See LICENSE file in the project root for full license information.
*
************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/pci-dma.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include "gti_drv.h"
#include "gti2803.h"
#include "gti_ioctl.h"

//char * gti_2803_buffer;

static ssize_t show_gti_info(struct device *dev, struct device_attribute *attr, char *buf){

	return 0;
}

static DEVICE_ATTR(gti_info, S_IRUGO, show_gti_info, NULL);

static int gti_open(struct inode *inode, struct file *file)
{

	GTI2803 * pGti2803 = container_of(inode->i_cdev, GTI2803, ch_dev);
	mutex_lock(&pGti2803->mlock);
	if(pGti2803->count !=0) {mutex_unlock(&pGti2803->mlock);return -EPERM;}
	pGti2803->count=1;

	//printk( "%s:%d:%s:%p\n", __FILE__, __LINE__, __func__,(void*)pGti2803->buffer);
	mutex_unlock(&pGti2803->mlock);
	file->private_data = pGti2803;

	return 0;
}

static int gti_close(struct inode *inode, struct file *file)
{
	GTI2803 * pGti2803 = (GTI2803 *)file->private_data;
	pGti2803->count=0;
	return 0;
}
static ssize_t gti_read(struct file *file, char __user *buf, size_t count,loff_t *pos)
{
	GTI2803 * pGti2803;
	pGti2803 = (GTI2803 *)file->private_data;

	size_t n;
	size_t ndwords, nleft, chunk_size;
	size_t word_size;
	word_size = sizeof(u32);
	u32 *p;
	volatile void * rtn_addr;
	int i;
	int fip = (int)iminor(file->f_inode);
	
	//printk( "%s:%d:%s\n", __FILE__, __LINE__, __func__);

	n = (count > GYR_BUFSIZE) ? GYR_BUFSIZE : count;
	ndwords = (n + (word_size - 1)) / word_size;
	nleft = ndwords;

	rtn_addr = (void *)(pGti2803->pio_bar + GYR_DATA_RTN_OFFSET(fip));
	p = (u32 *)pGti2803->buffer;

	while(nleft > 0 && (chunk_size = MIN(GYR_BUFSIZE, nleft)) > 0) {
		for (i = 0;  i < chunk_size;  i++) {
			*p = *((volatile u32 *)rtn_addr);
			p++;
		}
		nleft -= chunk_size;
	}

	n = (ndwords - nleft) * word_size;

	if (copy_to_user(buf, pGti2803->buffer, n) != 0) {
	    	return -EFAULT;
	}

	return count;
}

static ssize_t write_packed_32(int fip, size_t length, GTI2803 * pGti2803)
{
	ssize_t nleft;
	ssize_t size;
	u8 *p;
	u8  *c;
	void * post_addr;

	//printk( "%s:%d:%s\n", __FILE__, __LINE__, __func__);

	post_addr = (void *)(pGti2803->pio_bar + GYR_DATA_POST_OFFSET(fip));
	p = pGti2803->buffer;
	c = (u8 *)pGti2803->buffer;

	nleft = (ssize_t)length;

#if 0
	size = sizeof(u64);
	while (nleft >= size) {
		log_verbose("gti_write:  addr=%p, data=0x%x [%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x]\n",
			post_addr, *((u64 *)p), c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]);
		*((volatile u64 *)post_addr) = *((u64 *)p);
		p += size;
		nleft -= size;
		c = p;
	}
#endif
	size = sizeof(u32);
	while (nleft >= size) {
	/*	printk("gti_write:  addr=%p, data=0x%x [%.2x %.2x %.2x %.2x]\n", post_addr, *((u32 *)p),
				c[0], c[1], c[2], c[3]);*/
		*((volatile u32 *)post_addr) = *((u32 *)p);
		p += size;
		nleft -= size;
		c = p;
	}

	if (nleft > 0) {
		printk(KERN_ERR "write_packed_32: length not a multiple of 4!  length = %lu\n", length);
	}

	return length;
}

static ssize_t gti_write(struct file *file, const char __user *buf,size_t count, loff_t *pos)
{
	size_t n;
	int fip = (int)iminor(file->f_inode);
	GTI2803 * pGti2803;
	pGti2803 = (GTI2803 *)file->private_data;

	//printk( "%s:%d:%s\n", __FILE__, __LINE__, __func__);
	//printk("gti2803_write: count = %lu, offset = %llu\n", count, *pos);

	n = (count > GYR_BUFSIZE) ? GYR_BUFSIZE : count;

	copy_from_user(pGti2803->buffer, buf, n);

	return write_packed_32(fip, n, pGti2803);
}

void gti_vma_open(struct vm_area_struct *vma)
{
	printk(KERN_NOTICE "GTI VMA open, virt %lx, phys %lx\n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void gti_vma_close(struct vm_area_struct *vma)
{
	printk(KERN_NOTICE "GTI VMA close.\n");
}

static struct vm_operations_struct gti_mmap_vm_ops = {
		.open = gti_vma_open,
		.close = gti_vma_close,
};

static int gti_mmap(struct file *file, struct vm_area_struct *vma)
{
	//todo: add mmap and put data into dma buffer directly
	return -EINVAL;
	//return -1;
}

static long gti_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	return 0;

}

const static struct file_operations gti_fops = {
	.owner = THIS_MODULE,
	.open = gti_open,
	.release = gti_close,
	.read = gti_read,
	.write = gti_write,
	.mmap = gti_mmap,
	.unlocked_ioctl = gti_ioctl,
};


int gti2803_remove(void *gti_dev){
	GTI_DEV *pGti=(GTI_DEV *)gti_dev;
	GTI2803 * pGti2803=(GTI2803 * )pGti->private;

	pci_iounmap(pGti->pDev,pGti2803->control_bar);

	device_remove_file(&pGti->pDev->dev, &dev_attr_gti_info);
	cdev_del(&pGti2803->ch_dev);
	device_destroy(pGti->pClass,pGti2803->devBase);
	unregister_chrdev_region(pGti2803->devBase,1);

	kfree(pGti2803);

	return 0;
}

int gti2803_init(void *gti_dev){
	int rc = 0;
	GTI_DEV * pGti = (GTI_DEV *)gti_dev;
	GTI2803 * pGti2803;

	printk( "%s:%d:%s\n", __FILE__, __LINE__, __func__);

	//unsigned long basePAddr, baseLen;

	pGti2803 = kzalloc(sizeof(GTI2803), GFP_KERNEL); if(!pGti2803) return -ENOMEM;
	pGti->private=pGti2803;
	pGti2803->count=0;
	pGti2803->mode64=0;
	mutex_init(&pGti2803->mlock);
	rc = device_create_file(&pGti->pDev->dev, &dev_attr_gti_info);

	pGti2803->control_bar=pci_iomap(pGti->pDev, 0, pci_resource_len(pGti->pDev, 0));
	//printk("gti_control_bar:  addr=%p \n", (void *)pGti2803->control_bar);

	pGti2803->pio_bar=pci_iomap(pGti->pDev, 1, pci_resource_len(pGti->pDev, 1));
	//printk("gti_pio_bar:  addr=%p \n", (void *)pGti2803->pio_bar);

	rc = alloc_chrdev_region(&pGti2803->devBase, 0,1, "GTI2803_drv");

	*((u32 *)pGti2803->pio_bar) = 0;

	if(rc==0){
		struct device * pDev;
		cdev_init(&pGti2803->ch_dev, &gti_fops);
		pGti2803->ch_dev.owner=THIS_MODULE;
		rc=  cdev_add(&pGti2803->ch_dev, MKDEV(MAJOR(pGti2803->devBase),0), 1);
		if(rc!=0){ unregister_chrdev_region(pGti2803->devBase,1); return -1;}
		pDev=device_create(pGti->pClass,&pGti->pDev->dev,MKDEV(MAJOR(pGti2803->devBase),0),0,"gti2803-%d",pGti->instance);
		if(pDev==0){
			rc=-ENOMEM;
			cdev_del(&pGti2803->ch_dev);
			unregister_chrdev_region(pGti2803->devBase,1);
		}
	}

	return rc;
}
