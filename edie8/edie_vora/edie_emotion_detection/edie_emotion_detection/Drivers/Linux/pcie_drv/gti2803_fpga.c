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
#include "gti2803_fpga.h"
#include "gti_ioctl.h"


static ssize_t show_gti_info(struct device *dev, struct device_attribute *attr, char *buf){
	u32 Id[4];
	struct pci_dev * pdev = container_of(dev, struct pci_dev, dev);
	GTI_DEV *pGti = (GTI_DEV *)pci_get_drvdata(pdev);
	GTI2803_FPGA * pGti2803=(GTI2803_FPGA * )pGti->private;
	struct control_regs *reg=(struct control_regs*)(pGti2803->control_bar+REG_OFFSET_CONTROL);
	mutex_lock(&pGti2803->mlock);
	Id[0]=read_register(&reg->identifier[0]);
	Id[1]=read_register(&reg->identifier[1]);
	Id[2]=read_register(&reg->timestamp);
	Id[3]=read_register(&reg->temperature);
	mutex_unlock(&pGti2803->mlock);
	sprintf(buf,"Id0: %08x,Id1: %08x\nFPGA Build Data: %d/%02d/%02d-%02d:%02d:%02d\nTemperature:%d deg. C\n",Id[0],Id[1],
		((Id[2]>>17)&0x3f)+2000,(Id[2]>>23)&0xf,(Id[2]>>27)&0x1f,(Id[2]>>12)&0x1f,(Id[2]>>06)&0x3f,Id[2]&0x3f,
			((0xffff&Id[3])*503)/65536-273 );

	return strlen(buf);
}

static DEVICE_ATTR(gti_info, S_IRUGO, show_gti_info, NULL);

static int gyr_reset_device(struct file *file) {
	GTI2803_FPGA * pGti2803 = (GTI2803_FPGA *)file->private_data;
	struct control_regs *reg=pGti2803->control_bar + REG_OFFSET_CONTROL;

	u32 v=read_register(&reg->reset);
	write_register(v|(0x1L<<12),&reg->reset);
	mdelay(5);
	write_register(v&(~(0x1L<<12)),&reg->reset);
	return 0;
}


static int gti_open(struct inode *inode, struct file *file)
{
	GTI2803_FPGA * pGti2803 = container_of(inode->i_cdev, GTI2803_FPGA, ch_dev);
	mutex_lock(&pGti2803->mlock);
	if(pGti2803->count !=0) {mutex_unlock(&pGti2803->mlock);return -EPERM;}
	pGti2803->count=1;
	mutex_unlock(&pGti2803->mlock);
	file->private_data = pGti2803;

	return 0;
}

static int gti_close(struct inode *inode, struct file *file)
{
	GTI2803_FPGA * pGti2803 = (GTI2803_FPGA *)file->private_data;
	pGti2803->count=0;
	return 0;
}

static ssize_t gti_read(struct file *file, char __user *buf, size_t count,loff_t *pos)
{
	size_t ret,i;
	GTI2803_FPGA * pGti2803 = (GTI2803_FPGA *)file->private_data;
	u32 *p=(u32 *)pGti2803->buffer;
	u32 * fifo_addr= (u32 *)((u8 *)pGti2803->control_bar + REG_OFFSET_FIFO);

	pr_debug("In gti_read: length = %lu\n", count);

	if(count%sizeof(int)) 	return -EFAULT;

	ret = (count> BUFFER_SIZE) ? BUFFER_SIZE : count;

	for(i=0;i<ret;i+=sizeof(int))	*p++=read_register(fifo_addr);

	if (copy_to_user(buf,pGti2803->buffer , ret) != 0)    	return -EFAULT;

	return ret;
}

static ssize_t gti_write(struct file *file, const char __user *buf,size_t count, loff_t *pos)
{
	size_t ret,i;
	GTI2803_FPGA * pGti2803 = (GTI2803_FPGA *)file->private_data;
	u32 *p=(u32 *)pGti2803->buffer;
	u32 * fifo_addr= (u32 *)((u8 *)pGti2803->control_bar + REG_OFFSET_FIFO);

	pr_debug("In gti_write: length = %lu\n", count);

	if(count%sizeof(int)) 	return -EFAULT;

	ret = (count> BUFFER_SIZE) ? BUFFER_SIZE : count;

	if(copy_from_user(p, buf, ret)!=0) return -EFAULT;

	for(i=0;i<ret;i+=sizeof(int))	write_register(*p++,fifo_addr);


	return ret;
}

static int gti_mmap(struct file *file, struct vm_area_struct *vma)
{
	//todo: add mmap and put data into dma buffer directly
	return -EINVAL;
}

static long gti_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	switch (cmd) {
		case ICHIP_RESET:
			pr_debug("ICHIP_RESET\n");
			gyr_reset_device(file);
			break;

		default:
			printk(KERN_ERR "Invalid command %d\n", cmd);
			return  -1;
	}

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


int gti2803_fpga_remove(void *gti_dev){
	GTI_DEV *pGti=(GTI_DEV *)gti_dev;
	GTI2803_FPGA * pGti2803=(GTI2803_FPGA * )pGti->private;

	pci_iounmap(pGti->pDev,pGti2803->control_bar);

	device_remove_file(&pGti->pDev->dev, &dev_attr_gti_info);
	cdev_del(&pGti2803->ch_dev);
	device_destroy(pGti->pClass,pGti2803->devBase);
	unregister_chrdev_region(pGti2803->devBase,1);

	kfree(pGti2803);

	return 0;
}

int gti2803_fpga_init(void *gti_dev){
	int rc = 0;
	GTI_DEV *pGti=(GTI_DEV *)gti_dev;
	GTI2803_FPGA * pGti2803;

    pGti2803 = kzalloc(sizeof(GTI2803_FPGA), GFP_KERNEL); if(!pGti2803) return -ENOMEM;
    pGti->private=pGti2803;
    pGti2803->count=0;
    pGti2803->mode64=0;
    mutex_init(&pGti2803->mlock);
    rc = device_create_file(&pGti->pDev->dev, &dev_attr_gti_info);

    pGti2803->control_bar=pci_iomap(pGti->pDev, 0, pci_resource_len(pGti->pDev, 0));

	rc = alloc_chrdev_region(&pGti2803->devBase, 0,1, "GTI2803_drv");

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
		}else{
		    struct axis_switch_regs * switch_reg;
			switch_reg= (struct axis_switch_regs *)((u8 *)pGti2803->control_bar + REG_OFFSET_SWITCH);
			write_register(3,&(switch_reg->sda_post));
			mb();
			write_register(1,&(switch_reg->c2h_fifo));
			mb();
			write_register(2,&(switch_reg->command));
			mb();
		}
	}
	return rc;
}
