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
#include <linux/interrupt.h>

#include "gti_drv.h"
#include "gti2803_dma.h"
#include "gti_ioctl.h"

static ssize_t show_gti_info(struct device *dev, struct device_attribute *attr, char *buf){
	u32 Id[4];
	struct pci_dev * pdev = container_of(dev, struct pci_dev, dev);
	GTI_DEV * gti_dev = (GTI_DEV *)pci_get_drvdata(pdev);
	struct fpga_dev *fpga = (struct fpga_dev *)gti_dev->private;
	struct gti_control_regs *reg=(struct gti_control_regs*)fpga->control_bar;
	spin_lock(&fpga->slock);
	Id[0]=read_register(&reg->fpga_id[0]);
	Id[1]=read_register(&reg->fpga_id[4]);
	Id[2]=read_register(&reg->fpga_id[8]);
	Id[3]=read_register(&reg->fpga_id[0xc]);
	spin_unlock(&fpga->slock);
	sprintf(buf,"Id0: %08x,Id1: %08x\nFPGA Build Data: %d/%02d/%02d %02d/%02d/%02d\nTemperature:%d deg. C\n",Id[0],Id[1],
		((Id[2]>>17)&0x3f)+2000,(Id[2]>>23)&0xf,(Id[2]>>27)&0x1f,(Id[2]>>12)&0x1f,(Id[2]>>06)&0x3f,Id[2]&0x3f,
			((0xffff&Id[3])*503)/65536-273 );

	return strlen(buf);
}

static ssize_t show_pll_info(struct device *dev, struct device_attribute *attr, char *buf){
	u32 value;
	struct pci_dev * pdev = container_of(dev, struct pci_dev, dev);
	GTI_DEV * gti_dev = (GTI_DEV *)pci_get_drvdata(pdev);
	struct fpga_dev *fpga = (struct fpga_dev *)gti_dev->private;
	struct gti_control_regs *reg=(struct gti_control_regs*)fpga->control_bar;
	spin_lock(&fpga->slock);
	value=read_register(&reg->pll_control);
	spin_unlock(&fpga->slock);
	sprintf(buf,"pll offset:0x%px,setting: 0x%08x\n",&reg->pll_control,value );
	return strlen(buf);
}

static ssize_t store_pll_info(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	u32 value;
	struct pci_dev * pdev = container_of(dev, struct pci_dev, dev);
	GTI_DEV * gti_dev = (GTI_DEV *)pci_get_drvdata(pdev);
	struct fpga_dev *fpga = (struct fpga_dev *)gti_dev->private;
	struct gti_control_regs *reg=(struct gti_control_regs*)fpga->control_bar;
    sscanf(buf,"%x",&value);
	spin_lock(&fpga->slock);
	write_register(value, &reg->pll_control);
	spin_unlock(&fpga->slock);
	return count;
}

static DEVICE_ATTR(gti_info, S_IRUGO, show_gti_info, NULL);
static DEVICE_ATTR(pll_info, S_IRUSR | S_IWUSR, show_pll_info, store_pll_info);

static int gti_open(struct inode *inode, struct file *file)
{
	struct gti_dev * gti_char = container_of(inode->i_cdev, struct gti_dev, chdev);
	//printk( "%s:%d:%s\n", __FILE__, __LINE__, __func__);	
	spin_lock(&gti_char->fpga->slock);
	if(gti_char->count !=0) {spin_unlock(&gti_char->fpga->slock);return -EPERM;}
	gti_char->count=1;
	spin_unlock(&gti_char->fpga->slock);
	file->private_data = gti_char;

	return 0;
}

static int gti_close(struct inode *inode, struct file *file)
{
	struct gti_dev *gti_char = (struct gti_dev *)file->private_data;
	gti_char->count=0;
	return 0;
}

static ssize_t gti_read(struct file *file, char __user *buf, size_t count,loff_t *pos)
{
	int ret=count;
	int rc;
	void * addr;
	u32 data_available;

	//printk( "%s:%d:%s\n", __FILE__, __LINE__, __func__);
	struct gti_dev *gti;
	gti= (struct gti_dev *)file->private_data;
	struct interrupt_regs * int_reg=(struct interrupt_regs *)(gti->fpga->dma_bar + REG_OFFSET_INTERRUPT);

	gti_debug("read %s. size:%d\n",file->f_path.dentry->d_iname,(int)count);
	if(count >gti->fpga->buffer_len ) return -1;
	
#if 0
	spin_lock(&gti->fpga->slock);
	addr=gti->fpga->control_bar + GYR_RX_FIFO_FULLNESS_OFFSET(gti->channel);
	data_available=read_register(addr);
	spin_unlock(&gti->fpga->slock);
	gti_debug("read:data_available:%d\n",data_available);
	if(data_available<count) return 0;
#else
	down(&gti->fpga->user_wq_sema[gti->channel]);
	gti->fpga->user_q_state[gti->channel]=0;
	{
		spin_lock(&gti->fpga->slock);
                write_register((0x10 << gti->channel), &int_reg->user_int_enable_w1s);     
		spin_unlock(&gti->fpga->slock);
	}
	//rc= wait_event_interruptible_timeout(gti->fpga->wq,gti->fpga->state!=0,HZ*2);
	rc= wait_event_interruptible_timeout(gti->fpga->wq,gti->fpga->user_q_state[gti->channel]!=0,HZ*2);
        if(rc<=1){
        	up(&gti->fpga->user_wq_sema[gti->channel]);
		//return -1;
	}
#endif

    down(&gti->fpga->sema);
	gti->fpga->state=0;
	/*down(&gti->fpga->user_wq_sema[gti->channel]);
        gti->fpga->user_q_state[gti->channel]=0;*/
	{
		u32 w;
		struct xdma_desc *desc= (struct xdma_desc *)gti->fpga->rx_descriptor;
		struct engine_regs * c2h_reg= (struct engine_regs *)(gti->fpga->dma_bar + REG_OFFSET_ENGINE+REG_C2H_OFFSET);
		//struct interrupt_regs * int_reg=(struct interrupt_regs *)(gti->fpga->dma_bar + REG_OFFSET_INTERRUPT);

		desc->control= DESC_MAGIC | 0x13;
		desc->bytes = count;
		desc->dst_addr_hi= cpu_to_le32(PCI_DMA_H(virt_to_phys(gti->buffer)));
		desc->dst_addr_lo= cpu_to_le32(PCI_DMA_L(virt_to_phys(gti->buffer)));
		gti_debug("dst addr:%08x;%08x\n",(desc->dst_addr_hi),(desc->dst_addr_lo));
		desc->src_addr_hi= 0;
		desc->src_addr_lo= (gti->channel * GYR_FIP_SPACING) ;
		desc->next_hi=desc->next_lo=0;

		w = (u32)XDMA_CTRL_RUN_STOP;
		w |= (u32)XDMA_CTRL_IE_READ_ERROR;
		w |= (u32)XDMA_CTRL_IE_DESC_ERROR;
		w |= (u32)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
		w |= (u32)XDMA_CTRL_IE_MAGIC_STOPPED;
		w |= (u32)XDMA_CTRL_IE_DESC_STOPPED;
		w |= (u32)XDMA_CTRL_IE_DESC_COMPLETED;
		w |= (u32)XDMA_CTRL_NON_INCR_ADDR;

	    spin_lock(&gti->fpga->slock);
		write_register(0xffffffff, &int_reg->channel_int_enable_w1s);
		//write_register((0x01 << gti->channel), &int_reg->channel_int_enable_w1s);
	    	write_register(w, &c2h_reg->control);
		read_register(&c2h_reg->status);
	    spin_unlock(&gti->fpga->slock);
	}

	rc= wait_event_interruptible_timeout(gti->fpga->wq,gti->fpga->state!=0,HZ*2);
	if(rc<=1){
        up(&gti->fpga->sema);
        printk(KERN_ERR "read wait_event_interruptible_timeout:%d\n",rc);	/*return -1;*/	
    }
    if(buf!=0){
	    rc=copy_to_user(buf,gti->buffer,count);
	    if(rc!=0){printk(KERN_ERR "copy_to_user error.rc:%d\n",rc);	/*return -1;*/}
    }
	return ret;
static ssize_t show_gti_info(struct device *dev, struct device_attribute *attr, char *buf){    u32 Id[4];    struct pci_dev * pdev = container_of(dev, struct pci_dev, dev);    GTI_DEV *pGti = (GTI_DEV *)pci_get_drvdata(pdev);    GTI2803_FPGA * pGti2803=(GTI2803_FPGA * )pGti->private;    struct control_regs *reg=(struct control_regs*)(pGti2803->control_bar+REG_OFFSET_CONTROL);    mutex_lock(&pGti2803->mlock);    Id[0]=read_register(&reg->identifier[0]);    Id[1]=read_register(&reg->identifier[1]);    Id[2]=read_register(&reg->timestamp);    Id[3]=read_register(&reg->temperature);    mutex_unlock(&pGti2803->mlock);    sprintf(buf,"Id0: %08x,Id1: %08x\nFPGA Build Data: %d/%02d/%02d-%02d:%02d:%02d\nTemperature:%d deg. C\n",Id[0],Id[1],        ((Id[2]>>17)&0x3f)+2000,(Id[2]>>23)&0xf,(Id[2]>>27)&0x1f,(Id[2]>>12)&0x1f,(Id[2]>>06)&0x3f,Id[2]&0x3f,            ((0xffff&Id[3])*503)/65536-273 );    return strlen(buf);}
}

static ssize_t gti_write(struct file *file, const char __user *buf,size_t count, loff_t *pos)
{
	int ret=count;
	int rc;
	struct gti_dev *gti = (struct gti_dev *)file->private_data;

	gti_debug("write %s. size:%d\n",file->f_path.dentry->d_iname,(int)count);
	if(count >gti->fpga->buffer_len ) return -1;
    if(buf!=0){
	    rc=copy_from_user(gti->buffer,buf,count);
	    if(rc!=0){ printk(KERN_ERR "copy_from_user error.rc:%c\n",rc);return -1;}
    }
    down(&gti->fpga->sema);
	gti->fpga->state=0;
	{
		u32 w;
		struct xdma_desc *desc= (struct xdma_desc *)gti->fpga->tx_descriptor;
		struct engine_regs * h2c_reg= (struct engine_regs *)(gti->fpga->dma_bar + REG_OFFSET_ENGINE);
		struct interrupt_regs * int_reg=(struct interrupt_regs *)(gti->fpga->dma_bar + REG_OFFSET_INTERRUPT);

		desc->control= DESC_MAGIC | 0x13;
		desc->bytes = count;
		desc->src_addr_hi= cpu_to_le32(PCI_DMA_H(virt_to_phys(gti->buffer)));
		desc->src_addr_lo= cpu_to_le32(PCI_DMA_L(virt_to_phys(gti->buffer)));
		gti_debug("src addr:%08x;%08x\n",(desc->src_addr_hi),(desc->src_addr_lo));
		desc->dst_addr_hi= 0;
		desc->dst_addr_lo= (gti->channel * GYR_FIP_SPACING);
		desc->next_hi=desc->next_lo=0;

		w = (u32)XDMA_CTRL_RUN_STOP;
		w |= (u32)XDMA_CTRL_IE_READ_ERROR;
		w |= (u32)XDMA_CTRL_IE_DESC_ERROR;
		w |= (u32)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
		w |= (u32)XDMA_CTRL_IE_MAGIC_STOPPED;
		w |= (u32)XDMA_CTRL_IE_DESC_STOPPED;
		w |= (u32)XDMA_CTRL_IE_DESC_COMPLETED;
		w |= (u32)XDMA_CTRL_NON_INCR_ADDR;

	    spin_lock(&gti->fpga->slock);
		write_register(0xffffffff, &int_reg->channel_int_enable_w1s);
		write_register(w, &h2c_reg->control);
		read_register(&h2c_reg->status);
	    spin_unlock(&gti->fpga->slock);
	}

	rc= wait_event_interruptible_timeout(gti->fpga->wq,gti->fpga->state!=0,HZ*2);
	if(rc<=1){
        up(&gti->fpga->sema);
        printk(KERN_ERR "write wait_event_interruptible_timeout:%d\n",rc);	/*return -1;*/	
    }
	return ret;
}

static void gti_vma_open(struct vm_area_struct *vma)
{
    printk(KERN_NOTICE "GTI VMA open, virt %lx, phys %lx\n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static void gti_vma_close(struct vm_area_struct *vma)
{
    printk(KERN_NOTICE "GTI VMA close.\n");
}

static struct vm_operations_struct gti_mmap_vm_ops = {
    .open = gti_vma_open,
    //.fault = gti_vma_fault,
    .close = gti_vma_close,
};

static int gti_mmap(struct file *file, struct vm_area_struct *vma)
{
    //todo: add mmap and put data into dma buffer directly
    struct page *page = NULL;
    unsigned long size = 0;
    struct gti_dev *gti;
    gti= (struct gti_dev *)file->private_data;
    size = (unsigned long)(vma->vm_end - vma->vm_start);
    if(size >gti->fpga->buffer_len ) {
        printk(KERN_ERR "GTI mmap error. size: %d/%d \n", (int)size, (int)gti->fpga->buffer_len);
        return -EINVAL;
    }
    
    page = virt_to_page(gti->buffer);

    if(remap_pfn_range(vma, vma->vm_start, page_to_pfn(page), size, vma->vm_page_prot))
        return -EINVAL;

    vma->vm_ops = &gti_mmap_vm_ops;
    gti_vma_open(vma);
    return 0;
}

static long gti_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int rc,fip_cmd;
	void *addr;
	struct gti_dev *gti = (struct gti_dev *)file->private_data;

	switch (cmd) {
	case ICHIP_RESET:
		{
			addr=gti->fpga->control_bar + GYR_CMD_OFFSET(gti->channel);
			spin_lock(&gti->fpga->slock);
			write_register(0x1000, addr);
			read_register(addr);
			spin_unlock(&gti->fpga->slock);
			mdelay(5);
			spin_lock(&gti->fpga->slock);
			write_register(0x0, addr);
			read_register(addr);
			spin_unlock(&gti->fpga->slock);
			mdelay(200);
			spin_lock(&gti->fpga->slock);
			addr=gti->fpga->control_bar + GYR_TX_FIFO_FULLNESS_OFFSET(gti->channel);
			write_register(0x0, addr);
			read_register(addr);
			addr=gti->fpga->control_bar + GYR_RX_FIFO_FULLNESS_OFFSET(gti->channel);
			write_register(0x1000, addr);
			read_register(addr);
			spin_unlock(&gti->fpga->slock);
			gti->mode=GTI_CMD_IDLE;
		}
		break;

	case ISET_GTI_CMD:
		{
			if (copy_from_user(&fip_cmd, (int *)arg, sizeof(int))) 	rc = -EFAULT;
			else{gti->mode=fip_cmd;		gti_debug("ISET_GTI_CMD: cmd = %d\n",fip_cmd);}
		}
		break;

	default:
		printk(KERN_ERR "Invalid command %d\n", cmd);
		rc = -EFAULT;
		break;
	}

	return rc;

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

//static irqreturn_t gti_isr_bh(int irq, void *dev_id){return IRQ_HANDLED;}

static irqreturn_t gti_isr_th(int irq, void *dev_id)
{
	u32 irq_status,status, user_irq_status, i; 
    const u32 control=XDMA_CTRL_RUN_STOP;
	struct fpga_dev *fpga=dev_id;
	struct interrupt_regs *irq_regs= (struct interrupt_regs *)(fpga->dma_bar + REG_OFFSET_INTERRUPT);
	struct engine_regs * h2c_reg= (struct engine_regs *)(fpga->dma_bar + REG_OFFSET_ENGINE);
	struct engine_regs * c2h_reg =(struct engine_regs *)((char *)h2c_reg + REG_C2H_OFFSET);

	if(dev_id==0){	printk(KERN_ERR "got wrong irq:%d\n",irq);	return IRQ_HANDLED;	}

#if 1
	user_irq_status = read_register(&irq_regs->user_int_request);
	
	if(user_irq_status != 0){
		read_register(&irq_regs->user_int_pending);
		for(i=0;i<4;i++){
			if (user_irq_status&(0x10 << i)){
				write_register((0x10 << i), &irq_regs->user_int_enable_w1c);
				fpga->user_q_state[i]=1;
				up(&fpga->user_wq_sema[i]);
			}		
		}
		wake_up_interruptible(&fpga->wq);
	}
#endif

   	//spin_lock(&fpga->slock);
	irq_status = read_register(&irq_regs->channel_int_request);
   	//spin_unlock(&fpga->slock);
    if(irq_status != 0){
        fpga->last_irq=irq_status;
   	    //spin_lock(&fpga->slock);
	    write_register(fpga->last_irq, &irq_regs->channel_int_enable_w1c);
	    read_register(&irq_regs->channel_int_pending);
	    gti_debug("(irq=%d),dev_id(%px),request(%d)\n", irq,dev_id,fpga->last_irq);
	    if(fpga->last_irq &0x55){
		    status = read_register(&h2c_reg->status);
		    write_register(0xffffffff,&h2c_reg->status);
		    write_register(control, &h2c_reg->control_w1c);
		    gti_debug("h2c bh:%08x,%08x\n",status,read_register(&h2c_reg->completed_desc_count));
	    }
	    if(fpga->last_irq &0xaa){
	    	status = read_register(&c2h_reg->status);
		    write_register(0xffffffff,&c2h_reg->status);
    		write_register(control, &c2h_reg->control_w1c);
	    	gti_debug("c2h bh:%08x,%08x\n",status,read_register(&c2h_reg->completed_desc_count));
    	}
    	fpga->state=1;
    	//spin_unlock(&fpga->slock);
    	up(&fpga->sema);
	    wake_up_interruptible(&fpga->wq);
    }else{
	   	gti_debug("got unknown irq\n");
    }
	return IRQ_HANDLED;
}

int gti2803_dma_remove(void *gti_dev){
	int i;
	GTI_DEV *pdev=(GTI_DEV *)gti_dev;
	struct fpga_dev * fpga=(struct fpga_dev *)pdev->private;

	free_irq(pdev->pDev->irq,fpga);
	pci_iounmap(pdev->pDev,fpga->dma_bar);
	pci_iounmap(pdev->pDev,fpga->control_bar);

	device_remove_file(&pdev->pDev->dev, &dev_attr_gti_info);
	device_remove_file(&pdev->pDev->dev, &dev_attr_pll_info);
	for(i=0;i<GTI_CHANNEL_NUMBER;i++){
		cdev_del(&fpga->gti[i].chdev);
		device_destroy(pdev->pClass,fpga->gti[i].device->devt);
	}
	unregister_chrdev_region(fpga->cdevno_base,GTI_CHANNEL_NUMBER);
	dma_free_coherent(&pdev->pDev->dev,fpga->buffer_len*GTI_CHANNEL_NUMBER+2*PAGE_SIZE,fpga->buffer,fpga->dma_handle);
	kfree(fpga);

	return 0;
}

int gti2803_dma_init(void *gti_dev){
	int rc = 0;
	GTI_DEV *pdev=(GTI_DEV *)gti_dev;
	struct fpga_dev * fpga;
    int i;

	fpga = kzalloc(sizeof(struct fpga_dev), GFP_KERNEL); if(!fpga) return -ENOMEM;
    pdev->private=fpga;
    rc = pci_enable_msi(pdev->pDev);
    if(rc==0) rc = request_threaded_irq(pdev->pDev->irq, gti_isr_th,0, 0, "GTI2803_drv", fpga);
    if(rc==0) rc = device_create_file(&pdev->pDev->dev, &dev_attr_gti_info);
    if(rc==0) rc = device_create_file(&pdev->pDev->dev, &dev_attr_pll_info);
    if(rc!=0){
        kfree(fpga);
		printk(KERN_ERR "gti device init error, rc:%d\n",rc);
        return -ENOMEM;
    }

	if(!dma_set_mask_and_coherent(&pdev->pDev->dev,DMA_BIT_MASK(64))){
		printk(KERN_NOTICE "64bit DMA is supported\n");
	}else if (!dma_set_mask_and_coherent(&pdev->pDev->dev,DMA_BIT_MASK(32))){
		printk(KERN_NOTICE "32bit DMA is supported\n");
	}else{
		printk(KERN_NOTICE "No DMA is supported,exit\n");
		return -ENOMEM;
	}

	fpga->buffer_len=DMA_BUFFER_SIZE;
	fpga->buffer=dma_alloc_coherent(&pdev->pDev->dev,fpga->buffer_len*GTI_CHANNEL_NUMBER+2*PAGE_SIZE,&fpga->dma_handle,GFP_KERNEL|GFP_DMA);
	if(fpga->buffer==0){kfree(fpga); return -ENOMEM;}
	printk(KERN_NOTICE "dma buffer:%px:%llx\n",fpga->buffer,virt_to_phys(fpga->buffer));
	fpga->tx_descriptor =fpga->buffer + fpga->buffer_len*GTI_CHANNEL_NUMBER;
	fpga->rx_descriptor =fpga->tx_descriptor + PAGE_SIZE;

	fpga->dev=pdev;
	fpga->state=0;
    fpga->control_bar=pci_iomap(pdev->pDev, 0, pci_resource_len(pdev->pDev, 0));
    fpga->dma_bar=pci_iomap(pdev->pDev, 1, pci_resource_len(pdev->pDev, 1));

	mutex_init(&fpga->mlock);
	spin_lock_init(&fpga->slock);
    sema_init(&fpga->sema,1);

    	sema_init(&fpga->user_wq_sema[0],1);
    	sema_init(&fpga->user_wq_sema[1],1);
    	sema_init(&fpga->user_wq_sema[2],1);
    	sema_init(&fpga->user_wq_sema[3],1);

	init_waitqueue_head(&fpga->wq);

	rc = alloc_chrdev_region(&fpga->cdevno_base, 0,GTI_CHANNEL_NUMBER, "GTI2803_drv");

	if(rc==0){
		for(i=0;i<GTI_CHANNEL_NUMBER;i++){
			cdev_init(&fpga->gti[i].chdev, &gti_fops);
			fpga->gti[i].chdev.owner=THIS_MODULE;
			rc=  cdev_add(&fpga->gti[i].chdev, MKDEV(MAJOR(fpga->cdevno_base),i), 1); if(rc!=0) break;
			fpga->gti[i].device=device_create(pdev->pClass,&pdev->pDev->dev,MKDEV(MAJOR(fpga->cdevno_base),i),0,"gti2803-%d",i+GTI_CHANNEL_NUMBER*pdev->instance);
			if(fpga->gti[i].device==0){rc=-ENOMEM; break;}
			fpga->gti[i].fpga=fpga;
			fpga->gti[i].buffer=fpga->buffer+i*fpga->buffer_len;
			printk(KERN_NOTICE "channel dma buffer:%px:%llx\n",fpga->gti[i].buffer,virt_to_phys(fpga->gti[i].buffer));
			fpga->gti[i].channel=i;
		}
	}

    if(rc==0){
		u32 w;
		struct engine_regs * h2c_reg= (struct engine_regs *)(fpga->dma_bar + REG_OFFSET_ENGINE);
		struct engine_regs * c2h_reg =(struct engine_regs *)((char *)h2c_reg + REG_C2H_OFFSET);
		struct engine_sgdma_regs * h2c_sgdma_reg= (struct engine_sgdma_regs *)(fpga->dma_bar + REG_OFFSET_SGDMA);
		struct engine_sgdma_regs * c2h_sgdma_reg= (struct engine_sgdma_regs *)((char *)h2c_sgdma_reg + REG_C2H_OFFSET);
		struct interrupt_regs * irq_regs = (struct interrupt_regs *)(fpga->dma_bar + REG_OFFSET_INTERRUPT);

		u32 reg_value=XDMA_CTRL_IE_DESC_ALIGN_MISMATCH | XDMA_CTRL_IE_MAGIC_STOPPED |
					XDMA_CTRL_IE_INVALID_LENGTH | XDMA_CTRL_IE_READ_ERROR | XDMA_CTRL_IE_DESC_ERROR |
					XDMA_CTRL_IE_DESC_STOPPED   | XDMA_CTRL_IE_DESC_COMPLETED;

		write_register(reg_value, &h2c_reg->interrupt_enable_mask);
		write_register(reg_value, &c2h_reg->interrupt_enable_mask);

		write_register(0xffffffff, &h2c_reg->control_w1c);
		write_register(0xffffffff, &c2h_reg->control_w1c);
		write_register(0xffffffff, &irq_regs->channel_int_enable_w1c);
		write_register(0xffffffff, &irq_regs->user_int_enable_w1c);


		w = cpu_to_le32(PCI_DMA_L(virt_to_phys(fpga->tx_descriptor)));
		write_register(w, &h2c_sgdma_reg->first_desc_lo);
		w = cpu_to_le32(PCI_DMA_H(virt_to_phys(fpga->tx_descriptor)));
		write_register(w, &h2c_sgdma_reg->first_desc_hi);
		write_register(0, &h2c_sgdma_reg->first_desc_adjacent);

		w = cpu_to_le32(PCI_DMA_L(virt_to_phys(fpga->rx_descriptor)));
		write_register(w, &c2h_sgdma_reg->first_desc_lo);
		w = cpu_to_le32(PCI_DMA_H(virt_to_phys(fpga->rx_descriptor)));
		write_register(w, &c2h_sgdma_reg->first_desc_hi);
		write_register(0, &c2h_sgdma_reg->first_desc_adjacent);

		mmiowb();
		{
                struct gti_switch_control_regs *switch_regs=(struct gti_switch_control_regs *)(fpga->control_bar+ REG_SWITCH_OFFSET);
		write_register(0x80000000, &switch_regs->casc);

		write_register(0x3, &switch_regs->sda0);
		write_register(0x80000000, &switch_regs->reformat0);
		write_register(0x1, &switch_regs->c2h0);

		write_register(0x6, &switch_regs->sda1);
		write_register(0x80000000, &switch_regs->reformat1);
		write_register(0x4, &switch_regs->c2h1);

		write_register(0x9, &switch_regs->sda2);
		write_register(0x80000000, &switch_regs->reformat2);
		write_register(0x7, &switch_regs->c2h2);

		write_register(0xc, &switch_regs->sda3);
		write_register(0x80000000, &switch_regs->reformat3);
		write_register(0xa, &switch_regs->c2h3);

		mmiowb();
		write_register(0x2, &switch_regs->command);
		}
        if(0)
	{
	        struct gti_control_regs *reg=(struct gti_control_regs*)fpga->control_bar;
	        write_register(0x2,&reg->pll_control);
        }
#if 1
    	struct gti_control_regs *reg=(struct gti_control_regs*)fpga->control_bar;
    	write_register(0xff8,&reg->rx_fifo_service_threshold);
    	//printk(KERN_NOTICE "rx_fifo_srv_thsd: 0x%x\n",read_register(&reg->rx_fifo_service_threshold));
#endif
	}
	return rc;
}
