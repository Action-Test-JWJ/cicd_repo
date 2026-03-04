/***********************************************************************
*
* Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
* See LICENSE file in the project root for full license information.
*
************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/printk.h>

#include "gti_drv.h"

#define DRV_NAME "GTI_PCIE_DRIVER"
#define DRV_MODULE_VERSION "1.0"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(	"Gyrfalcon Technology" );
MODULE_VERSION(DRV_MODULE_VERSION);

static struct class *gti_class=0;
static int gInstance=0;
static DEFINE_MUTEX(gti_mutex);

static const struct pci_device_id pci_ids[] = {
        { PCI_DEVICE(0x10ee, 0x7014), },  //default xilinx ID
        { PCI_DEVICE(0x10ee, 0x7022), },  //default xilinx ID
        { PCI_DEVICE(0x10ee, 0x7024), },  //default xilinx ID
        { PCI_DEVICE(0x10ee, 0x7122), },
        { PCI_DEVICE(0x1e00, 0x2803), },
        {0,}
};
MODULE_DEVICE_TABLE(pci, pci_ids);



static void remove(struct pci_dev *pdev){
	GTI_DEV * pDev=pci_get_drvdata(pdev);
	if(pDev->exitFn) pDev->exitFn(pDev);
	pci_release_regions(pdev);
	pci_disable_msi(pdev);
	pci_disable_device(pdev);

}

static int probe(struct pci_dev *pdev, const struct pci_device_id *id){
	int rc = 0;
	//int i;
	GTI_DEV * gti_dev;

    gti_dev = kzalloc(sizeof(GTI_DEV), GFP_KERNEL);
    if(!gti_dev) return -ENOMEM;
    pci_set_drvdata(pdev, gti_dev);
    gti_dev->pDev=pdev;
    gti_dev->id=*id;
    gti_dev->pClass=gti_class;
    mutex_lock(&gti_mutex);
    gti_dev->instance=gInstance++;
    mutex_unlock(&gti_mutex);

    rc = pci_enable_device(pdev);

    pci_set_master(pdev);
    //if(rc==0) rc = pci_enable_msi(pdev);
    if(rc==0) rc = pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
    if(rc==0) rc = pci_request_regions(pdev, DRV_NAME);

    if(rc==0){
    	if((pdev->vendor==0x1e00)&&(pdev->device==0x2803)) {gti_dev->initFn=gti2803_init;gti_dev->exitFn=gti2803_remove;}
    	else if((pdev->subsystem_vendor==0x1e00)&&(pdev->subsystem_device==0x2801)) {gti_dev->initFn=gti2801_init;gti_dev->exitFn=gti2801_remove;}
    	else if((pdev->subsystem_vendor==0x1e00)&&(pdev->subsystem_device==0x2803)) {gti_dev->initFn=gti2803_dma_init;gti_dev->exitFn=gti2803_dma_remove;}
    	else if((pdev->subsystem_vendor==0x1e00)&&(pdev->subsystem_device==0x2803)) {gti_dev->initFn=gti2803_init;gti_dev->exitFn=gti2803_remove;}
    	else {gti_dev->initFn=gti2803_fpga_init;gti_dev->exitFn=gti2803_fpga_remove;}
    	rc=gti_dev->initFn(gti_dev);
    }

    if(rc!=0){
        remove(pdev);
        printk(KERN_ERR "Could not initialize PCI device, rc:%d\n",rc);
        return -ENOMEM;
    }


	return rc;
}

static struct pci_driver pci_driver = {
	.name = DRV_NAME,
	.id_table = pci_ids,
	.probe = probe,
	.remove = remove,
};

static int __init gti_pcie_init(void){
	printk(KERN_NOTICE DRV_NAME " v" DRV_MODULE_VERSION "\n");
	// gti_class = class_create(THIS_MODULE, DRV_NAME); if(gti_class==0) return -1;
    gti_class = class_create(DRV_NAME); if(gti_class==0) return -1;
	return pci_register_driver(&pci_driver);
}

static void __exit gti_pcie_exit(void){
	pci_unregister_driver(&pci_driver);
	class_destroy(gti_class);
}

module_init(gti_pcie_init);
module_exit(gti_pcie_exit);
  
