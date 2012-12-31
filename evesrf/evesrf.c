/*
 * evesrf.c
 * 
 * Copyright 2012 Jon Cross <joncross.cooljc@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <asm/uaccess.h>

#define USER_BUFF_SIZE	128

#define SPI_BUS 0
#define SPI_BUS_CS0 0
#define SPI_BUS_SPEED 1000000

#define EVESRF_DRIVER_NAME "evesrf"

/* ------------------------------------------------------------------ */
/* Debug MACRO used to print data to syslog. */
/* ------------------------------------------------------------------ */
#define dprintk(fmt, args...)					\
  do {								\
    if (debug)							\
      printk(KERN_INFO EVESRF_DRIVER_NAME ": "			\
	     fmt, ## args);					\
  } while (0)

/* ------------------------------------------------------------------ */
/* module parameters passed in via modprobe or insmod when module is
 * loaded */
/* ------------------------------------------------------------------ */
/* enable debugging messages */
static int debug = 1;


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
const char this_driver_name[] = "evesrf";

struct evesrf_dev_s {
  struct semaphore spi_sem;
  struct semaphore fop_sem;
  dev_t devt;
  struct cdev cdev;
  struct class *class;
  struct spi_device *spi_device;
  char *user_buff;
};

static struct evesrf_dev_s evesrf_dev;


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static ssize_t evesrf_read(struct file *filp, char __user *buff, 
			   size_t count, loff_t *offp)
{
  size_t len;
  ssize_t status = 0;

  if (!buff) 
    return -EFAULT;

  if (*offp > 0) 
    return 0;

  if (down_interruptible(&evesrf_dev.fop_sem)) 
    return -ERESTARTSYS;

  if (!evesrf_dev.spi_device)
    strcpy(evesrf_dev.user_buff, "spi_device is NULL\n");
  else if (!evesrf_dev.spi_device->master)
    strcpy(evesrf_dev.user_buff, "spi_device->master is NULL\n");
  else
    sprintf(evesrf_dev.user_buff, "%s ready on SPI%d.%d\n",
	    this_driver_name,
	    evesrf_dev.spi_device->master->bus_num,
	    evesrf_dev.spi_device->chip_select);


  len = strlen(evesrf_dev.user_buff);
 
  if (len < count) 
    count = len;

  if (copy_to_user(buff, evesrf_dev.user_buff, count))  {
    dprintk ("evesrf_read(): copy_to_user() failed\n");
    status = -EFAULT;
  } else {
    *offp += count;
    status = count;
  }

  up(&evesrf_dev.fop_sem);

  return status;	
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int evesrf_open(struct inode *inode, struct file *filp)
{	
  int status = 0;

  if (down_interruptible(&evesrf_dev.fop_sem)) 
    return -ERESTARTSYS;

  if (!evesrf_dev.user_buff) {
    evesrf_dev.user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);
    if (!evesrf_dev.user_buff) 
      status = -ENOMEM;
  }	

  up(&evesrf_dev.fop_sem);

  return status;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int evesrf_probe(struct spi_device *spi_device)
{
  if (down_interruptible(&evesrf_dev.spi_sem))
    return -EBUSY;

  dprintk ("%s()\n", __FUNCTION__);
  evesrf_dev.spi_device = spi_device;

  up(&evesrf_dev.spi_sem);

  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int evesrf_remove(struct spi_device *spi_device)
{
  if (down_interruptible(&evesrf_dev.spi_sem))
    return -EBUSY;

  evesrf_dev.spi_device = NULL;

  up(&evesrf_dev.spi_sem);

  return 0;
}


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init evesrf_add_device_to_bus(void)
{
  struct spi_master *spi_master;
  struct spi_device *spi_device;
  struct device *pdev;
  char buff[64];
  int status = 0;

  spi_master = spi_busnum_to_master (SPI_BUS);
  if (!spi_master) {
    dprintk ("spi_busnum_to_master(%d) returned NULL\n",
	     SPI_BUS);
    return -1;
  }

  spi_device = spi_alloc_device (spi_master);
  if (!spi_device) {
    put_device (&spi_master->dev);
    dprintk ("spi_alloc_device() failed\n");
    return -1;
  }

  spi_device->chip_select = SPI_BUS_CS0;

  /* Check whether this SPI bus.cs is already claimed */
  snprintf(buff, sizeof(buff), "%s.%u", 
	   dev_name(&spi_device->master->dev),
	   spi_device->chip_select);
  dprintk ("buff = %s\n", buff);

  pdev = bus_find_device_by_name (spi_device->dev.bus, NULL, buff);
  if (pdev) {
    /* We are not going to use this spi_device, so free it */ 
    spi_dev_put (spi_device);
    dprintk ("pdev = true\n");
    /* 
     * There is already a device configured for this bus.cs  
     * It is okay if it us, otherwise complain and fail.
     */
    if (pdev->driver && pdev->driver->name && 
	strcmp(this_driver_name, pdev->driver->name)) {
      dprintk ("Driver [%s] already registered for %s\n",
	       pdev->driver->name, buff);
      status = -1;
    } 
  }
  else {
    spi_device->max_speed_hz = SPI_BUS_SPEED;
    spi_device->mode = SPI_MODE_0;
    spi_device->bits_per_word = 8;
    spi_device->irq = -1;
    spi_device->controller_state = NULL;
    spi_device->controller_data = NULL;
    strlcpy (spi_device->modalias, this_driver_name, SPI_NAME_SIZE);

    status = spi_add_device (spi_device);		
    if (status < 0) {	
      spi_dev_put(spi_device);
      printk(KERN_ALERT "spi_add_device() failed: %d\n", 
	     status);		
    }
  }	
  

  put_device (&spi_master->dev);

  return status;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static struct spi_driver evesrf_driver = {
  .driver = {
    .name  = this_driver_name,
    .owner = THIS_MODULE,
  },
  .probe  = evesrf_probe,
  .remove = __devexit_p(evesrf_remove),
};

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init evesrf_init_spi(void)
{
  int error;

  error = spi_register_driver(&evesrf_driver);
  if (error < 0) {
    dprintk("spi_register_driver() failed %d\n", error);
    return error;
  }

  error = evesrf_add_device_to_bus();
  if (error < 0) {
    dprintk("evesrf_add_to_bus() failed\n");
    spi_unregister_driver(&evesrf_driver);
    return error;
  }

  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static const struct file_operations evesrf_fops = {
  .owner = THIS_MODULE,
  .read  = evesrf_read,
  .open  = evesrf_open,	
};

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init evesrf_init_cdev(void)
{
  int error = 0;
  evesrf_dev.devt = MKDEV(0, 0);

  error = alloc_chrdev_region(&evesrf_dev.devt, 0, 1, this_driver_name);
  if (error < 0) {
    dprintk("alloc_chrdev_region() failed: %d \n", 
	    error);
    return -1;
  }

  cdev_init(&evesrf_dev.cdev, &evesrf_fops);
  evesrf_dev.cdev.owner = THIS_MODULE;

  error = cdev_add(&evesrf_dev.cdev, evesrf_dev.devt, 1);
  if (error) {
    dprintk("cdev_add() failed: %d\n", error);
    unregister_chrdev_region(evesrf_dev.devt, 1);
    return -1;
  }	

  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init evesrf_init_class(void)
{
  evesrf_dev.class = class_create(THIS_MODULE, this_driver_name);

  if (!evesrf_dev.class) {
    dprintk("class_create() failed\n");
    return -1;
  }

  if (!device_create(evesrf_dev.class, NULL, evesrf_dev.devt, NULL, 	
		     this_driver_name)) {
    dprintk("device_create(..., %s) failed\n",
	    this_driver_name);
    class_destroy(evesrf_dev.class);
    return -1;
  }

  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init evesrf_init(void)
{
  memset(&evesrf_dev, 0, sizeof(evesrf_dev));

  sema_init(&evesrf_dev.spi_sem, 1);
  sema_init(&evesrf_dev.fop_sem, 1);

  if (evesrf_init_cdev() < 0) 
    goto fail_1;

  if (evesrf_init_class() < 0)  
    goto fail_2;

  if (evesrf_init_spi() < 0) 
    goto fail_3;

  return 0;

 fail_3:
  device_destroy(evesrf_dev.class, evesrf_dev.devt);
  class_destroy(evesrf_dev.class);

 fail_2:
  cdev_del(&evesrf_dev.cdev);
  unregister_chrdev_region(evesrf_dev.devt, 1);

 fail_1:
  return -1;
}
module_init(evesrf_init)

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void __exit evesrf_exit(void)
{
  spi_unregister_device(evesrf_dev.spi_device);
  spi_unregister_driver(&evesrf_driver);

  device_destroy(evesrf_dev.class, evesrf_dev.devt);
  class_destroy(evesrf_dev.class);

  cdev_del(&evesrf_dev.cdev);
  unregister_chrdev_region(evesrf_dev.devt, 1);

  if (evesrf_dev.user_buff)
    kfree(evesrf_dev.user_buff);
}
module_exit(evesrf_exit);

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
MODULE_DESCRIPTION("EVE Alpha SRF driver for Raspberry Pi.");
MODULE_AUTHOR("Jon Cross <joncross.cooljc@gmail.com>");
MODULE_LICENSE("GPL");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
