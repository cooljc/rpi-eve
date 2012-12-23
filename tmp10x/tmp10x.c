/*
 * tmp10x.c
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
#include <linux/module.h>
#include <linux/init.h>

#include <linux/sched.h>  /* current and everything */
#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>     /* everything... */
#include <linux/types.h>  /* size_t */
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/i2c.h>

#include "tmp10x.h"

#define TMP10X_DRIVER_NAME "tmp10x"

/*
 * From userlan execute the following:
 * modprobe i2c-dev
 * modprobe tmp10x (or insmod if not in /lib/modules/...)
 * echo tmp10x 0x48 > /sys/class/i2c-adapter/i2c-0/new_device
 */

/* ------------------------------------------------------------------ */
/* Debug MACRO used to print data to syslog. */
/* ------------------------------------------------------------------ */
#define dprintk(fmt, args...)					\
  do {								\
    if (debug)							\
      printk(KERN_INFO TMP10X_DRIVER_NAME ": "			\
	     fmt, ## args);					\
  } while (0)


/* ------------------------------------------------------------------ */
/* module parameters passed in via modprobe or insmod when module is
 * loaded */
/* ------------------------------------------------------------------ */
/* enable debugging messages */
static int debug = 0;

/* ------------------------------------------------------------------ */
/* Global variables required for driver operation. */
/* ------------------------------------------------------------------ */
static int tmp10x_major = 0;
static unsigned long driver_open;
static struct class *tmp10x_class = 0;

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
struct i2c_client *tmp10x_i2c_client = NULL;

/* ------------------------------------------------------------------ */
/* tmp10x_i2c_read()
 * This function reads bytes from TMP10X device. */
/* ------------------------------------------------------------------ */
static int tmp10x_i2c_read (void *data, uint8_t subaddr, uint8_t len)
{
  struct i2c_msg msgs[] = {
    {
      .addr = tmp10x_i2c_client->addr,
      .flags = 0,
      .len = 1,
      .buf = &subaddr,
    }, {
      .addr = tmp10x_i2c_client->addr,
      .flags = I2C_M_RD,
      .len = len,
      .buf = data,
    }
  };

  if (i2c_transfer(tmp10x_i2c_client->adapter, msgs, 2) == 2)
    return 0;

  return -EIO;
}

/* ------------------------------------------------------------------ */
/* tmp10x_i2c_write()
 * This function writes data to TMP10X device */
/* ------------------------------------------------------------------ */
static int tmp10x_i2c_write(void *data, uint8_t subaddr, uint8_t len)
{
  uint8_t buffer[len + 1];

  buffer[0] = subaddr;
  memcpy(&buffer[1], data, len);

  if (i2c_master_send(tmp10x_i2c_client, buffer, len + 1) == len + 1)
    return 0;

  return -EIO;
}

/* ------------------------------------------------------------------ */
/* tmp10x_open()
 * This function is called form userland using open(). If successful
 * it will return a file descriptor that can be used with ioctl() and
 * close() system functions. */
/* ------------------------------------------------------------------ */
static int tmp10x_open(struct inode *inode, struct file *file)
{
  if (test_and_set_bit(0, &driver_open))
    return -EBUSY;
  
  return nonseekable_open(inode, file);
}

/* ------------------------------------------------------------------ */
/* tmp10x_release()
 * This function is called from userland using close() on an open
 * file descriptor. */
/* ------------------------------------------------------------------ */
static int tmp10x_release(struct inode *inode, struct file *file)
{
  clear_bit(0, &driver_open);
  return 0;
}

/* ------------------------------------------------------------------ */
/* tmp10x_ioctl()
 * This function is called from userland using ioctl() on an open
 * file descriptor to /dev/tmp10x. */
/* ------------------------------------------------------------------ */
static long tmp10x_ioctl (struct file *file, unsigned int cmd, 
			  unsigned long arg)
{
  void __user *argp = (void __user *)arg;
  long ret = -ENOTTY; /* unknown command */
  tmp10x_temp_t temp;

  /* check command magic code matches */
  if(_IOC_TYPE(cmd) != TMP10X_IOC_MAGIC) {
    return ret;
  }

  switch (cmd) {
  case TMP10X_IOC_GETTEMP:
    {
      uint8_t buf[2];
      tmp10x_i2c_read(buf, 0, 2); 
      /* add some dummy data to test interface */
      temp.m_temp = ((buf[0] << 4) | ((buf[1] >> 4) & 0x0f));
      ret = copy_to_user(argp, &temp, sizeof(temp)) ? -EFAULT : 0;
    }
    break;
  case TMP10X_IOC_GETCONFIG:
    {
      uint8_t buf[1];
      tmp10x_config_t config;
      tmp10x_i2c_read(buf, 1, 1); 
      config.m_sd  = (buf[0] & 0x01);
      config.m_tm  = ((buf[0] >> 1) & 0x01);
      config.m_pol = ((buf[0] >> 2) & 0x01);
      config.m_fq  = ((buf[0] >> 3) & 0x03);
      config.m_cr  = ((buf[0] >> 5) & 0x03);
      config.m_os  = ((buf[0] >> 7) & 0x01);
      ret = copy_to_user(argp, &config, sizeof(config)) ? -EFAULT : 0;
    }
    break;
  case TMP10X_IOC_SETCONFIG:
    {
      tmp10x_config_t config;
      ret = copy_from_user(&config, argp, sizeof(config)) ? -EFAULT : 0;
      if (ret == 0) {
	uint8_t buf[1];
	buf[0] = (config.m_os << 7); 
	buf[0] |= (config.m_cr << 5); 
	buf[0] |= (config.m_fq << 3); 
	buf[0] |= (config.m_pol << 2);
	buf[0] |= (config.m_tm << 1);
	buf[0] |= config.m_sd;
	tmp10x_i2c_write (buf, 1, 1);
      }
    }
    break;
  }

  return ret;
}

/* ------------------------------------------------------------------ */
/* Define device file operations. */
/* ------------------------------------------------------------------ */
struct file_operations tmp10x_fops = {
  .owner          = THIS_MODULE,
  .open           = tmp10x_open,
  .release        = tmp10x_release,
  .unlocked_ioctl = tmp10x_ioctl,
};

/* ------------------------------------------------------------------ */
/* tmp10x_remove()
 * This function is called when the module is removed or init fails.  */
/* ------------------------------------------------------------------ */
static int __devexit tmp10x_remove(struct i2c_client *client)
{
  tmp10x_i2c_client = NULL;

  /* remove /dev/tmp10x */
  if (tmp10x_class) {
    device_destroy(tmp10x_class, MKDEV(tmp10x_major, 0));
    class_destroy(tmp10x_class);
  }

  /* unregister char device */
  unregister_chrdev(tmp10x_major, TMP10X_DRIVER_NAME);

  return 0;
}

/* ------------------------------------------------------------------ */
/* tmp10x_probe()
 * This function is called when the module is inserted/loaded. */
/* ------------------------------------------------------------------ */
static int tmp10x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
  int result;
  /*
   * Register your major, and accept a dynamic number
   */
  result = register_chrdev(tmp10x_major, TMP10X_DRIVER_NAME, &tmp10x_fops);
  if (result < 0)
    return result;
  if (tmp10x_major == 0)
    tmp10x_major = result; /* dynamic */

  tmp10x_class = class_create(THIS_MODULE, TMP10X_DRIVER_NAME);
  if (IS_ERR(tmp10x_class)) {
    result = PTR_ERR(tmp10x_class);
    goto fail;
  }

  /* create device in /dev/ directory */
  device_create(tmp10x_class, NULL, MKDEV(tmp10x_major, 0), NULL, "%s", TMP10X_DRIVER_NAME);
  dprintk ("device created: /dev/tmp10x\n");

  /* copy address of i2c client to global variable.. */
  tmp10x_i2c_client = client;

  return 0;

 fail:
  tmp10x_remove (client);
  return result;
}

static const struct i2c_device_id tmp10x_id[] = {
  { "tmp10x", 0 },
  { }
};
MODULE_DEVICE_TABLE(i2c, tmp10x_id);

static struct i2c_driver tmp10x_driver = {
  .driver = {
    .name    = TMP10X_DRIVER_NAME,
    .owner   = THIS_MODULE,
  },
  .probe     = tmp10x_probe,
  .remove    = __devexit_p(tmp10x_remove),
  .id_table  = tmp10x_id,
};

/* ------------------------------------------------------------------ */
/* register module load/exit functions */
/* ------------------------------------------------------------------ */
static __init int tmp10x_init(void)
{
  return i2c_add_driver(&tmp10x_driver);
}
module_init(tmp10x_init);

static __exit void tmp10x_exit(void)
{
  i2c_del_driver(&tmp10x_driver);
}
module_exit(tmp10x_exit);


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
MODULE_DESCRIPTION("TI-TMP101/101 driver for Raspberry Pi.");
MODULE_AUTHOR("Jon Cross <joncross.cooljc@gmail.com>");
MODULE_LICENSE("GPL");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

/* EOF */
