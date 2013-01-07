/*
 * ttysrf.c
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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>

#include "ttysrf.h"

/* ------------------------------------------------------------------ */
/* Debug MACRO used to print data to syslog. */
/* ------------------------------------------------------------------ */
#define dprintk(fmt, args...)					\
  do {								\
    if (debug)							\
      printk(KERN_INFO TTYSRF_DRIVER_NAME ": "			\
	     fmt, ## args);					\
  } while (0)

/* ------------------------------------------------------------------ */
/* module parameters passed in via modprobe or insmod when module is
 * loaded */
/* ------------------------------------------------------------------ */
/* enable debugging messages */
static int debug = 1;

static struct tty_driver *ttysrf_driver;

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_open(struct tty_struct *tty, struct file *file)
{
  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_close(struct tty_struct *tty, struct file *file)
{
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_write(struct tty_struct *tty, 
			const unsigned char *buffer, int count)
{
  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_write_room(struct tty_struct *tty) 
{
  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_set_termios(struct tty_struct *tty, 
			       struct ktermios *old_termios)
{
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static struct tty_operations ttysrf_serial_ops = {
  .open = ttysrf_open,
  .close = ttysrf_close,
  .write = ttysrf_write,
  .write_room = ttysrf_write_room,
  .set_termios = ttysrf_set_termios,
};

/* ------------------------------------------------------------------ */
/* ttysrf_init_tty()
 * This function is used to setup the tty device. */
/* ------------------------------------------------------------------ */
static int __init ttysrf_init_tty (void)
{
  int ret = 0;

  /* allocate memory for tty driver. */
  ttysrf_driver = alloc_tty_driver (TTYSRF_MINORS);
  if (!ttysrf_driver) {
    dprintk ("Failed to allocate memory for ttysrf driver!\n");
    return -ENOMEM;
  }

  /* initialize the tty driver */
  ttysrf_driver->owner = THIS_MODULE;
  ttysrf_driver->driver_name = TTYSRF_DRIVER_NAME;
  ttysrf_driver->name = "ttySRF";
  ttysrf_driver->minor_start = TTYSRF_MINORS;
  ttysrf_driver->num = TTYSRF_MINORS;
  ttysrf_driver->type = TTY_DRIVER_TYPE_SERIAL;
  ttysrf_driver->subtype = SERIAL_TYPE_NORMAL;
  ttysrf_driver->flags = TTY_DRIVER_REAL_RAW;
  ttysrf_driver->init_termios = tty_std_termios;
  tty_set_operations(ttysrf_driver, &ttysrf_serial_ops);

  /* register the tty driver */
  ret = tty_register_driver(ttysrf_driver);
  if (ret) {
    dprintk("failed to register ttysrf driver");
    put_tty_driver(ttysrf_driver);
    return ret;
  }

  /* register device */
  tty_register_device(ttysrf_driver, 0, NULL);

  return ret;
}

/* ------------------------------------------------------------------ */
/* ttysrf_init()
 * This function is called when the kernel module is loaded. */
/* ------------------------------------------------------------------ */
static int __init ttysrf_init (void)
{
  int ret = 0;

  dprintk ("%s()\n", __func__);

  ret = ttysrf_init_tty ();

  return ret;
}
module_init (ttysrf_init);

/* ------------------------------------------------------------------ */
/* ttysrf_exit()
 * This function is called when the kernel module is removed. */
/* ------------------------------------------------------------------ */
static void __exit ttysrf_exit (void)
{
   dprintk ("%s()\n", __func__);

   tty_unregister_device(ttysrf_driver, 0);
   tty_unregister_driver(ttysrf_driver);

}
module_exit (ttysrf_exit);

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
MODULE_DESCRIPTION("EVE Alpha SRF tty driver for Raspberry Pi.");
MODULE_AUTHOR("Jon Cross <joncross.cooljc@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
