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
#include <linux/slab.h>

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

static struct tty_driver *ttysrf_driver = NULL;
static struct ttysrf_serial *ttysrf_saved = NULL;

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_open(struct tty_struct *tty, struct file *file)
{
  if (!ttysrf_saved)
    return -ENOMEM;

  down (&ttysrf_saved->sem);

  /* save our structure within the tty structure */
  tty->driver_data = ttysrf_saved;
  ttysrf_saved->tty = tty;
  ++ttysrf_saved->open_count;
  if (ttysrf_saved->open_count == 1) {
    /* this is the first time this port is opened */
    /* do any hardware initialization needed here */
  }
  up (&ttysrf_saved->sem);

  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_close(struct tty_struct *tty, struct file *file)
{
  struct ttysrf_serial *ttysrf = tty->driver_data;
  down (&ttysrf->sem);
  if (!ttysrf->open_count) {
    /* port was never opened */
    goto exit;
  }

  --ttysrf->open_count;
  if (ttysrf->open_count <= 0) {
    /* The port is being closed by the last user. */
    /* Do any hardware specific stuff here */
  }

 exit:
  up (&ttysrf->sem);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_write(struct tty_struct *tty, 
			const unsigned char *buffer, int count)
{
  struct ttysrf_serial *ttysrf = tty->driver_data;

  down (&ttysrf->sem);

  /* echo data back */
  tty_insert_flip_string(tty, buffer, count);
  tty_flip_buffer_push(tty);

  up (&ttysrf->sem);
  return count;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_write_room(struct tty_struct *tty) 
{
  struct ttysrf_serial *ttysrf = tty->driver_data;
  int room = -EINVAL;

  if (!ttysrf)
    return -ENODEV;

  down(&ttysrf->sem);

  if (!ttysrf->open_count) {
    /* port was not opened */
    goto exit;
  }

  /* calculate how much room is left in the device */
  room = 255;

 exit:
  up(&ttysrf->sem);
  return room;
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
static int __init ttysrf_init_tty (struct ttysrf_serial *ttysrf)
{
  int ret = 0;

  /* allocate memory for tty driver. */
  //ttysrf_driver = tty_alloc_driver (TTYSRF_MINORS, TTY_DRIVER_UNNUMBERED_NODE);
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
  ttysrf_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
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
  ttysrf->tty_dev = tty_register_device(ttysrf_driver, 0, NULL);

  return ret;
}

/* ------------------------------------------------------------------ */
/* ttysrf_init()
 * This function is called when the kernel module is loaded. */
/* ------------------------------------------------------------------ */
static int __init ttysrf_init (void)
{
  int ret = 0;
  struct ttysrf_serial *ttysrf = NULL;

  dprintk ("%s()\n", __func__);

  /* allocate memory for ttysrf structure */
  ttysrf = kmalloc(sizeof(*ttysrf), GFP_KERNEL);
  if (!ttysrf)
    return -ENOMEM;
  sema_init(&ttysrf->sem, 1);
  ttysrf->open_count = 0;

  ret = ttysrf_init_tty (ttysrf);
  if (ret < 0)
    goto error;

  ttysrf_saved = ttysrf;
  return ret;
 error:
  /* free structure */
  kfree(ttysrf);
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
   kfree(ttysrf_saved);
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
