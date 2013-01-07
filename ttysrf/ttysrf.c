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


/* ------------------------------------------------------------------ */
/* ttysrf_init()
 * This function is called when the kernel module is loaded. */
/* ------------------------------------------------------------------ */
static int __init ttysrf_init (void)
{
  dprintk ("%s()\n", __func__);
  return 0;
}
module_init (ttysrf_init);

/* ------------------------------------------------------------------ */
/* ttysrf_exit()
 * This function is called when the kernel module is removed. */
/* ------------------------------------------------------------------ */
static void __exit ttysrf_exit (void)
{
   dprintk ("%s()\n", __func__);
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
