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
#include <linux/spi/spi.h>

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

static struct tty_driver *ttysrf_tty_driver = NULL;
static struct ttysrf_serial *ttysrf_saved = NULL;

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_open(struct tty_struct *tty, struct file *file)
{
  struct ttysrf_serial *ttysrf = ttysrf_saved; //tty->driver_data;
  return tty_port_open(&ttysrf->tty_port, tty, file);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_close(struct tty_struct *tty, struct file *file)
{
  struct ttysrf_serial *ttysrf = tty->driver_data;
  tty_port_close(&ttysrf->tty_port, tty, file);
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
static int ttysrf_port_activate(struct tty_port *port, struct tty_struct *tty)
{
  struct ttysrf_serial *ttysrf =
    container_of(port, struct ttysrf_serial, tty_port);

  dprintk ("%s()\n", __func__);

  /* put port data into this tty */
  tty->driver_data = ttysrf;

  /* allows flip string push from int context */
  tty->low_latency = 1;

  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_port_shutdown(struct tty_port *port)
{
  struct ttysrf_serial *ttysrf =
    container_of(port, struct ttysrf_serial, tty_port);
  ttysrf = ttysrf; /* prevent warning */
  dprintk ("%s()\n", __func__);
  /* stop threads, interrupts etc.. */
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static const struct tty_port_operations ttysrf_tty_port_ops = {
  .activate = ttysrf_port_activate,
  .shutdown = ttysrf_port_shutdown,
};

static struct tty_operations ttysrf_serial_ops = {
  .open = ttysrf_open,
  .close = ttysrf_close,
  .write = ttysrf_write,
  .write_room = ttysrf_write_room,
  .set_termios = ttysrf_set_termios,
};

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_free_port(struct ttysrf_serial *ttysrf)
{
  if (ttysrf->tty_dev)
    tty_unregister_device(ttysrf_tty_driver, ttysrf->minor);
  kfree (ttysrf);
}


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_create_port(struct ttysrf_serial *ttysrf)
{
  int ret = 0;
  struct tty_port *pport = &ttysrf->tty_port;

  tty_port_init(pport);
  pport->ops = &ttysrf_tty_port_ops;
  ttysrf->minor = 0; //TTYSRF_SPI_TTY_ID;

  ttysrf->tty_dev = tty_register_device(ttysrf_tty_driver,
					ttysrf->minor,
					&ttysrf->spi_dev->dev);

  if (IS_ERR(ttysrf->tty_dev)) {
    dprintk("%s: registering tty device failed", __func__);
    ret = PTR_ERR(ttysrf->tty_dev);
    goto error_ret;
  }
  return 0;

 error_ret:
  ttysrf_free_port(ttysrf);
  return ret;
}


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_free_device(struct ttysrf_serial *ttysrf)
{
  ttysrf_free_port(ttysrf);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_spi_probe(struct spi_device *spi)
{
  struct ttysrf_serial *ttysrf = NULL;
  int ret = 0;

  dprintk ("%s()\n", __func__);

  /* allocate memory for ttysrf structure */
  ttysrf = kzalloc(sizeof(*ttysrf), GFP_KERNEL);
  if (!ttysrf)
    return -ENOMEM;
  sema_init(&ttysrf->sem, 1);
  ttysrf->open_count = 0;
  ttysrf->spi_dev = spi;

  /* TODO?: Do I need to create any buffer :s ?? */
  /* set driver user data */
  spi_set_drvdata(spi, ttysrf);

  /* create TTY port */
  ret = ttysrf_create_port(ttysrf);
  if (ret != 0) {
    dprintk ("create default tty port failed");
    goto probe_error_1;
  }

  /* TODO: setup GPIO and Interrupt */

  /* set global pointer */
  ttysrf_saved = ttysrf;

  return 0;

 probe_error_1:
  kfree(ttysrf);
  return ret;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_spi_remove(struct spi_device *spi)
{
  struct ttysrf_serial *ttysrf = spi_get_drvdata(spi);
  dprintk ("%s()\n", __func__);
  /* TODO: free GPIOs and Interrupts */

  /* release SPI device from BUS */
  //spi_unregister_device (spi);
  //spi_dev_put (spi);

  /* free allocations */
  ttysrf_free_device (ttysrf);
  ttysrf_saved = NULL;
  return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init ttysrf_add_spi_device_to_bus(void)
{
  struct spi_master *spi_master;
  struct spi_device *spi_device;
  struct device *pdev;
  char buff[64];
  int status = 0;

  spi_master = spi_busnum_to_master (TTYSRF_SPI_BUS);
  if (!spi_master) {
    dprintk ("spi_busnum_to_master(%d) returned NULL\n",
	     TTYSRF_SPI_BUS);
    return -1;
  }

  spi_device = spi_alloc_device (spi_master);
  if (!spi_device) {
    put_device (&spi_master->dev);
    dprintk ("spi_alloc_device() failed\n");
    return -1;
  }

  spi_device->chip_select = TTYSRF_SPI_BUS_CS0;

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
	strcmp(TTYSRF_DRIVER_NAME, pdev->driver->name)) {
      dprintk ("Driver [%s] already registered for %s\n",
	       pdev->driver->name, buff);
      status = -1;
    }
  }
  else {
    spi_device->max_speed_hz = TTYSRF_SPI_BUS_SPEED;
    spi_device->mode = SPI_MODE_0;
    spi_device->bits_per_word = 8;
    spi_device->irq = -1;
    spi_device->controller_state = NULL;
    spi_device->controller_data = NULL;
    strlcpy (spi_device->modalias, TTYSRF_DRIVER_NAME, SPI_NAME_SIZE);

    /* This will force the SPI probe callback */
    status = spi_add_device (spi_device);
    if (status < 0) {
      spi_dev_put(spi_device);
      dprintk ("spi_add_device() failed: %d\n",
	       status);
    }
  }

  put_device (&spi_master->dev);

  return status;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static struct spi_driver ttysrf_spi_driver = {
  .driver = {
    .name  = TTYSRF_DRIVER_NAME,
    .owner = THIS_MODULE,
  },
  .probe  = ttysrf_spi_probe,
  .remove = __devexit_p(ttysrf_spi_remove),
};

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init ttysrf_init_spi(void)
{
  int error;

  error = spi_register_driver(&ttysrf_spi_driver);
  if (error < 0) {
    dprintk("spi_register_driver() failed %d\n", error);
    return error;
  }

  /* in a normal world we would do this in the board_init for the chip
   * but because this is an experimental driver we force it here */
  error = ttysrf_add_spi_device_to_bus();
  if (error < 0) {
    dprintk("evesrf_add_to_bus() failed\n");
    spi_unregister_driver(&ttysrf_spi_driver);
    return error;
  }

  return 0;
}

/* ------------------------------------------------------------------ */
/* ttysrf_init_tty()
 * This function is used to setup the tty device. */
/* ------------------------------------------------------------------ */
static int __init ttysrf_init_tty (void)
{
  int ret = 0;

  /* allocate memory for tty driver. */
  //ttysrf_tty_driver = tty_alloc_driver (TTYSRF_MINORS, TTY_DRIVER_UNNUMBERED_NODE);
  ttysrf_tty_driver = alloc_tty_driver (TTYSRF_MINORS);
  if (!ttysrf_tty_driver) {
    dprintk ("Failed to allocate memory for ttysrf driver!\n");
    return -ENOMEM;
  }

  /* initialize the tty driver */
  ttysrf_tty_driver->owner = THIS_MODULE;
  ttysrf_tty_driver->driver_name = TTYSRF_DRIVER_NAME;
  ttysrf_tty_driver->name = "ttySRF";
  ttysrf_tty_driver->minor_start = TTYSRF_MINORS;
  ttysrf_tty_driver->num = TTYSRF_MINORS;
  ttysrf_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
  ttysrf_tty_driver->subtype = SERIAL_TYPE_NORMAL;
  ttysrf_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
  ttysrf_tty_driver->init_termios = tty_std_termios;
  tty_set_operations(ttysrf_tty_driver, &ttysrf_serial_ops);

  /* register the tty driver */
  ret = tty_register_driver(ttysrf_tty_driver);
  if (ret) {
    dprintk("failed to register ttysrf driver");
    put_tty_driver(ttysrf_tty_driver);
    return ret;
  }

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

  /* initialise TTY driver */
  ret = ttysrf_init_tty ();
  if (ret < 0)
    goto init_error_1;

  /* initialise SPI driver */
  ret = ttysrf_init_spi ();
  if (ret < 0)
    goto init_error_2;

  return ret;

 init_error_2:
  /* free tty driver */
  tty_unregister_driver(ttysrf_tty_driver);

 init_error_1:
  /* free structure */
  //kfree(ttysrf);
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

   /* unregister SPI driver */
   spi_unregister_driver(&ttysrf_spi_driver);
   /* unregister TTY device and driver */
   tty_unregister_device(ttysrf_tty_driver, 0);
   tty_unregister_driver(ttysrf_tty_driver);

   /* free ttysrf control structure */
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
