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
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "ttysrf.h"

/* ------------------------------------------------------------------ */
/* Debug MACRO used to print data to syslog. */
/* ------------------------------------------------------------------ */
#define dprintk(fmt, args...)					 \
	do {							 \
		if (debug)					 \
			printk(KERN_INFO TTYSRF_DRIVER_NAME ": " \
				fmt, ## args);			 \
	} while (0)

/* ------------------------------------------------------------------ */
/* module parameters passed in via modprobe or insmod when module is
 * loaded */
/* ------------------------------------------------------------------ */
/* enable debugging messages */
static int debug = 1;
/* set the default GPIO irq pin */
static int gpio_irq_pin = 25;

static struct tty_driver *ttysrf_tty_driver = NULL;
static struct ttysrf_serial *ttysrf_saved = NULL;
static struct lock_class_key ttysrf_spi_key;

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_spi_complete(void *arg)
{
	struct ttysrf_serial *ttysrf = arg;
	struct tty_struct *tty;
	unsigned char *rx_buffer;
	int loop = 0;

	down(&ttysrf->sem);

	tty = tty_port_tty_get(&ttysrf->tty_port);
	rx_buffer = ttysrf->rx_buffer;

	/* TODO: process data (fe ff) */
	for (loop = 0; loop < ttysrf->tx_len; loop++) {
		if (rx_buffer[loop] == 0xfe && ttysrf->fe_flag == 0) {
			ttysrf->fe_flag = 1;
		} else if (rx_buffer[loop] == 0xff && ttysrf->fe_flag == 0) {
			/* do nothing - no data */
		} else {
			tty_insert_flip_char(tty, rx_buffer[loop], TTY_NORMAL);
			ttysrf->fe_flag = 0;
		}
		/*    tty_insert_flip_string(tty, rx_buffer, temp_count); */
	}
	tty_flip_buffer_push(tty);
	tty_kref_put(tty);

	ttysrf->spi_busy = 0;
	up(&ttysrf->sem);

}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_spi_send_msg(struct ttysrf_serial *ttysrf)
{
	int status;

	spi_message_init(&ttysrf->spi_msg);

	/* this gets called when the spi_message completes */
	ttysrf->spi_msg.complete = ttysrf_spi_complete;
	ttysrf->spi_msg.context = ttysrf;

	/* setup transfer buffers and length */
	ttysrf->spi_xfer.tx_buf = ttysrf->tx_buffer;
	ttysrf->spi_xfer.rx_buf = ttysrf->rx_buffer;
	ttysrf->spi_xfer.len = ttysrf->tx_len;

	spi_message_add_tail(&ttysrf->spi_xfer, &ttysrf->spi_msg);

	/* TODO: spin lock?? */
	if (ttysrf->spi_dev)
		status = spi_async(ttysrf->spi_dev, &ttysrf->spi_msg);
	else
		status = -ENODEV;
	/* TODO: spin unlock ?? */

	if (status == 0) {
		down(&ttysrf->sem);
		ttysrf->spi_busy = 1;
		up(&ttysrf->sem);
	}

	return status;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_spi_thread(void *arg)
{
	struct ttysrf_serial *ttysrf = arg;
	int status = 0;
	int isBusy = 0;
	do {
		int tx_fifo_len = kfifo_len(&ttysrf->tx_fifo);
		int temp_count = 0;

		down(&ttysrf->sem);
		isBusy = ttysrf->spi_busy;
		up(&ttysrf->sem);

		if (isBusy == 0) {
			if (tx_fifo_len) {
				unsigned char *tx_buffer = ttysrf->tx_buffer;
				unsigned long flags;
				unsigned char byte;
				int loop;
				int ret;

				/*get data from tx_fifo */
				ttysrf->tx_len = 0;
				spin_lock_irqsave(&ttysrf->fifo_lock, flags);
				for (loop = 0; loop < tx_fifo_len; loop++) {
					ret =
					    kfifo_get(&ttysrf->tx_fifo, &byte);
					if (ret > 0) {
						/* format tx_buffer for "fe ff" */
						if (byte == 0xfe
						    || byte == 0xff) {
							*tx_buffer++ = 0xfe;
							ttysrf->tx_len++;
						}
						*tx_buffer++ = byte;
						ttysrf->tx_len++;
					}
				}
				spin_unlock_irqrestore(&ttysrf->fifo_lock,
						       flags);
				status = ttysrf_spi_send_msg(ttysrf);
			} else if (gpio_get_value(ttysrf->gpio.irq_pin)) {
				/* Construct SPI message to read 1 byte */
				ttysrf->tx_buffer[0] = 0xff;
				ttysrf->tx_len = 1;
				ttysrf_spi_send_msg(ttysrf);
			} else {
				set_current_state(TASK_INTERRUPTIBLE);
				schedule();
			}
		} else {
			/* delay a short time to wait for transfer to complete */
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(usecs_to_jiffies(100));
		}
	} while (!kthread_should_stop());

	return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static irqreturn_t ttysrf_interrupt(int irq, void *data)
{
	struct ttysrf_serial *ttysrf = data;
	wake_up_process(ttysrf->spi_task);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_open(struct tty_struct *tty, struct file *file)
{
	struct ttysrf_serial *ttysrf = ttysrf_saved;	//tty->driver_data;
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
	unsigned char *tmp_buf = (unsigned char *)buffer;
	int tx_count = kfifo_in_locked(&ttysrf->tx_fifo, tmp_buf, count,
				       &ttysrf->fifo_lock);
	/* poke tasklet to process data */
	/* wake up thread */
	wake_up_process(ttysrf->spi_task);

	return tx_count;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_write_room(struct tty_struct *tty)
{
	struct ttysrf_serial *ttysrf = tty->driver_data;
	int room = -EINVAL;

	/* calculate how much room is left in the device */
	room = TTYSRF_FIFO_SIZE - kfifo_len(&ttysrf->tx_fifo);
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

	dprintk("%s()\n", __func__);

	/* clear any old data; can't do this in 'close' */
	kfifo_reset(&ttysrf->tx_fifo);

	/* put port data into this tty */
	tty->driver_data = ttysrf;

	/* allows flip string push from int context */
	tty->low_latency = 1;

	/* start thread */
	ttysrf->spi_task =
	    kthread_run(ttysrf_spi_thread, ttysrf, TTYSRF_DRIVER_NAME);
	if (IS_ERR(ttysrf->spi_task)) {
		dprintk("cannot run ttysrf_spi_thread\n");
		return -ECHILD;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static void ttysrf_port_shutdown(struct tty_port *port)
{
	struct ttysrf_serial *ttysrf =
	    container_of(port, struct ttysrf_serial, tty_port);
	dprintk("%s()\n", __func__);

	/* stop threads, interrupts etc.. */
	kthread_stop(ttysrf->spi_task);
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
	kfifo_free(&ttysrf->tx_fifo);
	kfree(ttysrf->tx_buffer);
	kfree(ttysrf->rx_buffer);
	kfree(ttysrf);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_create_port(struct ttysrf_serial *ttysrf)
{
	int ret = 0;
	struct tty_port *pport = &ttysrf->tty_port;

	spin_lock_init(&ttysrf->fifo_lock);
	lockdep_set_class_and_subclass(&ttysrf->fifo_lock, &ttysrf_spi_key, 0);

	if (kfifo_alloc(&ttysrf->tx_fifo, TTYSRF_FIFO_SIZE, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto error_ret;
	}

	tty_port_init(pport);
	pport->ops = &ttysrf_tty_port_ops;
	ttysrf->minor = 0;

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

	dprintk("%s()\n", __func__);

	/* allocate memory for ttysrf structure */
	ttysrf = kzalloc(sizeof(*ttysrf), GFP_KERNEL);
	if (!ttysrf)
		return -ENOMEM;

	/* allocate memory for transmit buffer */
	ttysrf->tx_buffer = kzalloc(TTYSRF_SPI_TX_SIZE, GFP_KERNEL);
	if (!ttysrf->tx_buffer) {
		kfree(ttysrf);
		return -ENOMEM;
	}

	/* allocate memory for receive buffer */
	ttysrf->rx_buffer = kzalloc(TTYSRF_SPI_RX_SIZE, GFP_KERNEL);
	if (!ttysrf->rx_buffer) {
		kfree(ttysrf->tx_buffer);
		kfree(ttysrf);
		return -ENOMEM;
	}

	sema_init(&ttysrf->sem, 1);
	ttysrf->open_count = 0;
	ttysrf->spi_dev = spi;
	ttysrf->spi_busy = 0;
	ttysrf->gpio.irq_pin = gpio_irq_pin;

	/* TODO?: Do I need to create any buffer :s ?? */
	/* set driver user data */
	spi_set_drvdata(spi, ttysrf);

	/* create TTY port */
	ret = ttysrf_create_port(ttysrf);
	if (ret != 0) {
		dprintk("create default tty port failed");
		goto probe_error_1;
	}

	/* TODO: setup GPIO and Interrupt */
	ret = gpio_request(ttysrf->gpio.irq_pin, "ttysrf/irq");
	if (ret < 0) {
		dprintk("Unable to allocate GPIO%d (IRQ)",
			ttysrf->gpio.irq_pin);
		ret = -EBUSY;
		goto probe_error_1;
	}
	ret += gpio_export(ttysrf->gpio.irq_pin, 1);
	ret += gpio_direction_input(ttysrf->gpio.irq_pin);
	if (ret) {
		dprintk("Unable to configure GPIO%d (IRQ)", gpio_irq_pin);
		ret = -EBUSY;
		goto probe_error_2;
	}
	ret = request_irq(gpio_to_irq(ttysrf->gpio.irq_pin),
			ttysrf_interrupt,
			IRQF_TRIGGER_RISING, TTYSRF_DRIVER_NAME,
			(void *)ttysrf);

	/* set global pointer */
	ttysrf_saved = ttysrf;

	return 0;

 probe_error_2:
	/* release GPIO */
	gpio_free(ttysrf->gpio.irq_pin);
 probe_error_1:
	ttysrf_free_device(ttysrf);
	return ret;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int ttysrf_spi_remove(struct spi_device *spi)
{
	struct ttysrf_serial *ttysrf = spi_get_drvdata(spi);
	dprintk("%s()\n", __func__);
	/* free GPIOs and Interrupts */
	free_irq(gpio_to_irq(ttysrf->gpio.irq_pin), (void *)ttysrf);
	gpio_free(ttysrf->gpio.irq_pin);

	/* release SPI device from BUS */
	/*spi_unregister_device (spi); */
	/*spi_dev_put (spi); */

	/* free allocations */
	ttysrf_free_device(ttysrf);
	ttysrf_saved = NULL;
	return 0;
}

#if 0
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static int __init ttysrf_add_spi_device_to_bus(void)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	struct device *pdev;
	char buff[64];
	int status = 0;

	spi_master = spi_busnum_to_master(TTYSRF_SPI_BUS);
	if (!spi_master) {
		dprintk("spi_busnum_to_master(%d) returned NULL\n",
			TTYSRF_SPI_BUS);
		return -1;
	}

	spi_device = spi_alloc_device(spi_master);
	if (!spi_device) {
		put_device(&spi_master->dev);
		dprintk("spi_alloc_device() failed\n");
		return -1;
	}

	spi_device->chip_select = TTYSRF_SPI_BUS_CS0;

	/* Check whether this SPI bus.cs is already claimed */
	snprintf(buff, sizeof(buff), "%s.%u",
		 dev_name(&spi_device->master->dev), spi_device->chip_select);
	dprintk("buff = %s\n", buff);

	pdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buff);
	if (pdev) {
		/* We are not going to use this spi_device, so free it */
		spi_dev_put(spi_device);
		dprintk("pdev = true\n");
		/*
		 * There is already a device configured for this bus.cs
		 * It is okay if it us, otherwise complain and fail.
		 */
		if (pdev->driver && pdev->driver->name &&
		    strcmp(TTYSRF_DRIVER_NAME, pdev->driver->name)) {
			dprintk("Driver [%s] already registered for %s\n",
				pdev->driver->name, buff);
			status = -1;
		}
	} else {
		spi_device->max_speed_hz = TTYSRF_SPI_BUS_SPEED;
		spi_device->mode = SPI_MODE_0;
		spi_device->bits_per_word = 8;
		spi_device->irq = -1;
		spi_device->controller_state = NULL;
		spi_device->controller_data = NULL;
		strlcpy(spi_device->modalias, TTYSRF_DRIVER_NAME,
			SPI_NAME_SIZE);

		/* This will force the SPI probe callback */
		status = spi_add_device(spi_device);
		if (status < 0) {
			spi_dev_put(spi_device);
			dprintk("spi_add_device() failed: %d\n", status);
		}
	}

	put_device(&spi_master->dev);

	return status;
}
#endif

static const struct spi_device_id ttysrf_id_table[] = {
	{"ttysrf", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, ttysrf_id_table);

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
static struct spi_driver ttysrf_spi_driver = {
	.driver = {
		   .name = TTYSRF_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = ttysrf_spi_probe,
	.remove = __devexit_p(ttysrf_spi_remove),
	.id_table = ttysrf_id_table,
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
#if 0
	/* in a normal world we would do this in the board_init for the chip
	 * but because this is an experimental driver we force it here */
	error = ttysrf_add_spi_device_to_bus();
	if (error < 0) {
		dprintk("evesrf_add_to_bus() failed\n");
		spi_unregister_driver(&ttysrf_spi_driver);
		return error;
	}
#endif

	return 0;
}

/* ------------------------------------------------------------------ */
/* ttysrf_init_tty()
 * This function is used to setup the tty device. */
/* ------------------------------------------------------------------ */
static int __init ttysrf_init_tty(void)
{
	int ret = 0;

	/* allocate memory for tty driver. */
	/*ttysrf_tty_driver = tty_alloc_driver (TTYSRF_MINORS, TTY_DRIVER_UNNUMBERED_NODE); */
	ttysrf_tty_driver = alloc_tty_driver(TTYSRF_MINORS);
	if (!ttysrf_tty_driver) {
		dprintk("Failed to allocate memory for ttysrf driver!\n");
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
static int __init ttysrf_init(void)
{
	int ret = 0;

	dprintk("%s()\n", __func__);

	/* initialise TTY driver */
	ret = ttysrf_init_tty();
	if (ret < 0)
		goto init_error_1;

	/* initialise SPI driver */
	ret = ttysrf_init_spi();
	if (ret < 0)
		goto init_error_2;

	return ret;

 init_error_2:
	/* free tty driver */
	tty_unregister_driver(ttysrf_tty_driver);

 init_error_1:
	/* free structure */
	/*kfree(ttysrf); */
	return ret;
}

module_init(ttysrf_init);

/* ------------------------------------------------------------------ */
/* ttysrf_exit()
 * This function is called when the kernel module is removed. */
/* ------------------------------------------------------------------ */
static void __exit ttysrf_exit(void)
{
	dprintk("%s()\n", __func__);

	/* unregister SPI driver */
	spi_unregister_driver(&ttysrf_spi_driver);
	/* unregister TTY device and driver */
	tty_unregister_device(ttysrf_tty_driver, 0);
	tty_unregister_driver(ttysrf_tty_driver);

	/* free ttysrf control structure */
	kfree(ttysrf_saved);
}

module_exit(ttysrf_exit);

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
MODULE_DESCRIPTION("EVE Alpha SRF tty driver for Raspberry Pi.");
MODULE_AUTHOR("Jon Cross <joncross.cooljc@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
module_param(gpio_irq_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_irq_pin, "GPIO irq pin number of the BCM processor."
		 " Valid pin numbers are: 0, 1, 4, 8, 7, 9, 10, 11, 14, 15,"
		 " 17, 18, 21, 22, 23, 24, 25, default 25");
