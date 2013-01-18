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
#ifndef _TTYSRF_H
#define _TTYSRF_H

#define TTYSRF_DRIVER_NAME   "ttysrf"
#define TTYSRF_MINORS        1

/* SPI defines */
#define TTYSRF_SPI_BUS       0
#define TTYSRF_SPI_BUS_CS0   0
#define TTYSRF_SPI_BUS_SPEED 100000

#define TTYSRF_FIFO_SIZE     1024
#define TTYSRF_SPI_TX_SIZE   246 /* TODO: Confirm this */
#define TTYSRF_SPI_RX_SIZE   TTYSRF_SPI_TX_SIZE

struct ttysrf_serial {
	/* our SPI device */
	struct spi_device *spi_dev;
	struct task_struct *spi_task;

	/* port specific data */
	struct kfifo tx_fifo;
	spinlock_t fifo_lock;
	unsigned int signal_state;
	unsigned char *tx_buffer;
	unsigned char *rx_buffer;
	int tx_len;
	int rx_len;
	struct spi_message spi_msg;
	struct spi_transfer spi_xfer;
	struct semaphore spi_busy;   /* locks spi thread while busy */
	int fe_flag;

	/* TTY Layer logic */
	struct tty_port tty_port;
	struct device *tty_dev;
	int minor;

	struct {
		int irq_pin;
		int irq_num;
	} gpio;
};

#endif				/* #ifndef _TTYSRF_H */
/* EOF */
