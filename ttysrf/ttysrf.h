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
#define TTYSRF_SPI_BUS_SPEED 1000000

#define TTYSRF_FIFO_SIZE     4096

struct ttysrf_serial {
  /* our SPI device */
  struct spi_device *spi_dev;
  struct task_struct *spi_task;

  /* port specific data */
  struct kfifo tx_fifo;
  spinlock_t fifo_lock;
  unsigned int signal_state;
  unsigned char *tx_buffer;

  /* TTY Layer logic */
  struct tty_port tty_port;
  struct device *tty_dev;
  int minor;

  struct {
    int irq_pin;
    int irq_num;
  } gpio;

  int open_count;          /* number of times this port has been opened */
  struct semaphore sem;    /* locks this structure */
};

#endif /* #ifndef _TTYSRF_H */
/* EOF */
