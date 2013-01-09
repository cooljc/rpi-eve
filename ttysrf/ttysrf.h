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

#define TTYSRF_DRIVER_NAME  "ttysrf"
#define TTYSRF_MINORS       1

struct ttysrf_serial {
  struct tty_struct *tty;  /* pointer to the tty for this device */
  struct device *tty_dev;
  int open_count;          /* number of times this port has been opened */
  struct semaphore sem;    /* locks this structure */
};

#endif /* #ifndef _TTYSRF_H */
/* EOF */
