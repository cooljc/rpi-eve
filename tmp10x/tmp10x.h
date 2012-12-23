/*
 * tmp10x.h
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
#ifndef _TMP10X_H
#define _TMP10X_H

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

typedef struct _tmp10x_temp_s
{
  unsigned short  m_temp;
} tmp10x_temp_t;

typedef struct _tmp10x_config_s
{
  unsigned char m_sd;  /* 1 bit: */
  unsigned char m_tm;  /* 1 bit: */
  unsigned char m_pol; /* 1 bit: */
  unsigned char m_fq;  /* 2 bits: */
  unsigned char m_cr;  /* 2 bits: */
  unsigned char m_os;  /* 1 bit: */
} tmp10x_config_t;

#define TMP10X_IOC_MAGIC      0x23
#define TMP10X_IOC_GETTEMP    _IOR(TMP10X_IOC_MAGIC, 1, tmp10x_temp_t)
#define TMP10X_IOC_GETCONFIG  _IOR(TMP10X_IOC_MAGIC, 2, tmp10x_config_t)
#define TMP10X_IOC_SETCONFIG  _IOR(TMP10X_IOC_MAGIC, 3, tmp10x_config_t)

#endif /* #ifndef _TMP10X_H */
/* EOF */
