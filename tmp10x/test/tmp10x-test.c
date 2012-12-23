/*
 * tmp10x-test.c
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "../tmp10x.h"

int tmp10x_read_temp (int fd)
{
  int ret = -1;
  tmp10x_temp_t temp;

  ret = ioctl(fd, TMP10X_IOC_GETTEMP, &temp);
  if (ret == 0) {
    int tmp1 = (temp.m_temp >> 4);
    int tmp2 = (temp.m_temp & 0x0f) * 625;
    fprintf (stderr, "temp.m_temp   = 0x%02x -> ", temp.m_temp);
    fprintf (stderr, "Temperature: %d.%04d C\n", tmp1, tmp2);
  }
  else {
    fprintf (stderr, "Error: ioctl(TMP10X_IOC_GETTEMP) failed (err=%d)\n", ret);
  }

  return ret;
}

int tmp10x_read_config (int fd)
{
  int ret = -1;
  tmp10x_config_t c;

  ret = ioctl(fd, TMP10X_IOC_GETCONFIG, &c);
  if (ret == 0) {
    fprintf (stderr, "c.m_sd   = 0x%02x\n", c.m_sd);
    fprintf (stderr, "c.m_tm   = 0x%02x\n", c.m_tm);
    fprintf (stderr, "c.m_pol  = 0x%02x\n", c.m_pol);
    fprintf (stderr, "c.m_fq   = 0x%02x\n", c.m_fq);
    fprintf (stderr, "c.m_cr   = 0x%02x\n", c.m_cr);
    fprintf (stderr, "c.m_os   = 0x%02x\n", c.m_os);
  }
  else {
    fprintf (stderr, "Error: ioctl(TMP10X_IOC_GETCONFIG) failed (err=%d)\n", ret);
  }

  return ret;
}

int tmp10x_write_config (int fd)
{
  int ret = -1;
  tmp10x_config_t c;

  // first get config
  ret = ioctl(fd, TMP10X_IOC_GETCONFIG, &c);
  if (ret == 0) {
    // change resolution to max
    c.m_cr = 3;
    // write new config
    ret = ioctl(fd, TMP10X_IOC_SETCONFIG, &c);
    if (ret != 0) {
      fprintf (stderr, "Error: ioctl(TMP10X_IOC_SETCONFIG) failed (err=%d)\n", ret);
    }
  }
  else {
    fprintf (stderr, "Error: ioctl(TMP10X_IOC_GETCONFIG) failed (err=%d)\n", ret);
  }

  return ret;
}

int main (int argc, char *argv[])
{
  int fd = -1;

  /* Open device node */
  fd = open("/dev/tmp10x", O_RDWR);
  if (fd < 0) {
    fprintf (stderr, "Error: Failed to open /dev/tmp10x!!\n");
    return EXIT_FAILURE;
  }

  /* test GETCONFIG ioctl */
  tmp10x_read_config (fd);

  /* test SETCONFIG ioctl */
  tmp10x_write_config (fd);

  /* read back config to see if it is written */
  tmp10x_read_config (fd);

  /* test GETTEMP ioctl */
  tmp10x_read_temp (fd);

  /* Close device */
  close (fd);

  return EXIT_SUCCESS;
}

/* EOF */
