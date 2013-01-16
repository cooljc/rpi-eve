/*
 * test-ttysrf-dev.c
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
#include <stdlib.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>


#define TTYSRF_DEV "/dev/ttySRF0"
#define TTYURF_DEV "/dev/ttyACM0"

ssize_t read_timeout (int fd, void *buf, size_t count, int timeout_s)
{
	fd_set rfds;
	struct timeval tv;
	int retval = 0;
	ssize_t n = 0;

	/* Wait up to five seconds. */
	tv.tv_sec = timeout_s; /* timeout */
	tv.tv_usec = 0;
	/* Watch fd to see when it has input. */
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	retval = select(fd+1, &rfds, NULL, NULL, &tv);
	if (retval == -1) {
		fprintf (stderr, "Error: select() returned -1!\n");
		n = -1;
	}
	else if(retval) {
		n =  read(fd, buf, count);
	}
	else {
		n = -2;
	}
	return n;
}

/* 
 * we want to send commands and wait for the result + OK.
 */
int send_AT_command (int fd, const char *command, char *result)
{
	fd_set rfds;
	struct timeval tv;
	int retval = 0;
	char buf[64];
	int pos = 0;
  
	/* clean buf */
	memset (buf, 0, 64);

	/* send command */
	write (fd, command, strlen(command));

	/* Wait up to five seconds. */
	tv.tv_sec = 5; /* timeout */
	tv.tv_usec = 0;

	while (1) {
		/* Watch fd to see when it has input. */
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		retval = select(fd+1, &rfds, NULL, NULL, &tv);
		if (retval == -1) {
			fprintf (stderr, "Error: select() returned -1!\n");
			return -1;
		}
		else if(retval) {
			char byte;
			int n =  read(fd, &byte, 1);
			if (n) {
				//fprintf (stderr, "[%02x] %c\n", byte, byte);
				buf[pos++] = byte;
			}
		}
		else {
			fprintf (stderr, "Timeout: waiting for data! (%s)\n", buf);
			return -2;
		}

		/* look for "OK"+<CR> */
		if ( strstr (buf, "OK\r") != NULL) {
			//fprintf (stderr, "Found \"OK\\r\"!!\n");
			/* copy data to result minus the "OK" */
			if (result) {
				/* we should have: 
				 * <command result> + <CR> + OK + <CR> 
				 * Strip off the OK and 2 <CR> characters */
				memcpy (result, &buf[0], strlen(buf)-4);
			}
			break;
		}
	} /* end of while(1) */
	return 0;
}

int get_version (int fd, const char *devname)
{
	int ret = 0;
	fprintf (stderr, "%s entering AT mode using \"+++\"\n", devname);
	if (send_AT_command (fd, "+++", NULL) == 0) {
		char fw_version[20];
		memset (fw_version, 0, 20);

		if (send_AT_command (fd, "ATVR\r", fw_version) == 0) {
			fprintf (stderr, "Firmware version: %s\n", fw_version);
			/* leave AT mode */
			ret = send_AT_command (fd, "ATDN\r", NULL);
		}
		else {
			ret = -1;
		}
	}
	else {
		fprintf (stderr, "%s entering AT mode failed!!\n", devname);
		ret = -1;
	}
	return ret;
}

void set_port_options (int fd)
{
	/* set up serial port */
	struct termios options;

	fcntl(fd, F_SETFL, FNDELAY); /* don't block if no data is available */
	/* Get the current options for the port... */
	tcgetattr(fd, &options);
	/* Set the baud rates to 9600... */
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);
	/* Enable the receiver and set local mode... */
	options.c_cflag |= (CLOCAL | CREAD);

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	/* disable hardware flow control */
	//options.c_cflag &= ~CNEW_RTSCTS;

	/* Set the new options for the port... */
	tcsetattr(fd, TCSANOW, &options);

	/* flush buffers */
	tcflush(fd, TCIOFLUSH);
}

int test1 (int out_fd, int in_fd)
{
	unsigned char data_out[1];
	unsigned char data_in[1];
	int loop;
	int error = 0;

	for (loop=0; loop<256; loop++) {
		data_out[0] = loop;
		/* write one byte out */
		write (out_fd, data_out, 1);
		/* read one byte in */
		if (read_timeout (in_fd, data_in, 1, 5) > 0) {
			/* compare byte */
			if (data_out[0] != data_in[0]) {
				fprintf (stderr, "Read: 0x%02x, Expected: 0x%02x\n",
					data_in[0], data_out[0]);
				error++;
			}
		}
		else {
			fprintf (stderr, "Loop(%d): timeout\n", loop);
		}
	}
	return error;
}

int test2 (int out_fd, int in_fd)
{
	unsigned char data_out[128];
	unsigned char data_in[128];
	int loop = 0;
	int error = 0;
	ssize_t in_size = 0;
	int in_pos = 0;

	/* fill data_out with test pattern */
	for (loop=0; loop<128; loop++) {
		data_out[loop] = loop;
	}

	/* clear data_in */
	memset (data_in, 0, 128);

	/* write data */
	write (out_fd, data_out, 128);

	/* read data in */
	do {
		in_size = read_timeout (in_fd, &data_in[in_pos], 128, 5);
		in_pos += in_size;
		if (in_size <= 0) 
			break;
	} while (in_pos < 128);

	/* check results */
	if (in_pos > 0) {
		/* compare bytes */
		for (loop=0; loop<128; loop++) {
			if (data_out[loop] != data_in[loop]) {
				fprintf (stderr, "Read: 0x%02x, Expected: 0x%02x\n",
					data_in[loop], data_out[loop]);
				error++;
			}
		}
	}
	else {
		fprintf (stderr, "read(): timeout\n");
		error = -1;
	}
	return error;
}

int main (int argc, char *argv[])
{
	int srf_fd = -1;
	int urf_fd = -1;
	int errors = 0;
	int ret = EXIT_SUCCESS;

	/* open SRF device */
	srf_fd = open(TTYSRF_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
	if (srf_fd == -1) {
		fprintf (stderr, "Error: failed to open %s!!\n", TTYSRF_DEV);
		ret = EXIT_FAILURE;
		goto exit_1;
	}
	set_port_options (srf_fd);
	if (get_version (srf_fd, "SRF") != 0) {
		goto exit_2;
	}

	/* open URF device */
	urf_fd = open(TTYURF_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
	if (urf_fd == -1) {
		fprintf (stderr, "Error: failed to open %s!!\n", TTYURF_DEV);
		ret = EXIT_FAILURE;
		goto exit_2;
	}
	set_port_options (urf_fd);
	if (get_version (urf_fd, "URF") != 0) {
		goto exit_3;
	}

	/* ------------------------------------------------
	 * TEST 1:
	 * This tests sending and receiving 1 byte at a time.
	 * First we send from URF to SRF.
	 * Second we send from SRF to URF */
	fprintf (stderr, "********* TEST 1 **********\n");
	fprintf (stderr, "URF -> SRF:\n");
	errors = test1 (urf_fd, srf_fd);
	fprintf (stderr, "Errors: %d\n", errors);

	fprintf (stderr, "SRF -> URF:\n");
	errors = test1 (srf_fd, urf_fd);
	fprintf (stderr, "Errors: %d\n", errors);


	/* ------------------------------------------------
	 * TEST 2:
	 * This tests sending and receiving 128 bytes at a time.
	 * First we send from URF to SRF.
	 * Second we send from SRF to URF */
	fprintf (stderr, "********* TEST 2 **********\n");
	fprintf (stderr, "URF -> SRF:\n");
	errors = test2 (urf_fd, srf_fd);
	fprintf (stderr, "Errors: %d\n", errors);

	fprintf (stderr, "SRF -> URF:\n");
	errors = test2 (srf_fd, urf_fd);
	fprintf (stderr, "Errors: %d\n", errors);

	/* done */
exit_3:
	close (urf_fd);
exit_2:
	close (srf_fd);
exit_1:
	return ret;
}

/* EOF */
