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
 * Some code is taken from:
 * http://www.easysw.com/~mike/serial/serial.html
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

#define ENABLE_TEST1
#define ENABLE_TEST2
#define ENABLE_TEST3

#define TTYSRF_DEV "/dev/ttySRF0"
#define TTYURF_DEV "/dev/ttyACM0"

#define TEST3_FILE  "Tux.png"

#define min(a,b) ((a)<(b)?(a):(b))

void print_bytes (char *buf)
{
	int loop = 0;
	for (loop=0; loop<strlen(buf); loop++) {
		fprintf (stderr, "[%02x]", buf[loop]);
	}
	fprintf (stderr, "\n");
}

/* ------------------------------------------------------------------ *
 * we want to send commands and wait for the result + OK.
 * ------------------------------------------------------------------ */
int send_AT_command (int fd, const char *command, char *result)
{
	char buffer[255];  /* Input buffer */
	char *bufptr;      /* Current char in buffer */
	int  nbytes;       /* Number of bytes read */
	int  tries;        /* Number of tries so far */
	char ok[4] = {'O', 'K', 0x0d, 0};

	for (tries = 0; tries < 3; tries ++) {
		/* send an AT command followed by a CR */
		if (write(fd, command, strlen(command)) < strlen(command))
			continue;

		/* clear buffer */
		memset (buffer, 0, 255);

		/* read characters into our string buffer until we get a CR or NL */
		bufptr = buffer;
		nbytes = read(fd, bufptr, sizeof(buffer));
		do {
			bufptr += nbytes;
			//print_bytes(buffer);
			if (strstr (buffer, ok) != NULL) {
				if (result != NULL) {
					/* we should have: 
					 * <command result> + <CR> + OK + <CR> 
					 * Strip off the OK and 2 <CR> characters */
					memcpy (result, &buffer[0], strlen(buffer)-4);
				}
				return (0);
			}
		} while ((nbytes = read(fd, bufptr, buffer + sizeof(buffer) - bufptr - 1)) > 0);

	}

	return (-1);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
void set_port_options (int fd)
{
	/* set up serial port */
	struct termios options;
	fcntl(fd, F_SETFL, 0);

	/* get the current options */
	memset(&options, 0, sizeof(options));

	/* Set the baud rates to 9600... */
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);

	/* set raw input, 3 second timeout */
	options.c_cflag     = (CLOCAL | CREAD);
	options.c_cflag     &= ~PARENB;
	options.c_cflag     &= ~CSTOPB;
	options.c_cflag     &= ~CSIZE;
	options.c_cflag     |= CS8;
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag     &= ~(OPOST);
	options.c_iflag     = (IGNPAR | IGNBRK);
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = 30;

	/* set the options */
	tcsetattr(fd, TCSANOW, &options);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
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
		if (read(in_fd, data_in, 1) > 0) {
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

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
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
		in_size = read(in_fd, &data_in[in_pos], 128);
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

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
int test3 (int out_fd, int in_fd)
{
	int fd = -1;
	int ret = -1;
	unsigned char *data_in;
	unsigned char *data_out;
	struct stat filestat;
	int test_finished = 0;
	int out_pos = 0;
	int in_pos = 0;
	int filesize = 0;
	int loop = 0;
	int errors = 0;

	/* Open test file for transfer */
	fd = open(TEST3_FILE, O_RDWR);
	if (fd < 0) {
		fprintf (stderr, "Error: Failed to open file: %s\n", TEST3_FILE);
		return ret;
	}
	/* get file stats */
	if (fstat(fd, &filestat) < 0) {
		fprintf (stderr, "Error: Failed to stat file: %s\n", TEST3_FILE);
		goto test3_exit_1;
	}
	filesize = (int)filestat.st_size;
	fprintf (stderr, "filestat.st_size=%d\n", filesize);

	/* allocate memory for in and out data */
	data_out = malloc(filestat.st_size);
	if (data_out == NULL) {
		fprintf (stderr, "Error: Failed to allocate memory for data_out\n");
		goto test3_exit_1;
	}
	data_in = malloc(filestat.st_size);
	if (data_in == NULL) {
		fprintf (stderr, "Error: Failed to allocate memory for data_in\n");
		goto test3_exit_2;
	}

	/* set memory to zero */
	memset(data_out, 0, filestat.st_size);
	memset(data_in, 0, filestat.st_size);

	/* read file data into memory */
	if (read(fd, data_out, filestat.st_size) != filestat.st_size) {
		fprintf (stderr, "Error: Failed to read file into memory!\n");
		goto test3_exit_3;
	}

	fprintf (stderr, "Sending...\n");

	/* send/receive data */
	do {
		int write_size = 0;
		int read_size = 0;
		if (out_pos < filesize ) {
			write_size = min ((filesize-out_pos), 200);
			out_pos += write(out_fd, &data_out[out_pos], write_size);
		}
		/* allow read to catch up.. */
		while (read_size < write_size) {
			read_size += read (in_fd, &data_in[in_pos+read_size], write_size);
		}
		in_pos += read_size;
		fprintf (stderr, "out_pos=%d, in_pos=%d\r", out_pos, in_pos);
		if (in_pos >= filesize && out_pos >= filesize) {
			test_finished = 1;
			fprintf (stderr, "\n");
		}
	} while (!test_finished);

	/* compare memory */
	for (loop=0; loop<filestat.st_size; loop++) {
		if (data_out[loop] != data_in[loop]) {
			errors += 1;
		}
	}
	ret = errors;

	/* Clean up */
test3_exit_3:
	free(data_in);
test3_exit_2:
	free(data_out);
test3_exit_1:
	close(fd);
	return ret;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
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

#ifdef ENABLE_TEST1
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
#endif


#ifdef ENABLE_TEST2
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
#endif

#ifdef ENABLE_TEST3
	/* ------------------------------------------------
	 * TEST 3:
	 * This test opens a file and sends it 200 bytes at a time.
	 * First we send from URF to SRF.
	 * Second we send from SRF to URF */
	fprintf (stderr, "********* TEST 3 **********\n");
	fprintf (stderr, "URF -> SRF:\n");
	errors = test3 (urf_fd, srf_fd);
	fprintf (stderr, "Errors: %d\n", errors);

	fprintf (stderr, "SRF -> URF:\n");
	errors = test3 (srf_fd, urf_fd);
	fprintf (stderr, "Errors: %d\n", errors);
#endif
	/* done */
exit_3:
	close (urf_fd);
exit_2:
	close (srf_fd);
exit_1:
	return ret;
}

/* EOF */
