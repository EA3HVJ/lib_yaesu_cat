/*
 * Copyright (C) 2018 EA3HVJ - Joan Planella Costa
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "yaesu_cat.h"

/****************
 * POSIX SERIAL *
 ****************/
#ifdef POSIX_COMPILATION
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "debug.h"

/************************
 * SERIAL DEVICE CONFIG *
 ************************/

/* 8 Bit + 2 Stop */
#define MAPI_8N2(BITRATE) BITRATE|CS8|CLOCAL|CREAD|CSTOPB
/* Inter-character timeout */
#define CAT_VTIME 2
#define CAT_VMIN  255 /* 255 for clone mode */

struct CAT_device {
	int timeout;
	int tty;
	struct termios old_tio, cat_tio;
	struct timeval tty_tv;
	fd_set tty_read;
};

struct CAT_device *
CAT_open_device(char* device, int bitrate, int timeout)
{
	DEBUG_PLACE;
	struct CAT_device* dev = malloc(sizeof(struct CAT_device));
	if (!dev) {
		g_cat_errno = CAT_ERR_MALLOC;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	}

	int rate;
	switch (bitrate) {
	case 4800:
		rate = B4800;
		break;
	case 9600:
		rate = B9600;
		break;
	case 38400:
		rate = B38400;
		break;
	default:
		g_cat_errno = CAT_ERR_BITRATE;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		free(dev);
		return NULL;
	}
	/* Open device */
	dev->tty = open(device, O_RDWR | O_NOCTTY);
	if (dev->tty < 0) {
		g_cat_errno = CAT_ERR_DEVICE;
		DEBUG_PRINT("%s: %s", device, CAT_errno_to_str(g_cat_errno));
		free(dev);
		return NULL;
	}

	/* RTS is set by default, clear for hardware PTT uses */
	int RTS_flag = TIOCM_RTS;
	if (ioctl(dev->tty, TIOCMBIC, &RTS_flag) == -1) {
		g_cat_errno = CAT_ERR_RTS;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		close(dev->tty);
		free(dev);
		return NULL;
	}

	/* Save current port settings */
	tcgetattr(dev->tty, &dev->old_tio);

	/* 
	 * SET-UP CAT TIO 
	 * Port settings:
	 * 1b start + 8b data + 2b stop - no parity - non canonical
	 */
	bzero(&dev->cat_tio, sizeof(dev->cat_tio));
	dev->cat_tio.c_cflag = MAPI_8N2(rate);
	dev->cat_tio.c_iflag = IGNPAR;
	dev->cat_tio.c_oflag = 0;
	dev->cat_tio.c_lflag = 0;
	/* Set intercharacter timer to 200ms */
	dev->cat_tio.c_cc[VTIME] = CAT_VTIME;
	/* Blocking read until n chars received */
	dev->cat_tio.c_cc[VMIN] = CAT_VMIN;

	/* Default TTY settings: CAT tio */
	tcflush(dev->tty, TCIFLUSH);
	tcsetattr(dev->tty, TCSANOW, &dev->cat_tio);

	/* Set up select() */
	dev->timeout = timeout;
	FD_SET(dev->tty, &dev->tty_read);

	DEBUG_PRINT("Device: \"%s\", Rate: %d bps, Transaction timeout: %d", device, bitrate, timeout);
	return dev;
}

void
CAT_close_device(struct CAT_device* dev)
{
	tcsetattr(dev->tty, TCSANOW, &dev->old_tio);
	close(dev->tty);
	free(dev);
	dev = NULL;
}

bool CAT_switch_rts_ptt(struct CAT_device* dev, bool on_off)
{
	DEBUG_PLACE;

	int RTS_flag = TIOCM_RTS;
	int res;

	if (on_off == false) {
		res = ioctl(dev->tty, TIOCMBIC, &RTS_flag);
	} else {
		res = ioctl(dev->tty, TIOCMBIS, &RTS_flag);
	}

	if (res == -1) {
		g_cat_errno = CAT_ERR_RTS;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return false;
	} else {
		return true;
	}
}

int CAT_get_cts_sql(struct CAT_device* dev)
{
	int CTS_flag;
	int res;

	res = ioctl(dev->tty, TIOCMGET, &CTS_flag);
	if (res == -1) {
		g_cat_errno = CAT_ERR_CTS;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return -1;
	} else {
		return(CTS_flag & TIOCM_CTS) != 0;
	}
}

int CAT_change_device_rate(struct CAT_device* dev, int new_rate)
{
	speed_t new_speed;
	speed_t old_speed = cfgetispeed(&dev->cat_tio);

	switch (new_rate) {
	case 4800:
		new_speed = B4800;
		break;
	case 9600:
		new_speed = B9600;
		break;
	case 38400:
		new_speed = B38400;
		break;
	default: return(-1);
	}

	if (old_speed == new_speed) return new_rate;

	cfsetspeed(&dev->cat_tio, new_speed);
	tcsetattr(dev->tty, TCSANOW, &dev->cat_tio);
	tcflush(dev->tty, TCIOFLUSH);

	switch (old_speed) {
	case B4800: return 4800;
	case B9600: return 9600;
	case B38400: return 38400;
	}
}

int
CAT_change_transaction_timeout(struct CAT_device* dev, int milliseconds)
{
	int old_timeout = dev->timeout;
	dev->timeout = milliseconds;
	return old_timeout;
}

int
CAT_write_bytes(struct CAT_device* dev, uint8_t* buf, size_t nbytes)
{
	int n_written = write(dev->tty, buf, nbytes);
	if (n_written != nbytes) {
		g_cat_errno = CAT_ERR_W_REQUEST;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return 0; // Error
	} else {
		return 1;
	}
}

int
CAT_read_bytes(struct CAT_device* dev, uint8_t* dest, size_t size)
{
	int n_read, res;

	dev->tty_tv.tv_usec = dev->timeout * 1000;
	res = select(dev->tty + 1, &dev->tty_read, NULL, NULL, &dev->tty_tv);
	if (res == -1) {
		g_cat_errno = CAT_ERR_SELECT;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return 0;
	} else if (res == 0) {
		g_cat_errno = CAT_ERR_TRANSACTION_TIMEDOUT;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return 0; //Timeout
	} else {
		n_read = read(dev->tty, dest, size);
		if (n_read < 1) {
			g_cat_errno = CAT_ERR_R_RESPONSE;
			DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
			return 0; // Error
		} else {
#ifdef _DEBUG
			DPRINTF(DEBUG_STR "Response: ");
			int i;
			for (i = 0; i < n_read; i++) {

				DPRINTF("0x%02X ", (uint8_t) dest[i]);
			}
			DPRINTF("\n");
#endif
			return n_read; // Success
		}
	}
}

#endif