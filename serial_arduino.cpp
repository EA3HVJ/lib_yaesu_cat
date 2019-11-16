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

/******************
 * ARDUINO SERIAL *
 *****************/
#ifdef ARDUINO_COMPILATION
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#define VTIME 200 //milliseconds

enum serial_type {
	HW_SERIAL,
	SW_SERIAL
};

struct CAT_device {
	int rate;
	int timeout;
	enum serial_type stype;
	void* tty;
	uint8_t rts_ptt;
	uint8_t cts_sql;
};

struct CAT_device *
CAT_open_device(char* device, int bitrate, int timeout)
{
	int sw_rx, sw_tx;
	int sw_inv = 0;

	if (CAT_check_dev_rate(bitrate) == false) {
		g_cat_errno = CAT_ERR_BITRATE;
		return NULL;
	}

	struct CAT_device* dev = (struct CAT_device*) malloc(sizeof(struct CAT_device));
	if (!dev) {
		g_cat_errno = CAT_ERR_MALLOC;
		return NULL;
	}

	if (sscanf(device, "SoftwareSerial(%d,%d),%d,%d", &sw_rx, &sw_tx, &dev->rts_ptt, &dev->cts_sql) == 4 ||
		sscanf(device, "SoftwareSerial(%d,%d,%d),%d,%d", &sw_rx, &sw_tx, &sw_inv, &dev->rts_ptt, &dev->cts_sql) == 5) {
		SoftwareSerial* sws = new SoftwareSerial(sw_rx, sw_tx, sw_inv);
		sws->begin(bitrate/*, SERIAL_8N2*/);
		sws->setTimeout(VTIME);
		dev->tty = (void*) sws;
		dev->stype = SW_SERIAL;

#ifdef HAVE_HWSERIAL0
	} else if (sscanf(device, "Serial,%d,%d", &dev->rts_ptt, &dev->cts_sql) == 2) {
		Serial.begin(bitrate, SERIAL_8N2);
		Serial.setTimeout(VTIME);
		dev->tty = (void*) &Serial;
		dev->stype = HW_SERIAL;
#endif
#ifdef HAVE_HWSERIAL1
	} else if (sscanf(device, "Serial1,%d,%d", &dev->rts_ptt, &dev->cts_sql) == 2) {
		Serial1.begin(bitrate, SERIAL_8N2);
		Serial1.setTimeout(VTIME);
		dev->tty = (void*) &Serial1;
		dev->stype = HW_SERIAL;
#endif
#ifdef HAVE_HWSERIAL2
	} else if (sscanf(device, "Serial2,%d,%d", &dev->rts_ptt, &dev->cts_sql) == 2) {
		Serial2.begin(bitrate, SERIAL_8N2);
		Serial2.setTimeout(VTIME);
		dev->tty = (void*) &Serial2;
		dev->stype = HW_SERIAL;
#endif
#ifdef HAVE_HWSERIAL3
	} else if (sscanf(device, "Serial3,%d,%d", &dev->rts_ptt, &dev->cts_sql) == 2) {
		Serial3.begin(bitrate, SERIAL_8N2);
		Serial3.setTimeout(VTIME);
		dev->tty = (void*) &Serial3;
		dev->stype = HW_SERIAL;
#endif
	} else {
		g_cat_errno = CAT_ERR_DEVICE;
		free(dev);
		return NULL;
	}

	pinMode(dev->rts_ptt, OUTPUT);
	pinMode(dev->cts_sql, INPUT);

	dev->timeout = timeout;

	return dev;
}

void
CAT_close_device(struct CAT_device* dev)
{
	HardwareSerial* hws;
	SoftwareSerial* sws;
	if (dev->stype == HW_SERIAL) {
		hws = (HardwareSerial*) dev->tty;
		hws->end();
	} else if (dev->stype == SW_SERIAL) {
		sws = (SoftwareSerial*) dev->tty;
		sws->end();
		delete sws;
	}
	free(dev);
	dev = NULL;
}

bool CAT_switch_rts_ptt(struct CAT_device* dev, bool on_off)
{
	if (on_off == false) {
		digitalWrite(dev->rts_ptt, LOW);
	} else {
		digitalWrite(dev->rts_ptt, HIGH);
	}
	return true;
}

int CAT_get_cts_sql(struct CAT_device* dev)
{
	int status = digitalRead(dev->cts_sql);
	if (status == LOW) {
		return 0;
	} else {
		return 1;
	}
}

int CAT_change_device_rate(struct CAT_device* dev, int new_rate)
{
	HardwareSerial* hws;
	SoftwareSerial* sws;

	int old_rate;
	if (dev->rate == new_rate) {
		return new_rate;
	} else {
		old_rate = dev->rate;
	}

	switch (new_rate) {
	case 4800:
	case 9600:
	case 38400:
		if (dev->stype == HW_SERIAL) {
			hws = (HardwareSerial*) dev->tty;
			hws->end();
			hws->begin(new_rate, SERIAL_8N2);
		} else if (dev->stype == SW_SERIAL) {
			sws = (SoftwareSerial*) dev->tty;
			sws->end();
			sws->begin(new_rate/*, SERIAL_8N2*/);
		}
		dev->rate = new_rate;
		return old_rate;
	default: return -1;
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
	HardwareSerial* hws;
	SoftwareSerial* sws;
	int n_written;

	if (dev->stype == HW_SERIAL) {
		hws = (HardwareSerial*) dev->tty;
		n_written = hws->write(buf, nbytes);
	} else if (dev->stype == SW_SERIAL) {
		sws = (SoftwareSerial*) dev->tty;
		n_written = sws->write(buf, nbytes);
	}
	if (n_written != nbytes) {
		g_cat_errno = CAT_ERR_W_REQUEST;
		return 0; // Error
	} else {
		return 1;
	}
}

int
CAT_read_bytes(struct CAT_device* dev, uint8_t* dest, size_t size)
{
	HardwareSerial* hws;
	SoftwareSerial* sws;
	int n_read;
	unsigned long time;
	bool timed_out = true;
	int n_available;

	time = millis();

	if (dev->stype == HW_SERIAL) {
		hws = (HardwareSerial*) dev->tty;
		while (millis() - time < dev->timeout) {
			n_available = hws->available();
			if (n_available > 0) {
				timed_out = false;
				n_read = hws->readBytes((char*) dest, size);
				break;
			}
		}
	} else if (dev->stype == SW_SERIAL) {
		sws = (SoftwareSerial*) dev->tty;
		while (millis() - time < dev->timeout) {
			n_available = sws->available();
			if (n_available > 0) {
				timed_out = false;
				n_read = sws->readBytes((char*) dest, size);
				break;
			}
		}
	}

	if (timed_out == true) {
		g_cat_errno = CAT_ERR_TRANSACTION_TIMEDOUT;
		return 0;
	} else if (n_read < 1) {
		g_cat_errno = CAT_ERR_R_RESPONSE;
		return 0;
	} else {
		return n_read;
	}
}
#endif