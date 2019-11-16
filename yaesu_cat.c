/*
 * Copyright (C) 2018 Joan Planella Costa - EA3HVJ
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

#include <stdio.h>
#include <string.h>
#include "debug.h"

#ifdef _WIN32
#define __thread __declspec(thread)
#endif

/***************
 * ARRAY SIZES *
 ***************/

#define NUM_OP_MODES        11
#define NUM_OP_MODES_SET    9 // Last two modes are only for get mode 
#define NUM_CTCSS_DCS_MODES 7
#define NUM_REP_SHIFT       3

/***********************
 * CAT REQUESTS VALUES *
 ***********************/

/* CAT CMD Values */
#define CMD_CLAR_FREQUENCY     0xF5
#define CMD_CLAR_ON            0x05
#define CMD_CLAR_OFF           0x85
#define CMD_CTCSS_DCS_MODE     0x0A
#define CMD_CTCSS_TONE         0x0B
#define CMD_DCS_CODE           0x0C
#define CMD_LOCK_ON            0x00
#define CMD_LOCK_OFF           0x80
#define CMD_PTT_ON             0x08
#define CMD_PTT_OFF            0x88
#define CMD_REPEATER_SHIFT     0x09
#define CMD_REPEATER_FREQUENCY 0xF9
#define CMD_SET_FREQUENCY      0x01
#define CMD_SET_MODE           0x07
#define CMD_SPLIT_ON           0x02
#define CMD_SPLIT_OFF          0x82
#define CMD_STATUS_RX          0xE7
#define CMD_STATUS_TX          0xF7
#define CMD_STATUS             0x03
#define CMD_TOGGLE_VFO         0x81
/* Extended CAT CMD Values*/
#define E_CMD_RIG_CONFIG       0xA7
#define E_CMD_R_EEPROM         0xBB
#define E_CMD_W_EEPROM         0xBC
#define E_CMD_TX_METERS        0xBD
#define E_CMD_FACTORY_RESET    0xBE
#define E_CMD_TURN_ON          0x0F // Only for FT817
#define E_CMD_TURN_OFF         0x8F // Only for FT817
#define E_CMD_TX_KEYED_STATE   0x10 // Only for FT817
/* Operation Mode Values */
#define P1_MODE_LSB 0x00
#define P1_MODE_USB 0x01
#define P1_MODE_CW  0x02
#define P1_MODE_CWR 0x03
#define P1_MODE_AM  0x04
#define P1_MODE_FM  0x08
#define P1_MODE_DIG 0x0A
#define P1_MODE_PKT 0x0C
#define P1_MODE_NFM 0x88
#define P1_MODE_WFM 0x06 // Never set WFM mode. Only get mode value
#define P1_MODE_CWN 0x82 // Never set CWN mode. Only get mode value
/* CTCSS - DCS Mode Values */
#define P1_CTCSSDCS_OFF 0x8A
#define P1_DCS          0x0A
#define P1_DCS_DEC      0x0B
#define P1_DCS_ENC      0x0C
#define P1_CTCSS        0x2A
#define P1_CTCSS_DEC    0x3A
#define P1_CTCSS_ENC    0x4A
/* Clarifier Values */
#define P1_CLAR_OFFSET_POS 0x00
#define P1_CLAR_OFFSET_NEG 0xFF
/* Repeater Shift Values */
#define P1_REP_SHIFT_POS 0x49
#define P1_REP_SHIFT_NEG 0x09
#define P1_REP_SIMPLEX   0x89


/***********
 * STRINGS *
 ***********/
/* enum op_mode */
#define STR_OP_MODE_LSB "LSB"
#define STR_OP_MODE_USB "USB"
#define STR_OP_MODE_CW  "CW"
#define STR_OP_MODE_CWR "CWR"
#define STR_OP_MODE_AM  "AM"
#define STR_OP_MODE_FM  "FM"
#define STR_OP_MODE_DIG "DIG"
#define STR_OP_MODE_PKT "PKT"
#define STR_OP_MODE_NFM "NFM"
#define STR_OP_MODE_WFM "WFM"
#define STR_OP_MODE_CWN "CWN"
/* enum repeater_shift */
#define STR_REPEATER_SIMPLEX  "SIMPLEX"
#define STR_REPEATER_NEGATIVE "NEGATIVE"
#define STR_REPEATER_POSITIVE "POSITIVE"
/* enum selective_call */
#define STR_SELECTIVE_CALL_OFF       "OFF"
#define STR_SELECTIVE_CALL_DCS       "DCS"
#define STR_SELECTIVE_CALL_DCS_DEC   "DCS-DEC"
#define STR_SELECTIVE_CALL_DCS_ENC   "DCS-ENC"
#define STR_SELECTIVE_CALL_CTCSS     "CTCSS"
#define STR_SELECTIVE_CALL_CTCSS_DEC "CTCSS-DEC"
#define STR_SELECTIVE_CALL_CTCSS_ENC "CTCSS-ENC"
/* enum cat_nack */
#define STR_CAT_NACK_DEFAULT           "Command not successful"
#define STR_CAT_NACK_ALREADY_LOCKED    "Already locked"
#define STR_CAT_NACK_ALREADY_UNLOCKED  "Already unlocked"
#define STR_CAT_NACK_SPLIT_ALREADY_ON  "Split already ON"
#define STR_CAT_NACK_SPLIT_ALREADY_OFF "Split already OFF"
#define STR_CAT_NACK_CLAR_ALREADY_ON   "Clarifier already ON"
#define STR_CAT_NACK_CLAR_ALREADY_OFF  "Clarifier already OFF"
#define STR_CAT_NACK_PTT_ALREADY_ON    "PTT already keyed"
#define STR_CAT_NACK_PTT_ALREADY_OFF   "PTT already unkeyed"
#define STR_CAT_NACK_SELEC_CALL_NOT_FM "CTCSS/DCS only works on FM and PKT modes"
#define STR_CAT_NACK_EEPROM_ADR        "Wrong EEPROM Adress"
/* enum cat_errno */
#define STR_CAT_ERR_INIT                ""
#define STR_CAT_ERR_MALLOC              "Memory not allocated"
#define STR_CAT_ERR_BITRATE             "Invalid bitrate"
#define STR_CAT_ERR_DEVICE              "Device not found"
#define STR_CAT_ERR_NULL_DEVICE_FD      "Null device file descriptor"
#define STR_CAT_ERR_RTS                 "RTS (hardware PTT output) error"
#define STR_CAT_ERR_CTS                 "CTS (hardware SQL input) error"
#define STR_CAT_ERR_NULL_REQUEST        "Null request"
#define STR_CAT_ERR_W_REQUEST           "Error writing request"
#define STR_CAT_ERR_R_RESPONSE          "Error reading response"
#define STR_CAT_ERR_SELECT              "Select() error"
#define STR_CAT_ERR_TRANSACTION_TIMEOUT "Transaction timeout (no response)"
#define STR_CAT_ERR_NACK                "NACK response"
#define STR_CAT_ERR_FREQUENCY_OVERFLOW  "Invalid input (Frequency overflow)"
#define STR_CAT_ERR_SELECTIVE_CALL      "Invalid input (Selective Call Mode)"
#define STR_CAT_ERR_CTCSS_TONE          "Invalid input (CTCSS Tone)"
#define STR_CAT_ERR_DCS_CODE            "Invalid input (DCS Code)"
#define STR_CAT_ERR_REPEATER_SHIFT      "Invalid input (Repeater Shift)"
#define STR_CAT_ERR_OP_MODE             "Invalid input (Op Mode)"
#define STR_CAT_ERR_EEPROM_BLOCK_SIZE   "Invalid size (less than 2 bytes)"
#define STR_CAT_ERR_UNKNOWN             "Unknown error"

#define BCD_TO_UINT8(x) ((((x)>>4)*10)+((x)&0x0F))

#define UINT32_TO_BCD(x) (((((x)/10000000UL)<<28)) | \
			  (((((x)%10000000UL)/1000000UL)<<24)) | \
			  (((((x)%1000000UL)/100000UL)<<20)) | \
			  (((((x)%100000UL)/10000UL)<<16)) | \
			  (((((x)%10000UL)/1000UL)<<12)) | \
                          (((((x)%1000UL)/100UL)<<8)) | \
                          (((((x)%100UL)/10UL)<<4)) | \
			  ((x)%10UL))


/*******************************
 * STATIC FUNCTIONS PROTOTYPES *
 *******************************/
static void set_g_cat_nack(uint8_t cmd);
static size_t expected_response_size(uint8_t cmd);


/********************
 * STATIC CONSTANTS *
 ********************/

static const struct CAT_request TOOGLE_VFO_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_TOGGLE_VFO
};
static const struct CAT_request SWITCH_LOCK_OFF_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_LOCK_OFF
};
static const struct CAT_request SWITCH_LOCK_ON_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_LOCK_ON
};
static const struct CAT_request SWITCH_PTT_OFF_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_PTT_OFF
};
static const struct CAT_request SWITCH_PTT_ON_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_PTT_ON
};
static const struct CAT_request SWITCH_SPLIT_OFF_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_SPLIT_OFF
};
static const struct CAT_request SWITCH_SPLIT_ON_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_SPLIT_ON
};
static const struct CAT_request SWITCH_CLAR_OFF_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_CLAR_OFF
};
static const struct CAT_request SWITCH_CLAR_ON_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_CLAR_ON
};
static const struct CAT_request GET_STATUS_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_STATUS
};
static const struct CAT_request GET_STATUS_RX_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_STATUS_RX
};
static const struct CAT_request GET_STATUS_TX_REQUEST = {
	0x00, 0x00, 0x00, 0x00, CMD_STATUS_TX
};
static const struct CAT_request TURN_RIG_OFF_REQUEST = {
	0x00, 0x00, 0x00, 0x00, E_CMD_TURN_OFF
};
static const struct CAT_request TURN_RIG_ON_REQUEST = {
	0x00, 0x00, 0x00, 0x00, E_CMD_TURN_ON
};
static const struct CAT_request GET_RIG_CONFIG_REQUEST = {
	0x00, 0x00, 0x00, 0x00, E_CMD_RIG_CONFIG
};
static const struct CAT_request GET_TX_KEYED_STATE_REQUEST = {
	0x00, 0x00, 0x00, 0x00, E_CMD_TX_KEYED_STATE
};
static const struct CAT_request GET_TX_METERS_REQUEST = {
	0x00, 0x00, 0x00, 0x00, E_CMD_TX_METERS
};
static const struct CAT_request FACTORY_RESET_REQUEST = {
	0x00, 0x00, 0x00, 0x00, E_CMD_FACTORY_RESET
};
static const struct CAT_request SET_CTCSS_DCS_MODE_REQUEST[NUM_CTCSS_DCS_MODES] = {
	{P1_CTCSSDCS_OFF, 0x00, 0x00, 0x00, CMD_CTCSS_DCS_MODE},
	{P1_DCS, 0x00, 0x00, 0x00, CMD_CTCSS_DCS_MODE},
	{P1_DCS_DEC, 0x00, 0x00, 0x00, CMD_CTCSS_DCS_MODE},
	{P1_DCS_ENC, 0x00, 0x00, 0x00, CMD_CTCSS_DCS_MODE},
	{P1_CTCSS, 0x00, 0x00, 0x00, CMD_CTCSS_DCS_MODE},
	{P1_CTCSS_DEC, 0x00, 0x00, 0x00, CMD_CTCSS_DCS_MODE},
	{P1_CTCSS_ENC, 0x00, 0x00, 0x00, CMD_CTCSS_DCS_MODE}
};
static const struct CAT_request SET_REPEATER_SHIFT_REQUEST[NUM_REP_SHIFT] = {
	{P1_REP_SIMPLEX, 0x00, 0x00, 0x00, CMD_REPEATER_SHIFT},
	{P1_REP_SHIFT_NEG, 0x00, 0x00, 0x00, CMD_REPEATER_SHIFT},
	{P1_REP_SHIFT_POS, 0x00, 0x00, 0x00, CMD_REPEATER_SHIFT}
};
static const struct CAT_request SET_MODE_REQUEST[NUM_OP_MODES_SET] = {
	{P1_MODE_LSB, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_USB, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_CW, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_CWR, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_AM, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_FM, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_DIG, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_PKT, 0x00, 0x00, 0x00, CMD_SET_MODE},
	{P1_MODE_NFM, 0x00, 0x00, 0x00, CMD_SET_MODE}
};

/********************
 * GLOBAL CONSTANTS *
 *******************/

/* Standard CTSS Tone Frequency
 * Values in Hz x10 (standard values x 10) */
const CAT_decihertz CAT_STD_CTCSS_TONES[CAT_MAX_STD_CTCSS] = {
	670, 693, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974,
	1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273, 1318, 1365,
	1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679, 1713, 1738,
	1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995, 2035, 2065,
	2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541
};

/* Standard DCS Code */
const unsigned short CAT_STD_DCS_CODES[CAT_MAX_STD_DCS] = {
	23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72,
	73, 74, 114, 115, 116, 122, 125, 131, 132, 134, 143, 145, 152, 155,
	156, 162, 165, 172, 174, 205, 212, 223, 225, 226, 243, 244, 245, 246,
	251, 252, 255, 261, 263, 265, 266, 271, 274, 306, 311, 315, 325, 331,
	332, 343, 346, 351, 356, 364, 365, 371, 411, 412, 413, 423, 431, 432,
	445, 446, 452, 454, 455, 462, 464, 465, 466, 503, 506, 516, 523, 526,
	532, 546, 565, 606, 612, 624, 627, 631, 632, 654, 662, 664, 703, 712,
	723, 731, 732, 734, 743, 754
};

/********************
 * GLOBAL VARIABLES *
 ********************/
enum CAT_errno g_cat_errno = CAT_ERR_INIT;
enum CAT_nack g_cat_nack = CAT_NACK_ERR;

/******************
 * Enum functions *
 ******************/

/* Enum op_mode */
enum CAT_op_mode CAT_op_mode_from_str(char* str)
{
	if (0 == strcmp(str, STR_OP_MODE_LSB)) {
		return CAT_OP_MODE_LSB;
	} else if (0 == strcmp(str, STR_OP_MODE_USB)) {
		return CAT_OP_MODE_USB;
	} else if (0 == strcmp(str, STR_OP_MODE_CW)) {
		return CAT_OP_MODE_CW;
	} else if (0 == strcmp(str, STR_OP_MODE_CWR)) {
		return CAT_OP_MODE_CWR;
	} else if (0 == strcmp(str, STR_OP_MODE_AM)) {
		return CAT_OP_MODE_AM;
	} else if (0 == strcmp(str, STR_OP_MODE_FM)) {
		return CAT_OP_MODE_FM;
	} else if (0 == strcmp(str, STR_OP_MODE_DIG)) {
		return CAT_OP_MODE_DIG;
	} else if (0 == strcmp(str, STR_OP_MODE_PKT)) {
		return CAT_OP_MODE_PKT;
	} else if (0 == strcmp(str, STR_OP_MODE_NFM)) {
		return CAT_OP_MODE_NFM;
	} else if (0 == strcmp(str, STR_OP_MODE_WFM)) {
		return CAT_OP_MODE_WFM;
	} else if (0 == strcmp(str, STR_OP_MODE_CWN)) {
		return CAT_OP_MODE_CWN;
	} else {
		return CAT_OP_MODE_ERR;
	}
}

const char* CAT_op_mode_to_str(enum CAT_op_mode mode)
{
	switch (mode) {
	case CAT_OP_MODE_LSB: return STR_OP_MODE_LSB;
	case CAT_OP_MODE_USB: return STR_OP_MODE_USB;
	case CAT_OP_MODE_CW: return STR_OP_MODE_CW;
	case CAT_OP_MODE_CWR: return STR_OP_MODE_CWR;
	case CAT_OP_MODE_AM: return STR_OP_MODE_AM;
	case CAT_OP_MODE_FM: return STR_OP_MODE_FM;
	case CAT_OP_MODE_DIG: return STR_OP_MODE_DIG;
	case CAT_OP_MODE_PKT: return STR_OP_MODE_PKT;
	case CAT_OP_MODE_NFM: return STR_OP_MODE_NFM;
	case CAT_OP_MODE_WFM: return STR_OP_MODE_WFM;
	case CAT_OP_MODE_CWN: return STR_OP_MODE_CWN;
	default: return NULL;
	}
}

/* Enum selective_call */
enum CAT_selective_call CAT_selective_call_from_str(char* str)
{
	if (0 == strcmp(str, STR_SELECTIVE_CALL_OFF)) {
		return CAT_SELEC_CALL_OFF;
	} else if (0 == strcmp(str, STR_SELECTIVE_CALL_DCS)) {
		return CAT_SELEC_CALL_DCS;
	} else if (0 == strcmp(str, STR_SELECTIVE_CALL_DCS_ENC)) {
		return CAT_SELEC_CALL_DCS_DEC;
	} else if (0 == strcmp(str, STR_SELECTIVE_CALL_DCS_DEC)) {
		return CAT_SELEC_CALL_DCS_ENC;
	} else if (0 == strcmp(str, STR_SELECTIVE_CALL_CTCSS)) {
		return CAT_SELEC_CALL_CTCSS;
	} else if (0 == strcmp(str, STR_SELECTIVE_CALL_CTCSS_DEC)) {
		return CAT_SELEC_CALL_CTCSS_DEC;
	} else if (0 == strcmp(str, STR_SELECTIVE_CALL_CTCSS_ENC)) {
		return CAT_SELEC_CALL_CTCSS_ENC;
	} else {
		return CAT_SELEC_CALL_ERR;
	}
}

const char* CAT_selective_call_to_str(enum CAT_selective_call mode)
{
	switch (mode) {
	case CAT_SELEC_CALL_OFF: return STR_SELECTIVE_CALL_OFF;
	case CAT_SELEC_CALL_DCS: return STR_SELECTIVE_CALL_DCS;
	case CAT_SELEC_CALL_DCS_DEC: return STR_SELECTIVE_CALL_DCS_ENC;
	case CAT_SELEC_CALL_DCS_ENC: return STR_SELECTIVE_CALL_DCS_DEC;
	case CAT_SELEC_CALL_CTCSS: return STR_SELECTIVE_CALL_CTCSS;
	case CAT_SELEC_CALL_CTCSS_DEC: return STR_SELECTIVE_CALL_CTCSS_DEC;
	case CAT_SELEC_CALL_CTCSS_ENC: return STR_SELECTIVE_CALL_CTCSS_ENC;
	default: return NULL;
	}
}

/* Enum repeater_shift */
enum CAT_repeater_shift CAT_repeater_shift_from_str(char* str)
{
	if (0 == strcmp(str, STR_REPEATER_SIMPLEX)) {
		return CAT_REPEATER_SIMPLEX;
	} else if (0 == strcmp(str, STR_REPEATER_NEGATIVE)) {
		return CAT_REPEATER_NEGATIVE;
	} else if (0 == strcmp(str, STR_REPEATER_POSITIVE)) {
		return CAT_REPEATER_POSITIVE;
	} else {
		return CAT_REPEATER_ERR;
	}
}

const char* CAT_repeater_shift_to_str(enum CAT_repeater_shift mode)
{
	switch (mode) {
	case CAT_REPEATER_SIMPLEX: return STR_REPEATER_SIMPLEX;
	case CAT_REPEATER_NEGATIVE: return STR_REPEATER_NEGATIVE;
	case CAT_REPEATER_POSITIVE: return STR_REPEATER_POSITIVE;
	default: return NULL;
	}
}

char* CAT_decahertz_to_khz_str(CAT_decahertz frequency)
{
	static __thread char buf[20];

	if (frequency < 0) return NULL;

	snprintf(buf, sizeof(buf), "%02.f kHz", (float) frequency / 100);
	return buf;

}

char* CAT_decahertz_to_mhz_str(CAT_decahertz frequency)
{
	static __thread char buf[20];

	if (frequency < 0) return NULL;

	snprintf(buf, sizeof(buf), "%02.f MHz", (float) frequency / 100000);
	return buf;

}

/**********************
 * YAESU CAT REQUESTS *
 **********************/

struct CAT_request* CAT_request_dup(struct CAT_request* src)
{
	void* p = malloc(sizeof(struct CAT_request));
	if (!p) {
		g_cat_errno = CAT_ERR_MALLOC;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	}
	memcpy(p, src, sizeof(struct CAT_request));
	return(struct CAT_request*) p;
}

struct CAT_request*
CAT_toggle_vfo_request(void)
{
	return(struct CAT_request*) &TOOGLE_VFO_REQUEST;
}

struct CAT_request*
CAT_switch_lock_request(bool on_off)
{
	if (on_off == false) {
		return(struct CAT_request*) &SWITCH_LOCK_OFF_REQUEST;
	} else {
		return(struct CAT_request*) &SWITCH_LOCK_ON_REQUEST;
	}
}

struct CAT_request*
CAT_switch_ptt_request(bool on_off)
{
	if (on_off == false) {
		return(struct CAT_request*) &SWITCH_PTT_OFF_REQUEST;
	} else {
		return(struct CAT_request*) &SWITCH_PTT_ON_REQUEST;
	}
}

struct CAT_request*
CAT_switch_split_request(bool on_off)
{
	if (on_off == false) {
		return(struct CAT_request*) &SWITCH_SPLIT_OFF_REQUEST;
	} else {
		return(struct CAT_request*) &SWITCH_SPLIT_ON_REQUEST;
	}
}

struct CAT_request*
CAT_switch_clarifier_request(bool on_off)
{
	if (on_off == false) {
		return(struct CAT_request*) &SWITCH_CLAR_OFF_REQUEST;
	} else {
		return(struct CAT_request*) &SWITCH_CLAR_ON_REQUEST;
	}
}

struct CAT_request*
CAT_set_clarifier_frequency_request(CAT_decahertz frequency)
{
	static __thread struct CAT_request SET_CLAR_FREQUENCY_REQUEST = {
		0x00, 0x00, 0x00, 0x00, CMD_CLAR_FREQUENCY
	};
	if (frequency < -9999 && frequency > 9999) {
		g_cat_errno = CAT_ERR_FREQUENCY_OVERFLOW;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	}
	if (frequency < 0) {
		SET_CLAR_FREQUENCY_REQUEST.P[0] = P1_CLAR_OFFSET_NEG;
		frequency = -frequency;
	} else {
		SET_CLAR_FREQUENCY_REQUEST.P[0] = P1_CLAR_OFFSET_POS;
	}
	frequency = UINT32_TO_BCD(frequency);
	SET_CLAR_FREQUENCY_REQUEST.P[2] = (frequency & 0x0000FF00) >> 8;
	SET_CLAR_FREQUENCY_REQUEST.P[3] = frequency & 0x00FF;
	return(struct CAT_request*) &SET_CLAR_FREQUENCY_REQUEST;
}

struct CAT_request*
CAT_set_ctcss_dcs_mode_request(enum CAT_selective_call mode)
{
	DEBUG_PLACE;
	if (mode < 0 && mode >= NUM_CTCSS_DCS_MODES) {
		g_cat_errno = CAT_ERR_SELECTIVE_CALL;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	} else {
		return(struct CAT_request*) &SET_CTCSS_DCS_MODE_REQUEST[mode];
	}
}

struct CAT_request*
CAT_set_ctcss_tone_request(CAT_decihertz tone_tx, CAT_decihertz tone_rx)
{
	static __thread struct CAT_request SET_CTCSS_TONE_REQUEST = {
		0x00, 0x00, 0x00, 0x00, CMD_CTCSS_TONE
	};
	int ok_tx = CAT_check_ctcss_tone(tone_tx);
	int ok_rx = CAT_check_ctcss_tone(tone_rx);
	if (ok_tx == 1 && ok_rx == 1) {
		tone_tx = UINT32_TO_BCD(tone_tx);
		tone_rx = UINT32_TO_BCD(tone_rx);
		SET_CTCSS_TONE_REQUEST.P[0] = (tone_tx & 0x0000FF00) >> 8;
		SET_CTCSS_TONE_REQUEST.P[1] = tone_tx & 0x00FF;
		SET_CTCSS_TONE_REQUEST.P[2] = (tone_rx & 0x0000FF00) >> 8;
		SET_CTCSS_TONE_REQUEST.P[3] = tone_rx & 0x00FF;
		return(struct CAT_request*) &SET_CTCSS_TONE_REQUEST;
	} else {
		g_cat_errno = CAT_ERR_CTCSS_TONE;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	}
}

struct CAT_request*
CAT_set_dcs_code_request(int code_tx, int code_rx)
{
	static __thread struct CAT_request SET_DCS_CODE_REQUEST = {
		0x00, 0x00, 0x00, 0x00, CMD_DCS_CODE
	};
	int ok_tx = CAT_check_dcs_code(code_tx);
	int ok_rx = CAT_check_dcs_code(code_rx);
	if (ok_tx == 1 && ok_rx == 1) {
		code_tx = UINT32_TO_BCD(code_tx);
		code_rx = UINT32_TO_BCD(code_rx);
		SET_DCS_CODE_REQUEST.P[0] = (code_tx & 0x0000FF00) >> 8;
		SET_DCS_CODE_REQUEST.P[1] = code_tx & 0x00FF;
		SET_DCS_CODE_REQUEST.P[2] = (code_rx & 0x0000FF00) >> 8;
		SET_DCS_CODE_REQUEST.P[3] = code_rx & 0x00FF;
		return(struct CAT_request*) &SET_DCS_CODE_REQUEST;
	} else {
		g_cat_errno = CAT_ERR_DCS_CODE;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	}
}

struct CAT_request*
CAT_set_repeater_shift_request(enum CAT_repeater_shift shift)
{
	if (shift < 0 && shift >= NUM_REP_SHIFT) {
		g_cat_errno = CAT_ERR_REPEATER_SHIFT;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	} else {
		return(struct CAT_request*) &SET_REPEATER_SHIFT_REQUEST[shift];
	}
}

struct CAT_request*
CAT_set_repeater_offset_request(CAT_hertz frequency)
{
	static __thread struct CAT_request SET_REPEATER_OFFSET_REQUEST = {
		0x00, 0x00, 0x00, 0x00, CMD_REPEATER_FREQUENCY
	};
	if (frequency < 0 && frequency > 99999999) {
		g_cat_errno = CAT_ERR_FREQUENCY_OVERFLOW;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	}
	frequency = UINT32_TO_BCD(frequency);
	SET_REPEATER_OFFSET_REQUEST.P[0] = frequency >> 24;
	SET_REPEATER_OFFSET_REQUEST.P[1] = (frequency & 0x00FF0000) >> 16;
	SET_REPEATER_OFFSET_REQUEST.P[2] = (frequency & 0x0000FF00) >> 8;
	SET_REPEATER_OFFSET_REQUEST.P[3] = frequency & 0x000000FF;
	return(struct CAT_request*) &SET_REPEATER_OFFSET_REQUEST;
}

struct CAT_request*
CAT_set_frequency_request(CAT_decahertz frequency)
{
	static __thread struct CAT_request SET_FREQUENCY_REQUEST = {
		0x00, 0x00, 0x00, 0x00, CMD_SET_FREQUENCY
	};
	if (frequency < 0 && frequency > 99999999) {
		g_cat_errno = CAT_ERR_FREQUENCY_OVERFLOW;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	}
	frequency = UINT32_TO_BCD(frequency);
	SET_FREQUENCY_REQUEST.P[0] = frequency >> 24;
	SET_FREQUENCY_REQUEST.P[1] = (frequency & 0x00FF0000) >> 16;
	SET_FREQUENCY_REQUEST.P[2] = (frequency & 0x0000FF00) >> 8;
	SET_FREQUENCY_REQUEST.P[3] = frequency & 0x000000FF;
	return(struct CAT_request*) &SET_FREQUENCY_REQUEST;
}

struct CAT_request*
CAT_set_mode_request(enum CAT_op_mode mode)
{
	if (mode < 0 && mode >= NUM_OP_MODES_SET) {
		g_cat_errno = CAT_ERR_OP_MODE;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return NULL;
	} else {
		return(struct CAT_request*) &SET_MODE_REQUEST[mode];
	}
}

struct CAT_request*
CAT_get_frequency_mode_request(void)
{
	return(struct CAT_request*) &GET_STATUS_REQUEST;
}

struct CAT_request*
CAT_get_status_rx_request(void)
{
	return(struct CAT_request*) &GET_STATUS_RX_REQUEST;
}

struct CAT_request*
CAT_get_status_tx_request(void)
{
	return(struct CAT_request*) &GET_STATUS_TX_REQUEST;
}

/* Extended CAT Requests: Officially UNDOCUMENTED
 * Be carefull using this commands */

struct CAT_request*
CAT_write_eeprom_request(uint16_t adress, uint8_t data_adr, uint8_t data_next_adr)
{
	static __thread struct CAT_request WRITE_EEPROM_REQUEST = {
		0x00, 0x00, 0x00, 0x00, E_CMD_W_EEPROM
	};
	WRITE_EEPROM_REQUEST.P[0] = (adress & 0xFF00) >> 8;
	WRITE_EEPROM_REQUEST.P[1] = adress & 0x00FF;
	WRITE_EEPROM_REQUEST.P[2] = data_adr;
	WRITE_EEPROM_REQUEST.P[3] = data_next_adr;
	return(struct CAT_request*) &WRITE_EEPROM_REQUEST;
}

struct CAT_request*
CAT_read_eeprom_request(uint16_t adress)
{
	static __thread struct CAT_request READ_EEPROM_REQUEST = {
		0x00, 0x00, 0x00, 0x00, E_CMD_R_EEPROM
	};
	READ_EEPROM_REQUEST.P[0] = (adress & 0xFF00) >> 8;
	READ_EEPROM_REQUEST.P[1] = adress & 0x00FF;
	return(struct CAT_request*) &READ_EEPROM_REQUEST;
}

struct CAT_request*
CAT_turn_radio_request(bool on_off)
{
	if (on_off == false) {
		return(struct CAT_request*) &TURN_RIG_OFF_REQUEST;
	} else {
		return(struct CAT_request*) &TURN_RIG_ON_REQUEST;
	}
}

struct CAT_request*
CAT_get_rig_config_request(void)
{
	return(struct CAT_request*) &GET_RIG_CONFIG_REQUEST;
}

struct CAT_request*
CAT_get_tx_keyed_state_request(void)
{
	return(struct CAT_request*) &GET_TX_KEYED_STATE_REQUEST;
}

struct CAT_request*
CAT_get_tx_meters_request(void)
{
	return(struct CAT_request*) &GET_TX_METERS_REQUEST;
}

struct CAT_request*
CAT_factory_reset_request(void)
{
	return(struct CAT_request*) &FACTORY_RESET_REQUEST;
}

/*****************
 * CAT RESPONSES *
 *****************/

int
CAT_check_response(struct CAT_response* resp)
{
	if (resp == NULL || resp->len == 0) {
		return -1;
	} else if (resp->len == 1 && resp->frame[0] == CAT_NACK) {
		return 0;
	} else {
		return 1;
	}
}

enum CAT_op_mode
CAT_decode_mode_response(struct CAT_response* resp)
{
	if (resp->len == 5) {
		switch (resp->frame[4]) {
		case P1_MODE_LSB: return CAT_OP_MODE_LSB;
		case P1_MODE_USB: return CAT_OP_MODE_USB;
		case P1_MODE_CW: return CAT_OP_MODE_CW;
		case P1_MODE_CWR: return CAT_OP_MODE_CWR;
		case P1_MODE_AM: return CAT_OP_MODE_AM;
		case P1_MODE_FM: return CAT_OP_MODE_FM;
		case P1_MODE_DIG: return CAT_OP_MODE_DIG;
		case P1_MODE_PKT: return CAT_OP_MODE_PKT;
		case P1_MODE_NFM: return CAT_OP_MODE_NFM;
		case P1_MODE_WFM: return CAT_OP_MODE_WFM;
		case P1_MODE_CWN: return CAT_OP_MODE_CWN;
		default: return CAT_OP_MODE_ERR;
		}
	} else {
		return CAT_OP_MODE_ERR;
	}
}

CAT_decahertz
CAT_decode_frequency_response(struct CAT_response* resp)
{
	if (resp->len == 5) {
		return(BCD_TO_UINT8(resp->frame[0]) * 1000000UL) +
			(BCD_TO_UINT8(resp->frame[1]) * 10000UL) +
			(BCD_TO_UINT8(resp->frame[2]) * 100UL) +
			BCD_TO_UINT8(resp->frame[3]);
	} else {
		return -1;
	}
}

struct CAT_status_rx
CAT_decode_status_rx_response(struct CAT_response* resp)
{
	return *(struct CAT_status_rx*) resp->frame;
}

struct CAT_status_tx
CAT_decode_status_tx_response(struct CAT_response* resp)
{
	return *(struct CAT_status_tx*) resp->frame;
}

struct CAT_tx_meters
CAT_decode_tx_meters_response(struct CAT_response* resp)
{
	return *(struct CAT_tx_meters*) resp->frame;
}

bool CAT_decode_eeprom_response(struct CAT_eeprom_byte* dest, struct CAT_response* resp)
{
	if (resp->len == 2) {
		dest->data = resp->frame[0];
		dest->data_next = resp->frame[1];
		return true;
	} else {
		return false;
	}
}

enum CAT_cmd CAT_cmd_id_from_request(struct CAT_request* request)
{
	switch (request->CMD) {
	case CMD_CLAR_FREQUENCY: return CAT_CMD_SET_CLAR_FREQUENCY;
	case CMD_CLAR_OFF: return CAT_CMD_SET_CLAR_OFF;
	case CMD_CLAR_ON: return CAT_CMD_SET_CLAR_ON;
	case CMD_CTCSS_DCS_MODE: return CAT_CMD_SET_CTCSS_DCS_MODE;
	case CMD_CTCSS_TONE: return CAT_CMD_SET_CTCSS_TONE;
	case CMD_DCS_CODE: return CAT_CMD_SET_DCS_CODE;
	case CMD_LOCK_OFF: return CAT_CMD_SET_LOCK_OFF;
	case CMD_LOCK_ON: return CAT_CMD_SET_LOCK_ON;
	case CMD_PTT_OFF: return CAT_CMD_SET_PTT_OFF;
	case CMD_PTT_ON: return CAT_CMD_SET_PTT_ON;
	case CMD_REPEATER_FREQUENCY: return CAT_CMD_SET_REPEATER_FREQUENCY;
	case CMD_REPEATER_SHIFT: return CAT_CMD_SET_REPEATER_SHIFT;
	case CMD_SET_FREQUENCY: return CAT_CMD_SET_FREQUENCY;
	case CMD_SET_MODE: return CAT_CMD_SET_OP_MODE;
	case CMD_SPLIT_OFF: return CAT_CMD_SET_SPLIT_OFF;
	case CMD_SPLIT_ON: return CAT_CMD_SET_SPLIT_ON;
	case CMD_STATUS: return CAT_CMD_GET_FREQUENCY_AND_MODE;
	case CMD_STATUS_RX: return CAT_CMD_GET_STATUS_RX;
	case CMD_STATUS_TX: return CAT_CMD_GET_STATUS_TX;
	case CMD_TOGGLE_VFO: return CAT_CMD_TOOGLE_VFO;
	case E_CMD_FACTORY_RESET: return CAT_EXT_CMD_FACTORY_RESET;
	case E_CMD_RIG_CONFIG: return CAT_EXT_CMD_GET_RADIO_CONFIG;
	case E_CMD_R_EEPROM: return CAT_EXT_CMD_READ_EEPROM;
	case E_CMD_W_EEPROM: return CAT_EXT_CMD_WRITE_EEPROM;
	case E_CMD_TX_METERS: return CAT_EXT_CMD_GET_TX_METERS;
	case E_CMD_TURN_OFF: return CAT_EXT_CMD_TURN_OFF;
	case E_CMD_TURN_ON: return CAT_EXT_CMD_TURN_ON;
	case E_CMD_TX_KEYED_STATE: return CAT_EXT_CMD_TX_KEYED_STATE;
	default: return CAT_CMD_ERR;
	}
}

/*************************
 * CHECK VALUE FUNCTIONS *
 *************************/

int CAT_check_dev_rate(int bitrate)
{
	switch (bitrate) {
	case 4800: case 9600: case 38400:
		return 1;
	default:
		return 0;
	}
}

int CAT_check_ctcss_tone(unsigned short tone)
{
	int i = 0;
	while (tone != CAT_STD_CTCSS_TONES[i] && i < CAT_MAX_STD_CTCSS) {
		i++;
	}
	if (i <= CAT_MAX_STD_CTCSS) {
		return 1; //Standard CTCSS Tone
	} else {
		return 0; //Invalid CTCSS Tone
	}
}

int CAT_check_dcs_code(unsigned short code)
{
	int i = 0;
	while (code != CAT_STD_DCS_CODES[i] && i < CAT_MAX_STD_DCS) {
		i++;
	}
	if (i <= CAT_MAX_STD_CTCSS) {
		return 1; //Standard DCS Code
	} else {
		return 0; //Invalid DCS Code
	}
}

/******************
 * ERROR HANDLERS *
 ******************/

const char* CAT_errno_to_str(enum CAT_errno err)
{
	switch (err) {
	case CAT_ERR_INIT: return STR_CAT_ERR_INIT;
	case CAT_ERR_MALLOC: return STR_CAT_ERR_MALLOC;
	case CAT_ERR_BITRATE: return STR_CAT_ERR_BITRATE;
	case CAT_ERR_DEVICE: return STR_CAT_ERR_DEVICE;
	case CAT_ERR_NULL_DEVICE_FD: return STR_CAT_ERR_NULL_DEVICE_FD;
	case CAT_ERR_RTS: return STR_CAT_ERR_RTS;
	case CAT_ERR_CTS: return STR_CAT_ERR_CTS;
	case CAT_ERR_NULL_REQUEST: return STR_CAT_ERR_NULL_REQUEST;
	case CAT_ERR_W_REQUEST: return STR_CAT_ERR_W_REQUEST;
	case CAT_ERR_R_RESPONSE: return STR_CAT_ERR_R_RESPONSE;
	case CAT_ERR_SELECT: return STR_CAT_ERR_SELECT;
	case CAT_ERR_TRANSACTION_TIMEDOUT: return STR_CAT_ERR_TRANSACTION_TIMEOUT;
	case CAT_ERR_NACK: return STR_CAT_ERR_NACK;
	case CAT_ERR_FREQUENCY_OVERFLOW: return STR_CAT_ERR_FREQUENCY_OVERFLOW;
	case CAT_ERR_SELECTIVE_CALL: return STR_CAT_ERR_SELECTIVE_CALL;
	case CAT_ERR_CTCSS_TONE: return STR_CAT_ERR_CTCSS_TONE;
	case CAT_ERR_DCS_CODE: return STR_CAT_ERR_DCS_CODE;
	case CAT_ERR_REPEATER_SHIFT: return STR_CAT_ERR_REPEATER_SHIFT;
	case CAT_ERR_OP_MODE: return STR_CAT_ERR_OP_MODE;
	case CAT_ERR_EEPROM_BLOCK_SIZE: return STR_CAT_ERR_EEPROM_BLOCK_SIZE;
	default: return STR_CAT_ERR_UNKNOWN;
	}
}

char* CAT_nack_to_str(enum CAT_nack nack)
{
	switch (nack) {
	case CAT_NACK_ALREADY_LOCKED: return STR_CAT_NACK_ALREADY_LOCKED;
	case CAT_NACK_ALREADY_UNLOCKED: return STR_CAT_NACK_ALREADY_UNLOCKED;
	case CAT_NACK_SPLIT_ALREADY_ON: return STR_CAT_NACK_SPLIT_ALREADY_ON;
	case CAT_NACK_SPLIT_ALREADY_OFF: return STR_CAT_NACK_SPLIT_ALREADY_OFF;
	case CAT_NACK_CLAR_ALREADY_ON: return STR_CAT_NACK_CLAR_ALREADY_ON;
	case CAT_NACK_CLAR_ALREADY_OFF: return STR_CAT_NACK_CLAR_ALREADY_OFF;
	case CAT_NACK_PTT_ALREADY_ON: return STR_CAT_NACK_PTT_ALREADY_ON;
	case CAT_NACK_PTT_ALREADY_OFF: return STR_CAT_NACK_PTT_ALREADY_OFF;
	case CAT_NACK_SELEC_CALL_NOT_FM: return STR_CAT_NACK_SELEC_CALL_NOT_FM;
	case CAT_NACK_EEPROM_ADR: return STR_CAT_NACK_EEPROM_ADR;
	default: return STR_CAT_NACK_DEFAULT;
	}
}

static void
set_g_cat_nack(uint8_t cmd)
{
	switch (cmd) {
	case CMD_LOCK_ON: g_cat_nack = CAT_NACK_ALREADY_LOCKED;
		break;
	case CMD_LOCK_OFF: g_cat_nack = CAT_NACK_ALREADY_UNLOCKED;
		break;
	case CMD_SPLIT_ON: g_cat_nack = CAT_NACK_SPLIT_ALREADY_ON;
		break;
	case CMD_SPLIT_OFF: g_cat_nack = CAT_NACK_SPLIT_ALREADY_OFF;
		break;
	case CMD_CLAR_ON: g_cat_nack = CAT_NACK_CLAR_ALREADY_ON;
		break;
	case CMD_CLAR_OFF: g_cat_nack = CAT_NACK_CLAR_ALREADY_OFF;
		break;
	case CMD_PTT_ON: g_cat_nack = CAT_NACK_PTT_ALREADY_ON;
		break;
	case CMD_PTT_OFF: g_cat_nack = CAT_NACK_PTT_ALREADY_OFF;
		break;
	case CMD_CTCSS_DCS_MODE: g_cat_nack = CAT_NACK_SELEC_CALL_NOT_FM;
		break;
	case E_CMD_R_EEPROM: g_cat_nack = CAT_NACK_EEPROM_ADR;
		break;
	default: g_cat_nack = CAT_NACK_DEFAULT;
	}
}

/* EEPROM BLOCK */

struct CAT_eeprom_block* CAT_read_eeprom_block(
	struct CAT_device* dev,
	uint16_t start_adr,
	size_t len,
	void (*progress_callback)(float, void*),
	void* callback_args)
{
	struct CAT_eeprom_block* r = malloc(sizeof(struct CAT_eeprom_block));
	if (!r) return NULL;

	struct CAT_response response;
	bool odd;
	int i = 0;

	r->start_adr = start_adr;
	r->size = len;
	r->data = malloc(len * sizeof(uint8_t));
	if (!r->data) {
		g_cat_errno = CAT_ERR_MALLOC;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		CAT_free_eeprom_block(r);
		return NULL;
	}
	odd = len % 2;
	if (odd == true) {
		response = CAT_transaction(dev,
			CAT_read_eeprom_request(start_adr));
		if (CAT_check_response(&response) < 1) {
			CAT_free_eeprom_block(r);
			return NULL;
		}
		r->data[i] = response.frame[0];
		i++;
		start_adr++;
		if (progress_callback != NULL) {
			progress_callback((float) i / len, callback_args);
		}
	}
	for (i; i < len; i = i + 2) {
		response = CAT_transaction(dev,
			CAT_read_eeprom_request(start_adr));
		if (CAT_check_response(&response) < 1) {
			CAT_free_eeprom_block(r);
			return NULL;
		}
		start_adr = start_adr + 2;
		r->data[i] = response.frame[0];
		r->data[i + 1] = response.frame[1];
		if (progress_callback != NULL) {
			progress_callback((float) i / len, callback_args);
		}
	}
	if (progress_callback != NULL) {
		progress_callback((float) i / len, callback_args);
	}
	return r;
}

void
CAT_free_eeprom_block(struct CAT_eeprom_block* src)
{
	if (src != NULL) {
		free(src->data);
		free(src);
		src = NULL;
	}
}

int
CAT_write_eeprom_block(
	struct CAT_device* dev,
	struct CAT_eeprom_block* src,
	void (*progress_callback)(float, void*),
	void* callback_args)
{
	struct CAT_response response;
	if (src->size < 2) {
		g_cat_errno = CAT_ERR_EEPROM_BLOCK_SIZE;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		return 0;
	}
	bool odd = src->size % 2;
	int i = 0;
	if (odd == true) {
		response = CAT_transaction(dev, CAT_write_eeprom_request(
			src->start_adr, src->data[0], src->data[1]));
		i++;
		if (CAT_check_response(&response) < 1) {
			return 0;
		}
	}
	for (i; i < src->size; i = i + 2) {
		response = CAT_transaction(dev, CAT_write_eeprom_request(
			src->start_adr + i, src->data[i], src->data[i + 1]));
		if (CAT_check_response(&response) < 1) {
			return 0;
		}
	}
	return 1;
}

/* COMMON SERIAL FN */

struct CAT_response
CAT_transaction(struct CAT_device* dev, struct CAT_request* request)
{
	struct CAT_response r;
	size_t resp_size;
	uint8_t cmd;
	bool ok;

	/* Check request */
	if (request == NULL) {
		g_cat_errno = CAT_ERR_NULL_REQUEST;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		r.len = 0;
		return r; //Error
	}

	/* Check serial device */
	if (dev == NULL) {
		g_cat_errno = CAT_ERR_NULL_DEVICE_FD;
		DEBUG_PRINT("%s", CAT_errno_to_str(g_cat_errno));
		r.len = 0;
		return r; //Error
	}

	DEBUG_PLACE;
	DEBUG_PRINT("Request: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
		(uint8_t) request->P[0],
		(uint8_t) request->P[1],
		(uint8_t) request->P[2],
		(uint8_t) request->P[3],
		(uint8_t) request->CMD);


	cmd = request->CMD;

	ok = CAT_write_bytes(dev, (uint8_t*) request, CAT_REQUEST_SIZE);
	if (ok == false) {
		r.len = 0;
		return r; //Error
	}

	resp_size = expected_response_size(request->CMD);
	r.len = CAT_read_bytes(dev, r.frame, resp_size);

	/* Check ACK */
	if (r.len == 1 &&
		r.frame[0] == CAT_NACK) {
		g_cat_nack = CAT_ERR_NACK;
		set_g_cat_nack(cmd);
		DEBUG_PRINT("%s", CAT_nack_to_str(g_cat_nack));
	}

	return r;
}

static size_t expected_response_size(uint8_t cmd)
{
	switch (cmd) {
	case CMD_STATUS: return 5;
	case E_CMD_TX_METERS:
	case E_CMD_R_EEPROM: return 2;
	case E_CMD_RIG_CONFIG: return 9;
	default: return 1;
	}
}