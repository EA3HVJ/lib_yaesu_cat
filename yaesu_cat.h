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

/**
 * @file yaesu_cat.h
 * @author Joan Planella Costa - EA3HVJ
 * @date 22/06/2018
 * @version 0.9
 * @brief Yaesu CAT interface for FT8x7 series
 */

#ifndef YAESU_CAT_H
#define YAESU_CAT_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__unix__) || defined(_APPLE_) || (defined(_WIN32) && defined(__CYGWIN__))
#define POSIX_COMPILATION
#elif defined(_WIN32) && !defined(__CYGWIN__)
#error Compiling for Windows needs cygwin for POSIX compatibility layer
#elif defined(ARDUINO)
#define ARDUINO_COMPILATION
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

	/*******************
	 * CONSTANT VALUES *
	 *******************/

#define CAT_REQUEST_SIZE     5 /**< CAT frame: total size in bytes */
#define CAT_P_SIZE           4 /**< CAT frame: parameters size in bytes */
#define CAT_RESPONSE_SIZE    9 /**< Max length in bytes of a CAT response frame */
#define CAT_ACK              0x00 /**< CAT Acknowledgement for non data response */
#define CAT_NACK             0xF0 /**< CAT Negative acknowledgement */
#define CAT_MAX_STD_CTCSS    50 /**< Size of CTCSS Tones array */
#define CAT_MAX_STD_DCS      104 /**< Size of DCS Codes array */
#define CAT_EEPROM_DATA_SIZE 2 /**< Number of EEPROM bytes for R/W CAT transactions */
#define CAT_DEFAULT_TIMEOUT  200 /**< Milliseconds for transaction timeout */

	typedef int32_t CAT_decihertz; /**< Frequency unit: 32bits integer */
	typedef int32_t CAT_hertz; /**< Frequency unit: 32bits integer */
	typedef int32_t CAT_decahertz; /**< Frequency unit: 32bits integer */

	/*********
	 * ENUMS *
	 *********/

	/**
	 * @brief CAT error numbers
	 */
	enum CAT_errno {
		CAT_ERR_INIT, /**< Init value, no errors */
		CAT_ERR_MALLOC, /**< No memory, malloc() error */
		CAT_ERR_BITRATE, /**< CAT Bitrate not allowed */
		CAT_ERR_DEVICE, /**< No such device */
		CAT_ERR_NULL_DEVICE_FD, /**< Device file descriptor is NULL */
		CAT_ERR_RTS, /**< RTS (hardware PTT output) error */
		CAT_ERR_CTS, /**< CTS (hardware SQL input) error */
		CAT_ERR_NULL_REQUEST, /**< NULL Request to send */
		CAT_ERR_W_REQUEST, /**< Request not sent, write() error */
		CAT_ERR_R_RESPONSE, /**< Response not received, read() error */
		CAT_ERR_SELECT, /**< select() error */
		CAT_ERR_TRANSACTION_TIMEDOUT, /**< No response, timed out */
		CAT_ERR_NACK, /**< NACK response */
		/* requests */
		CAT_ERR_FREQUENCY_OVERFLOW,
		CAT_ERR_SELECTIVE_CALL,
		CAT_ERR_CTCSS_TONE,
		CAT_ERR_DCS_CODE,
		CAT_ERR_REPEATER_SHIFT,
		CAT_ERR_OP_MODE,
		/* other */
		CAT_ERR_EEPROM_BLOCK_SIZE
	};

	/**
	 * @brief CAT nack values
	 */

	enum CAT_nack {
		CAT_NACK_ERR,
		CAT_NACK_DEFAULT, /**< Invalid request */
		CAT_NACK_ALREADY_LOCKED, /**< Transceiver is already locked */
		CAT_NACK_ALREADY_UNLOCKED, /**< Transceiver is already unlocked */
		CAT_NACK_SPLIT_ALREADY_ON, /**< Split mode is already ON */
		CAT_NACK_SPLIT_ALREADY_OFF, /**< Split mode is already OFF */
		CAT_NACK_CLAR_ALREADY_ON, /**< Clarifier is already ON */
		CAT_NACK_CLAR_ALREADY_OFF, /**< Clarifier mode is already OFF */
		CAT_NACK_PTT_ALREADY_ON, /**< PTT is already ON */
		CAT_NACK_PTT_ALREADY_OFF, /**< PTT is already OFF */
		CAT_NACK_SELEC_CALL_NOT_FM, /**< Invalid op mode for CTCSS/DCS */
		CAT_NACK_EEPROM_ADR /**< Invalid EEPROM adress */
	};

	enum CAT_cmd {
		CAT_CMD_ERR = -1,
		CAT_CMD_SET_CLAR_FREQUENCY,
		CAT_CMD_SET_CLAR_ON,
		CAT_CMD_SET_CLAR_OFF,
		CAT_CMD_SET_CTCSS_DCS_MODE,
		CAT_CMD_SET_CTCSS_TONE,
		CAT_CMD_SET_DCS_CODE,
		CAT_CMD_SET_LOCK_ON,
		CAT_CMD_SET_LOCK_OFF,
		CAT_CMD_SET_PTT_ON,
		CAT_CMD_SET_PTT_OFF,
		CAT_CMD_SET_REPEATER_SHIFT,
		CAT_CMD_SET_REPEATER_FREQUENCY,
		CAT_CMD_SET_FREQUENCY,
		CAT_CMD_SET_OP_MODE,
		CAT_CMD_SET_SPLIT_ON,
		CAT_CMD_SET_SPLIT_OFF,
		CAT_CMD_TOOGLE_VFO,
		CAT_CMD_GET_STATUS_RX,
		CAT_CMD_GET_STATUS_TX,
		CAT_CMD_GET_FREQUENCY_AND_MODE,
		CAT_EXT_CMD_GET_RADIO_CONFIG,
		CAT_EXT_CMD_READ_EEPROM,
		CAT_EXT_CMD_WRITE_EEPROM,
		CAT_EXT_CMD_GET_TX_METERS,
		CAT_EXT_CMD_FACTORY_RESET,
		CAT_EXT_CMD_TURN_ON,
		CAT_EXT_CMD_TURN_OFF,
		CAT_EXT_CMD_TX_KEYED_STATE
	};

	/**
	 * @brief CAT Operation Mode enum
	 */
	enum CAT_op_mode {
		CAT_OP_MODE_ERR = -1, /**< OP MODE ERROR */
		CAT_OP_MODE_LSB, /**< LSB mode (Lower Side Band) */
		CAT_OP_MODE_USB, /**< USB mode, (Upper Side Band) */
		CAT_OP_MODE_CW, /**< CW mode, (Continuous Wave) (Morse)*/
		CAT_OP_MODE_CWR, /**< CWR mode, (Continuous Wave Reverse) (Morse) */
		CAT_OP_MODE_AM, /**< AM mode (Amplitude Modulation) */
		CAT_OP_MODE_FM, /**< FM mode (Frequency Modulation) */
		CAT_OP_MODE_DIG, /**< DIG mode (digital mode) */
		CAT_OP_MODE_PKT, /**< PKT mode (packet mode) */
		CAT_OP_MODE_NFM, /**< NFM mode (Narrow FM) */
		CAT_OP_MODE_WFM, /**< WFM mode (Wide FM) */
		CAT_OP_MODE_CWN /**< CWN mode (Continuous Wave Narrow) */
	};

	/**
	 * @brief CAT CTCSS/DCS Mode enum
	 */
	enum CAT_selective_call {
		CAT_SELEC_CALL_ERR = -1, /**< CTCSS/DCS MODE ERROR */
		CAT_SELEC_CALL_OFF, /**< Selective call (CTCSS/DCS) OFF */
		CAT_SELEC_CALL_DCS, /**< DCS ON (encoder & decoder) */
		CAT_SELEC_CALL_DCS_DEC, /**< DCS decoder ON */
		CAT_SELEC_CALL_DCS_ENC, /**< DCS encoder ON */
		CAT_SELEC_CALL_CTCSS, /**< CTCSS ON (encoder & decoder) */
		CAT_SELEC_CALL_CTCSS_DEC, /**< CTCSS decoder ON */
		CAT_SELEC_CALL_CTCSS_ENC /**< CTCSS encoder ON */
	};

	/**
	 * @brief CAT Repeater Shift enum
	 */
	enum CAT_repeater_shift {
		CAT_REPEATER_ERR = -1, /**< REPEATER SHIFT ERROR */
		CAT_REPEATER_SIMPLEX, /**< Repeater disabled */
		CAT_REPEATER_NEGATIVE, /**< Repeater negative shift */
		CAT_REPEATER_POSITIVE /**< Repeater positive shift */
	};

	/**
	 * @brief CAT serial device struct
	 */
	struct CAT_device;

	/**
	 * @brief CAT Request struct
	 */
	struct __attribute__((__packed__)) CAT_request
	{
		uint8_t P[CAT_P_SIZE]; /**< Parameters */
		const uint8_t CMD; /**< Command code */
	};

	/** 
	 * @brief CAT Response struct
	 */
	struct CAT_response {
		uint8_t len;
		uint8_t frame[CAT_RESPONSE_SIZE];
	};

	/**
	 * @brief CAT Status Rx response byte struct
	 */
	struct CAT_status_rx {
		uint8_t smeter : 4;
		uint8_t dummy : 1; /**< Ignore this bit */
		uint8_t disc_is_off_center : 1; /**< Discriminator centering: 0 center (or SSB/CW/AM), 1 off-center */
		uint8_t ctcss_dcs_is_unmatched : 1; /**< CTCSS/DCS: 0 matched/off, 1 un-matched */
		uint8_t squelch : 1; /**<  Squelch status */
	};

	/**
	 * @brief CAT Status Tx response byte struct
	 */
	struct CAT_status_tx {
		uint8_t pwr_meter : 4; /**< Power Meter */
		uint8_t dummy : 1; /**< Ignore this bit */
		uint8_t split_is_off : 1; /**< Split status */
		uint8_t hi_swr : 1; /**< High SWR status */
		uint8_t ptt_is_off : 1; /**< Push To Talk status (Rx/Tx) */
	};

	/* Extended CAT structs */

	/**
	 * @brief CAT EEPROM byte struct
	 */
	struct CAT_eeprom_byte {
		uint16_t adress; /**< EEPROM adress */
		uint8_t data; /**< Data in that adress */
		uint8_t data_next; /**< Data in next byte */
	};

	/**
	 * @brief CAT Tx Meters response uint16 struct
	 */
	struct __attribute__((__packed__)) CAT_tx_meters
	{
		uint8_t ALC : 4; /**< Automatic Level Control meter */
		uint8_t PWR : 4; /**< Power meter */
		uint8_t MOD : 4; /**< Modulation meter */
		uint8_t VSWR : 4; /**< Voltage Standing Wave Ratio meter */
	};

	/* Extra functionality */

	/* EEPROM Block Type */
	struct CAT_eeprom_block {
		uint16_t start_adr;
		size_t size;
		uint8_t* data;
	};

	/********************
	 * GLOBAL VARIABLES *
	 ********************/

	extern enum CAT_errno g_cat_errno; /**< Global CAT errno */
	extern enum CAT_nack g_cat_nack; /**< Global CAT nack value */

	/* Values Standard CTCSS Tones (value x10) */
	extern const CAT_decihertz CAT_STD_CTCSS_TONES[CAT_MAX_STD_CTCSS];
	/* Values Standard DCS Codes */
	extern const unsigned short CAT_STD_DCS_CODES[CAT_MAX_STD_DCS];

	/* SERIAL DEVICE FN */
	/**
	 * @brief Creates a CAT device object to use with CAT_transaction()
	 * @param device
	 *	POSIX: path to the serial device file
	 *	ARDUINO: string with serial name, RTS and CTS pins
	 * @param bitrate 4800, 9600 or 38400 bps
	 * @param timeout max wait time for transaction response in milliseconds
	 * Can use macro CAT_DEFAULT_TIMEOUT
	 * @return Serial CAT device object pointer on success. Null on error.
	 */
	struct CAT_device* CAT_open_device(char* device, int bitrate, int timeout);

	/**
	 * @brief Close serial CAT device and restores previous config.
	 * @param dev
	 */
	void CAT_close_device(struct CAT_device* dev);

	/**
	 * @brief Hardware PTT switching via RTS
	 * For use with 6-MiniDIN data port PTT. If use the same PC serial port
	 * for RTS must use the same CAT_device object, else open two CAT_device
	 * objects: one for CAT commands and other for hardware PTT.
	 * @param dev
	 * @param on_off
	 * @return true on success, false on error
	 */
	bool CAT_switch_rts_ptt(struct CAT_device* dev, bool on_off);

	int CAT_get_cts_sql(struct CAT_device* dev);

	/**
	 * @brief
	 * @param dev
	 * @param new_rate
	 * @return old rate
	 */
	int CAT_change_device_rate(struct CAT_device* dev, int new_rate);

	/**
	 * @brief
	 * @param dev
	 * @param seconds
	 * @return old timeout
	 */
	int CAT_change_transaction_timeout(struct CAT_device* dev, int milliseconds);

	/**
	 * @brief
	 * @param dev
	 * @param request
	 * @param free_request
	 * @return 
	 */
	struct CAT_response CAT_transaction(struct CAT_device* dev, struct CAT_request* request);

	/**
	 * @brief Write n bytes in CAT device. The purpose of this function is for clone mode. Use CAT_transaction for regular CAT protocol instead
	 * @param dev
	 * @param buf is a pointer to the buffer to write
	 * @param nbytes is the number of bytes to write
	 * @return 0 on error, 1 on success.
	 */
	int CAT_write_bytes(struct CAT_device* dev, uint8_t* buf, size_t nbytes);

	/**
	 * @brief Read bytes in CAT device. The purpose of this function is for clone mode. Use CAT_transaction for regular CAT protocol instead
	 * @param dev
	 * @return 
	 */
	int CAT_read_bytes(struct CAT_device* dev, uint8_t* dest, size_t size);


	/* EEPROM BLOCK FN*/

	/**
	 * @brief Read EEPROM block from the transceiver
	 * @param dev serial CAT device wich transceiver is plugged
	 * @param dest struct where data will be stored
	 * @param start_adr EEPROM adress to start reading.
	 * @param len number of bytes to read
	 * @return 
	 */
	struct CAT_eeprom_block* CAT_read_eeprom_block(
		struct CAT_device* dev,
		uint16_t start_adr,
		size_t len,
		void (*progress_callback)(float, void*),
		void* callback_args);

	void
	CAT_free_eeprom_block(struct CAT_eeprom_block* src);

	int
	CAT_write_eeprom_block(
		struct CAT_device* dev,
		struct CAT_eeprom_block* src,
		void (*progress_callback)(float, void*),
		void* callback_args);

	/******************
	 * ENUM "METHODS" *
	 ******************/

	/**
	 * @brief Get corresponding op_mode enum from string
	 * @param str_mode is the string that contains the op_mode name
	 * @return enum op_mode
	 */
	enum CAT_op_mode CAT_op_mode_from_str(char* str_mode);

	/**
	 * @brief
	 * @param mode
	 * @return 
	 */
	const char* CAT_op_mode_to_str(enum CAT_op_mode mode);

	/**
	 * @brief
	 * @param str
	 * @return 
	 */
	enum CAT_selective_call CAT_selective_call_from_str(char* str);

	/**
	 * @brief
	 * @param mode
	 * @return 
	 */
	const char* CAT_selective_call_to_str(enum CAT_selective_call mode);

	/**
	 * @brief
	 * @param str
	 * @return 
	 */
	enum CAT_repeater_shift CAT_repeater_shift_from_str(char* str);

	/**
	 * @brief
	 * @param mode
	 * @return 
	 */
	const char* CAT_repeater_shift_to_str(enum CAT_repeater_shift mode);

	char* CAT_decahertz_to_khz_str(CAT_decahertz frequency);

	char* CAT_decahertz_to_mhz_str(CAT_decahertz frequency);

	/**
	 * @brief
	 * @param nack
	 * @return 
	 */
	char* CAT_nack_to_str(enum CAT_nack nack);

	/**
	 * @brief
	 * @param resp
	 * @return 
	 */
	enum CAT_nack CAT_get_nack(struct CAT_response* resp);

	/**
	 * @brief
	 * @param err
	 * @return 
	 */
	const char* CAT_errno_to_str(enum CAT_errno err);

	/**
	 * @brief Valid CTCSS tone checker
	 * @param tone
	 * @return 0 not valid, 1 valid
	 */
	int CAT_check_ctcss_tone(unsigned short tone);

	/**
	 * @brief Valid DCS code checker
	 * @param code
	 * @return 0 not valid, 1 valid
	 */
	int CAT_check_dcs_code(unsigned short code);

	/**
	 * @ Valid CAT bitrate checker
	 * @param bitrate
	 * @return 
	 */
	int CAT_check_dev_rate(int bitrate);

	/**********************
	 * YAESU CAT REQUESTS *
	 **********************/

	struct CAT_request* CAT_request_dup(struct CAT_request* src);

	/**
	 * @brief Toogle VFO A/B Request
	 * @return Pointer to a malloc()ed request to send
	 */
	struct CAT_request* CAT_toggle_vfo_request(void);

	/**
	 * @brief Lock transceiver Request
	 * @param on_off OFF (0) or ON (1) to unlock or lock
	 * @return Pointer to a malloc()ed request to send or NULL if error
	 */
	struct CAT_request* CAT_switch_lock_request(bool on_off);

	/**
	 * @brief Key/unkey PTT Request
	 * @param on_off OFF (0) or ON (1) to unkey or key PTT
	 * @return Pointer to a malloc()ed request to send
	 */
	struct CAT_request* CAT_switch_ptt_request(bool on_off);

	/**
	 * @brief Turn Split mode ON/OFF Request
	 * @param on_off OFF (0) or ON (1)
	 * @return Pointer to a malloc()ed request to send
	 */
	struct CAT_request* CAT_switch_split_request(bool on_off);

	/**
	 * @brief Turn Clarifier ON/OFF Request
	 * @param on_off OFF (0) or ON (1)
	 * @return RPointer to a malloc()ed request to send
	 */
	struct CAT_request* CAT_switch_clarifier_request(bool on_off);

	/**
	 * @brief Set Clarifier Frequency Request
	 * @param frequency is the new value to set
	 * @pre
	 * -# frequency must be an integer > 0 < 9999
	 * @return Pointer to a malloc()ed request to send or NULL
	 */
	struct CAT_request* CAT_set_clarifier_frequency_request(CAT_decahertz frequency);
	/**
	 * @brief Set Selectivel Call (CTCSS/DCS) mode;
	 * @param mode
	 * @return Pointer to a malloc()ed request to send or NULL
	 */
	struct CAT_request* CAT_set_ctcss_dcs_mode_request(enum CAT_selective_call mode);

	/**
	 * @brief
	 * @param tone_Tx
	 * @param tone_Rx
	 * @return 
	 */
	struct CAT_request* CAT_set_ctcss_tone_request(CAT_decihertz tone_tx, CAT_decihertz tone_rx);

	/**
	 * @brief
	 * @param code_Tx
	 * @param code_Rx
	 * @return 
	 */
	struct CAT_request* CAT_set_dcs_code_request(int code_tx, int code_rx);

	/**
	 * @brief
	 * @param shift
	 * @return 
	 */
	struct CAT_request* CAT_set_repeater_shift_request(enum CAT_repeater_shift shift);

	/**
	 * @brief 
	 * @param frequency must be integer (unit is Hz/10);
	 * @return 
	 */
	struct CAT_request* CAT_set_repeater_offset_request(CAT_hertz frequency);

	/**
	 * @brief Set transceiver frequency Request
	 * @param frequency is the new value to set
	 * @pre
	 * -# frequency must be an integer > 0 < 99999999
	 * @return Request to send or Request CMD Error code
	 */
	struct CAT_request* CAT_set_frequency_request(CAT_decahertz frequency);

	/**
	 * @brief Set transceiver frequency Request
	 * @param mode is the operation mode to set
	 * @return Request to send or Request CMD Error code
	 */
	struct CAT_request* CAT_set_mode_request(enum CAT_op_mode mode);

	/**
	 * @brief Get frequency and operation mode status Request
	 * @return Request to send
	 */
	struct CAT_request* CAT_get_frequency_mode_request(void);

	/**
	 * @brief Get Rx status Request
	 * @return Request to send
	 */
	struct CAT_request* CAT_get_status_rx_request(void);

	/**
	 * @brief Get Tx status Request
	 * @return Request to send
	 */
	struct CAT_request* CAT_get_status_tx_request(void);

	/**
	 * @brief
	 * @param adr
	 * @param data
	 * @param data_next
	 * @return 
	 */
	struct CAT_request* CAT_write_eeprom_request(uint16_t adress, uint8_t data, uint8_t data_next);

	/**
	 * @brief Read Eeprom Request
	 * 
	 * Reads two bytes in trasnceiver eeprom starting with adress and adress +1 
	 * @return Request to send
	 */
	struct CAT_request* CAT_read_eeprom_request(uint16_t adress);

	/**
	 * @brief Turn transceiver ON/OFF Request
	 * @param on_off OFF (0) or ON (1)
	 * @return Request to send
	 * @note Only for FT817
	 */
	struct CAT_request* CAT_turn_radio_request(bool on_off);

	/**
	 * @brief Get transceiver config status Request
	 * @return Request to send
	 */
	struct CAT_request* CAT_get_rig_config_request(void);

	/**
	 * @brief Get Tx keyed state Request
	 * @return Request to send
	 * @note Only for FT817
	 */
	struct CAT_request* CAT_get_tx_keyed_state_request(void);

	/**
	 * @brief Get Tx meter status Request
	 * @return Request to send
	 */
	struct CAT_request* CAT_get_tx_meters_request(void);

	/**
	 * @brief Transceiver Factory Reset Request
	 * @return Request to send
	 */
	struct CAT_request* CAT_factory_reset_request(void);

	/*****************
	 * CAT RESPONSES *
	 *****************/

	int CAT_check_response(struct CAT_response* resp);

	/**
	 * @brief Read transceiver operation mode from a Response
	 * @param *ResponseFreqMode is a Response String.
	 * @return Operation mode from a Response.
	 * @see getFreqModeRequest()
	 */
	enum CAT_op_mode CAT_decode_mode_response(struct CAT_response* status);

	/**
	 * @brief Read transceiver frequency from a Response
	 * @param *ResponseFreqMode is a Response String.
	 * @return Frequency from a Response String.
	 * @see getFreqModeRequest()
	 */
	CAT_decahertz CAT_decode_frequency_response(struct CAT_response* status);

	/**
	 * @brief
	 * @param dest
	 * @param resp
	 * @param free_resp
	 * @return 
	 */
	struct CAT_status_rx CAT_decode_status_rx_response(struct CAT_response* resp);

	/**
	 * 
	 * @param resp
	 * @return 
	 */
	struct CAT_status_tx CAT_decode_status_tx_response(struct CAT_response* resp);

	/**
	 * 
	 * @param resp
	 * @return 
	 */
	struct CAT_tx_meters CAT_decode_tx_meters_response(struct CAT_response* resp);

	/**
	 * @brief
	 * @param dest
	 * @param resp
	 * @param free_resp
	 * @return 
	 */
	bool CAT_decode_eeprom_response(struct CAT_eeprom_byte* dest, struct CAT_response* resp);

	enum CAT_cmd CAT_cmd_id_from_request(struct CAT_request* request);


#ifdef __cplusplus
}
#endif

#endif /* YAESU_CAT_H */

