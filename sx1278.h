// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __SX1278_H__
#define __SX1278_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

#define SX1278_IRQ_ACTIVE_LEVEL  		0
#define SX1278_IRQ_UNACTIVE_LEVEL		1

typedef err_code_t (*sx1278_func_spi_send)(uint8_t *buf_send, uint16_t len);
typedef err_code_t (*sx1278_func_spi_recv)(uint8_t *buf_recv, uint16_t len);
typedef err_code_t (*sx1278_func_set_gpio)(uint8_t level);
typedef err_code_t (*sx1278_func_get_gpio)(uint8_t *level);
typedef void (*sx1278_func_delay)(uint32_t time_ms);

/**
 * @brief   SX1278 handle structure.
 */
typedef struct sx1278* sx1278_handle_t;

/**
 * @brief   Output power.
 */
typedef enum {
	SX1278_OUTPUT_PWR_11dBm = 0,		/*!< Output power 11 dBm */
	SX1278_OUTPUT_PWR_14dBm,			/*!< Output power 14 dBm */
	SX1278_OUTPUT_PWR_17dBm,			/*!< Output power 17 dBm */
	SX1278_OUTPUT_PWR_20dBm				/*!< Output power 20 dBm */
} sx1278_output_pwr_t;

/**
 * @brief   Spread factor.
 */
typedef enum {
	SX1278_SPREAD_FACTOR_6 = 0,			/*!< Spread factor 6 */
	SX1278_SPREAD_FACTOR_7,				/*!< Spread factor 7 */
	SX1278_SPREAD_FACTOR_8,				/*!< Spread factor 8 */
	SX1278_SPREAD_FACTOR_9,				/*!< Spread factor 9 */
	SX1278_SPREAD_FACTOR_10,			/*!< Spread factor 10 */
	SX1278_SPREAD_FACTOR_11,			/*!< Spread factor 11 */
	SX1278_SPREAD_FACTOR_12				/*!< Spread factor 12 */
} sx1278_spread_factor_t;

/**
 * @brief   Bandwidth.
 */
typedef enum {
	SX1278_BANDWIDTH_7_8kHz = 0,		/*!< Bandwidth 7.8 kHz */
	SX1278_BANDWIDTH_10_4kHz,			/*!< Bandwidth 10.4 kHz */
	SX1278_BANDWIDTH_15_6kHz,			/*!< Bandwidth 15.6 kHz */
	SX1278_BANDWIDTH_20_8kHz,			/*!< Bandwidth 20.8 kHz */
	SX1278_BANDWIDTH_31_2kHz,			/*!< Bandwidth 31.2 kHz */
	SX1278_BANDWIDTH_41_7kHz,			/*!< Bandwidth 41.7 kHz */
	SX1278_BANDWIDTH_62_5kHz,			/*!< Bandwidth 62.5 kHz */
	SX1278_BANDWIDTH_125kHz,			/*!< Bandwidth 125 kHz */
	SX1278_BANDWIDTH_250kHz,			/*!< Bandwidth 250 kHz */
	SX1278_BANDWIDTH_500kHz				/*!< Bandwidth 500 kHz */
} sx1278_bandwidth_t;

/**
 * @brief   Coding rate.
 */
typedef enum {
	SX1278_CODING_RATE_4_5 = 0,			/*!< Coding rate 4/5 */
	SX1278_CODING_RATE_4_6,				/*!< Coding rate 4/6 */
	SX1278_CODING_RATE_4_7,				/*!< Coding rate 4/7 */
	SX1278_CODING_RATE_4_8				/*!< Coding rate 4/8 */
} sx1278_coding_rate_t;

/**
 * @brief   CRC mode.
 */
typedef enum {
	SX1278_CRC_DISABLE = 0,				/*!< Disable CRC */
	SX1278_CRC_ENABLE					/*!< Enable CRC */
} sx1278_crc_mode_t;

/**
 * @brief   Transceiver mode.
 */
typedef enum {
	SX1278_TRANSCEIVER_MODE_SLEEP = 0,	/*!< Sleep mode */
	SX1278_TRANSCEIVER_MODE_STANDBY,	/*!< Standby mode */
	SX1278_TRANSCEIVER_MODE_FS_TX,		/*!< Frequency synthesiser to Tx frequency mode */
	SX1278_TRANSCEIVER_MODE_TX,			/*!< Transmit mode */
	SX1278_TRANSCEIVER_MODE_FS_RX,		/*!< Frequency synthesiser to Rx frequency mode */
	SX1278_TRANSCEIVER_MODE_RX			/*!< Receive mode */
} sx1278_transceiver_mode_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	uint64_t  					freq;				/*!< Frequency */
	uint8_t  					packet_len;			/*!< Packet length */
	sx1278_output_pwr_t   		output_pwr;	 		/*!< Ouput power */
	sx1278_spread_factor_t 		spread_factor;		/*!< Spread factor */
	sx1278_bandwidth_t  		bandwidth;			/*!< Bandwidth */
	sx1278_coding_rate_t  		coding_rate;		/*!< Coding rate */
	sx1278_crc_mode_t 			crc_en;				/*!< CRC enable/disable */
	sx1278_transceiver_mode_t 	transceiver_mode;	/*!< Transceiver mode */
	sx1278_func_spi_send 		spi_send;			/*!< Function SPI send */
	sx1278_func_spi_recv 		spi_recv;			/*!< Function SPI receive */
	sx1278_func_set_gpio 		set_cs;				/*!< Function set chip select pin */
	sx1278_func_set_gpio 		set_rst;			/*!< Function set reset pin */
	sx1278_func_get_gpio  		get_irq;  			/*!< Function get irq pin */
	sx1278_func_delay 			delay;				/*!< Function delay */
} sx1278_cfg_t;

/*
 * @brief   Initialize SX1278 with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
sx1278_handle_t sx1278_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_set_config(sx1278_handle_t handle, sx1278_cfg_t config);

/*
 * @brief   Configure SX1278.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_config(sx1278_handle_t handle);

/*
 * @brief   Transmit data. After that, monitor IRQ pin is necessary to call
 * 			"sx1278_lora_clear_irq_flags" which clear transmitted interrupt flags.
 *
 * @note    SX1278 need to be configured in SX1278_TRANSCEIVER_MODE_TX mode.
 *
 *
 * @param 	handle Handle structure.
 * @param 	data Data.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_lora_transmit(sx1278_handle_t handle, uint8_t *data);

/*
 * @brief   Transmit data and polling until IRQ is triggered or timeout. Function
 * 			"sx1278_lora_clear_irq_flags" is called automatically to clear interrupt
 * 			flags if transmit success.
 *
 * @note 	Functions "get_irq" and "delay" need to be assigned .
 * 			SX1278 need to be configured in SX1278_TRANSCEIVER_MODE_TX mode.
 *
 * @param 	handle Handle structure.
 * @param 	data Data.
 * @param 	timeout_ms Timeout in ms.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_lora_transmit_polling(sx1278_handle_t handle, uint8_t *data, uint32_t timeout_ms);

/*
 * @brief   Receive data and call "sx1278_lora_clear_irq_flags" automatically to clear
 * 			interrupt flags when receive success.
 * 			Monitor IRQ pin is neccessary to ensure data are received before
 * 			call this function.
 *
 * @note    SX1278 need to be configured in SX1278_TRANSCEIVER_MODE_RX mode.
 *
 * @param 	handle Handle structure.
 * @param 	data Data.
 * @param 	num_bytes Number of bytes data that have been received.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_lora_receive(sx1278_handle_t handle, uint8_t *data, uint16_t *num_bytes);

/*
 * @brief   Polling until IRQ pin is triggered or timeout. When data is received,
 * 			function "sx1278_lora_clear_irq_flags" is called automatically to clear
 * 			interrupt flags.
 *
 * @note    Functions "get_irq" and "delay" need to be assigned.
 * 			SX1278 need to be configured in SX1278_TRANSCEIVER_MODE_RX mode.
 *
 * @param 	handle Handle structure.
 * @param 	data Data.
 * @param 	num_bytes Number of bytes data that have been received.
 * @param 	timeout_ms Timeout in ms.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_lora_receive_polling(sx1278_handle_t handle, uint8_t *data, uint16_t *num_bytes, uint32_t timeout_ms);

/*
 * @brief   Clear LoRa interrupt flags.
 *
 * @note 	Mode transmitter: After data sent by "sx1278_lora_transmit", this
 * 			function should be called when IRQ pin triggered which notify
 * 			transmit complete.
 * 			Mode receiver: This function should be called when IRQ pin triggered
 * 			which notify data ready. Then, "nrf24l01_receive" can be called to
 * 			read received data.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_lora_clear_irq_flags(sx1278_handle_t handle);

/*
 * @brief   Get RSSI in LoRa mode.
 *
 * @param 	handle Handle structure.
 * @param 	rssi RSSI.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_lora_get_rssi(sx1278_handle_t handle, uint8_t *rssi);

/*
 * @brief   Configure SX1278 to enter sleep.
 *
 * @param 	handle Handle structure.
 * @param 	mode Transceiver mode.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_set_transceiver_mode(sx1278_handle_t handle, sx1278_transceiver_mode_t mode);

/*
 * @brief   Reset SX1278 by hardware.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t sx1278_hw_reset(sx1278_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __SX1278_H__ */
