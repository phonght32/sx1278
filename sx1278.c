#include "stdlib.h"
#include "string.h"
#include "sx1278.h"

/**
 * @brief   Register address in LoRa mode.
 */
#define SX1278_LR_RegFifo                     	0x00
#define SX1278_LR_RegOpMode                   	0x01
#define SX1278_LR_RegFrMsb                    	0x06
#define SX1278_LR_RegFrMid                    	0x07
#define SX1278_LR_RegFrLsb                   	0x08
#define SX1278_LR_RegPaConfig                  	0x09
#define SX1278_LR_RegPaRamp                    	0x0A
#define SX1278_LR_RegOcp                       	0x0B
#define SX1278_LR_RegLna                       	0x0C
#define SX1278_LR_RegFifoAddrPtr               	0x0D
#define SX1278_LR_RegFifoTxBaseAddr            	0x0E
#define SX1278_LR_RegFifoRxBaseAddr        		0x0F
#define SX1278_LR_RegFifoRxCurrentaddr        	0x10
#define SX1278_LR_RegIrqFlagsMask            	0x11
#define SX1278_LR_RegIrqFlags               	0x12
#define SX1278_LR_RegRxNbBytes          		0x13
#define SX1278_LR_RegRxHeaderCntValueMsb      	0x14
#define SX1278_LR_RegRxHeaderCntValueLsb    	0x15
#define SX1278_LR_RegRxPacketCntValueMsb    	0x16
#define SX1278_LR_RegRxPacketCntValueLsb       	0x17
#define SX1278_LR_RegModemStat           		0x18
#define SX1278_LR_RegPktSnrValue           		0x19
#define SX1278_LR_RegPktRssiValue           	0x1A
#define SX1278_LR_RegRssiValue           		0x1B
#define SX1278_LR_RegHopChannel           		0x1C
#define SX1278_LR_RegModemConfig1           	0x1D
#define SX1278_LR_RegModemConfig2           	0x1E
#define SX1278_LR_RegSymbTimeoutLsb           	0x1F
#define SX1278_LR_RegPreambleMsb           		0x20
#define SX1278_LR_RegPreambleLsb           		0x21
#define SX1278_LR_RegPayloadLength           	0x22
#define SX1278_LR_RegMaxPayloadLength           0x23
#define SX1278_LR_RegHopPeriod           		0x24
#define SX1278_LR_RegFifoRxByteAddr           	0x25
#define SX1278_LR_RegModemConfig3              	0x26
#define SX1278_LR_REG_DIOMAPPING1              	0x40
#define SX1278_LR_REG_DIOMAPPING2              	0x41
#define SX1278_LR_REG_VERSION                 	0x42
#define SX1278_LR_REG_PLLHOP                  	0x44
#define SX1278_LR_REG_TCXO                    	0x4B
#define SX1278_LR_REG_PADAC                  	0x4D
#define SX1278_LR_REG_FORMERTEMP           		0x5B
#define SX1278_LR_REG_AGCREF					0x61
#define SX1278_LR_REG_AGCTHRESH1             	0x62
#define SX1278_LR_REG_AGCTHRESH2             	0x63
#define SX1278_LR_REG_AGCTHRESH3                0x64

/**
 * @brief   Register address in FSK/OOK mode.
 */
#define SX1278_FSK_OOK_RegFIFO                	0x00
#define SX1278_FSK_OOK_RegOpMode              	0x01
#define SX1278_FSK_OOK_RegBitRateMsb      		0x02
#define SX1278_FSK_OOK_RegBitRateLsb      		0x03
#define SX1278_FSK_OOK_RegFdevMsb             	0x04
#define SX1278_FSK_OOK_RegFdevLsb             	0x05
#define SX1278_FSK_OOK_RegFreqMsb             	0x06
#define SX1278_FSK_OOK_RegFreqMid             	0x07
#define SX1278_FSK_OOK_RegFreqLsb         		0x08
#define SX1278_FSK_OOK_RegPaConfig            	0x09
#define SX1278_FSK_OOK_RegPaRamp              	0x0a
#define SX1278_FSK_OOK_RegOcp                 	0x0b
#define SX1278_FSK_OOK_RegLna                 	0x0c
#define SX1278_FSK_OOK_RegRxConfig            	0x0d
#define SX1278_FSK_OOK_RegRssiConfig      		0x0e
#define SX1278_FSK_OOK_RegRssiCollision 		0x0f
#define SX1278_FSK_OOK_RegRssiThresh      		0x10
#define SX1278_FSK_OOK_RegRssiValue           	0x11
#define SX1278_FSK_OOK_RegRxBw                	0x12
#define SX1278_FSK_OOK_RegAfcBw               	0x13
#define SX1278_FSK_OOK_RegOokPeak             	0x14
#define SX1278_FSK_OOK_RegOokFix              	0x15
#define SX1278_FSK_OOK_RegOokAvg              	0x16
#define SX1278_FSK_OOK_RegAfcFei              	0x1a
#define SX1278_FSK_OOK_RegAfcMsb              	0x1b
#define SX1278_FSK_OOK_RegAfcLsb              	0x1c
#define SX1278_FSK_OOK_RegFeiMsb              	0x1d
#define SX1278_FSK_OOK_RegFeiLsb              	0x1e
#define SX1278_FSK_OOK_RegPreambleDetect  		0x1f
#define SX1278_FSK_OOK_RegRxTimeout1      		0x20
#define SX1278_FSK_OOK_RegRxTimeout2      		0x21
#define SX1278_FSK_OOK_RegRxTimeout3      		0x22
#define SX1278_FSK_OOK_RegRxDelay             	0x23
#define SX1278_FSK_OOK_RegOsc                 	0x24
#define SX1278_FSK_OOK_RegPreambleMsb     		0x25
#define SX1278_FSK_OOK_RegPreambleLsb     		0x26
#define SX1278_FSK_OOK_RegSyncConfig      		0x27
#define SX1278_FSK_OOK_RegSyncValue1      		0x28
#define SX1278_FSK_OOK_RegSyncValue2      		0x29
#define SX1278_FSK_OOK_RegSyncValue3      		0x2a
#define SX1278_FSK_OOK_RegSyncValue4      		0x2b
#define SX1278_FSK_OOK_RegSyncValue5      		0x2c
#define SX1278_FSK_OOK_RegSyncValue6      		0x2d
#define SX1278_FSK_OOK_RegSyncValue7      		0x2e
#define SX1278_FSK_OOK_RegSyncValue8      		0x2f
#define SX1278_FSK_OOK_RegPacketConfig1       	0x30
#define SX1278_FSK_OOK_RegPacketConfig2       	0x31
#define SX1278_FSK_OOK_RegPayloadLength       	0x32
#define SX1278_FSK_OOK_RegNodeAdrs            	0x33
#define SX1278_FSK_OOK_RegBroadcastAdrs       	0x34
#define SX1278_FSK_OOK_RegFifoThresh      		0x35
#define SX1278_FSK_OOK_RegSeqConfig1      		0x36
#define SX1278_FSK_OOK_RegSeqConfig2      		0x37
#define SX1278_FSK_OOK_RegTimerResol      		0x38
#define SX1278_FSK_OOK_RegTimer1Coef      		0x39
#define SX1278_FSK_OOK_RegSyncWord				0x39
#define SX1278_FSK_OOK_RegTimer2Coef      		0x3a
#define SX1278_FSK_OOK_RegImageCal            	0x3b
#define SX1278_FSK_OOK_RegTemp                	0x3c
#define SX1278_FSK_OOK_RegLowBat              	0x3d
#define SX1278_FSK_OOK_RegIrqFlags1           	0x3e
#define SX1278_FSK_OOK_RegIrqFlags2           	0x3f
#define SX1278_FSK_OOK_RegDioMapping1			0x40
#define SX1278_FSK_OOK_RegDioMapping2			0x41
#define SX1278_FSK_OOK_RegVersion				0x42
#define SX1278_FSK_OOK_RegPllHop				0x44
#define SX1278_FSK_OOK_RegPaDac					0x4d
#define SX1278_FSK_OOK_RegBitRateFrac			0x5d

#define SX1278_CS_ACTIVE   						0
#define SX1278_CS_UNACTIVE   					1

#define SX1278_RST_ACTIVE   					0
#define SX1278_RST_UNACTIVE   					1

typedef struct sx1278 {
	uint64_t  					freq;			/*!< Frequency */
	uint8_t  					packet_len;		/*!< Packet length */
	sx1278_output_pwr_t   		output_pwr;	 	/*!< Ouput power */
	sx1278_spread_factor_t 		spread_factor;	/*!< Spread factor */
	sx1278_bandwidth_t  		bandwidth;		/*!< Bandwidth */
	sx1278_coding_rate_t  		coding_rate;	/*!< Coding rate */
	sx1278_crc_mode_t 			crc_en;			/*!< CRC enable/disable */
	sx1278_transceiver_mode_t 	mode;			/*!< Transceiver mode */
	sx1278_func_spi_send 		spi_send;		/*!< Function SPI send */
	sx1278_func_spi_recv 		spi_recv;		/*!< Function SPI receive */
	sx1278_func_set_gpio 		set_cs;			/*!< Function set chip select pin */
	sx1278_func_set_gpio 		set_rst;		/*!< Function set reset pin */
	sx1278_func_get_gpio  		get_irq;  		/*!< Function get irq pin */
	sx1278_func_delay 			delay;			/*!< Function delay */
} sx1278_t;

static err_code_t sx1278_read_register(sx1278_handle_t handle, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	if (handle->set_cs != NULL)
	{
		handle->set_cs(SX1278_CS_ACTIVE);
	}

	uint8_t cmd = reg_addr;

	handle->spi_send(&cmd, 1);
	handle->spi_recv(data, len);

	if (handle->set_cs != NULL)
	{
		handle->set_cs(SX1278_CS_UNACTIVE);
	}

	return ERR_CODE_SUCCESS;
}

static err_code_t sx1278_write_register(sx1278_handle_t handle, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	if (handle->set_cs != NULL)
	{
		handle->set_cs(SX1278_CS_ACTIVE);
	}

	uint8_t cmd = reg_addr | 0x80;

	handle->spi_send(&cmd, 1);
	handle->spi_recv(data, len);

	if (handle->set_cs != NULL)
	{
		handle->set_cs(SX1278_CS_UNACTIVE);
	}

	return ERR_CODE_SUCCESS;
}

static err_code_t sx1278_enter_sleep(sx1278_handle_t handle)
{
	uint8_t cmd = 0x08;
	sx1278_write_register(handle, SX1278_LR_RegOpMode, &cmd, 1);

	handle->mode = SX1278_TRANSCEIVER_MODE_SLEEP;

	return ERR_CODE_SUCCESS;
}

static err_code_t sx1278_enter_standby(sx1278_handle_t handle)
{
	uint8_t cmd = 0x09;
	sx1278_write_register(handle, SX1278_LR_RegOpMode, &cmd, 1);

	handle->mode = SX1278_TRANSCEIVER_MODE_STANDBY;

	return ERR_CODE_SUCCESS;
}

static err_code_t sx1278_enter_lora_transmitter(sx1278_handle_t handle)
{
	uint8_t tx_base_addr;
	uint8_t cmd_data;
	uint8_t payload_len;
	uint32_t timeout_ms = 1000;

	/* Tx for 20dBm */
	cmd_data = 0x87;
	sx1278_write_register(handle, SX1278_LR_REG_PADAC, &cmd_data, 1);

	/* No FHSS */
	cmd_data = 0x00;
	sx1278_write_register(handle, SX1278_LR_RegHopPeriod, &cmd_data, 1);

	/* DIO0=01, DIO1=00,DIO2=00, DIO3=01 */
	cmd_data = 0x41;
	sx1278_write_register(handle, SX1278_LR_REG_DIOMAPPING1, &cmd_data, 1);

	sx1278_lora_clear_irq_flags(handle);

	/* Open TX done interrupt */
	cmd_data = 0xF7;
	sx1278_write_register(handle, SX1278_LR_RegIrqFlagsMask, &cmd_data, 1);

	/* Packet length - This register must be defined when the data long of one byte in SF is 6 */
	cmd_data = handle->packet_len;
	sx1278_write_register(handle, SX1278_LR_RegPayloadLength, &cmd_data, 1);

	/* Read TX base address */
	sx1278_read_register(handle, SX1278_LR_RegFifoTxBaseAddr, &tx_base_addr, 1);
	sx1278_write_register(handle, SX1278_LR_RegFifoAddrPtr, &tx_base_addr, 1);

	while (1)
	{
		sx1278_read_register(handle, SX1278_LR_RegPayloadLength, &payload_len, 1);

		if (payload_len == handle->packet_len)
		{
			handle->mode = SX1278_TRANSCEIVER_MODE_TX;
			return ERR_CODE_SUCCESS;
		}

		if (--timeout_ms == 0)
		{
			return ERR_CODE_FAIL;
		}

		handle->delay(1);
	}

	return ERR_CODE_SUCCESS;
}

static err_code_t sx1278_enter_lora_receiver(sx1278_handle_t handle)
{
	uint8_t rx_base_addr;
	uint8_t cmd_data;
	uint8_t modem_stat;
	uint32_t timeout_ms = 1000;

	/* Normal and RX */
	cmd_data = 0x84;
	sx1278_write_register(handle, SX1278_LR_REG_PADAC, &cmd_data, 1);

	/* No FHSS */
	cmd_data = 0xFF;
	sx1278_write_register(handle, SX1278_LR_RegHopPeriod, &cmd_data, 1);

	/* DIO=00,DIO1=00,DIO2=00, DIO3=01 */
	cmd_data = 0x01;
	sx1278_write_register(handle, SX1278_LR_REG_DIOMAPPING1, &cmd_data, 1);

	/* Open RX done interrupt & timeout */
	cmd_data = 0x3F;
	sx1278_write_register(handle, SX1278_LR_RegIrqFlagsMask, &cmd_data, 1);

	sx1278_lora_clear_irq_flags(handle);

	/* Packet length - This register must be defined when the data long of one byte in SF is 6 */
	cmd_data = handle->packet_len;
	sx1278_write_register(handle, SX1278_LR_RegPayloadLength, &cmd_data, 1);

	/* Read RX base address */
	sx1278_read_register(handle, SX1278_LR_RegFifoRxBaseAddr, &rx_base_addr, 1);
	sx1278_write_register(handle, SX1278_LR_RegFifoAddrPtr, &rx_base_addr, 1);

	/* Set mode low frequency */
	cmd_data = 0x8d;
	sx1278_write_register(handle, SX1278_LR_RegOpMode, &cmd_data, 1);

	while (1)
	{
		sx1278_read_register(handle, SX1278_LR_RegModemStat, &modem_stat, 1);

		if ((modem_stat & 0x04) == 0x04)
		{
			handle->mode = SX1278_TRANSCEIVER_MODE_RX;
			return ERR_CODE_SUCCESS;
		}

		if (--timeout_ms == 0)
		{
			return ERR_CODE_FAIL;
		}

		handle->delay(1);
	}

	return ERR_CODE_SUCCESS;
}

sx1278_handle_t sx1278_init(void)
{
	sx1278_handle_t handle = calloc(1, sizeof(sx1278_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

err_code_t sx1278_set_config(sx1278_handle_t handle, sx1278_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->freq = config.freq;
	handle->packet_len = config.packet_len;
	handle->output_pwr = config.output_pwr;
	handle->spread_factor = config.spread_factor;
	handle->bandwidth = config.bandwidth;
	handle->coding_rate = config.coding_rate;
	handle->crc_en = config.crc_en;
	handle->mode = config.mode;
	handle->spi_send = config.spi_send;
	handle->spi_recv = config.spi_recv;
	handle->set_cs = config.set_cs;
	handle->set_rst = config.set_rst;
	handle->get_irq = config.get_irq;
	handle->delay = config.delay;

	return ERR_CODE_SUCCESS;
}

err_code_t sx1278_config(sx1278_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t ret;
	uint8_t cmd_data;
	uint8_t freq_reg[3];

	/* Change modem mode must in sleep mode */
	sx1278_enter_sleep(handle);

	if (handle->delay != NULL)
	{
		handle->delay(15);
	}

	/* Entry LoRa */
	cmd_data = 0x88;
	sx1278_write_register(handle, SX1278_LR_RegOpMode, &cmd_data, 1);

	/* Configure frequency */
	uint64_t freq = ((uint64_t) handle->freq << 19) / 32000000;
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	sx1278_write_register(handle, SX1278_LR_RegFrMsb, (uint8_t*) freq_reg, 3);

	cmd_data = 0x34;
	sx1278_write_register(handle, SX1278_FSK_OOK_RegSyncWord, &cmd_data, 1);

	/* Configure base output power */
	switch (handle->output_pwr)
	{
	case SX1278_OUTPUT_PWR_11dBm:
		cmd_data = 0xF6;
		break;
	case SX1278_OUTPUT_PWR_14dBm:
		cmd_data = 0xF9;
		break;
	case SX1278_OUTPUT_PWR_17dBm:
		cmd_data = 0xFC;
		break;
	case SX1278_OUTPUT_PWR_20dBm:
		cmd_data = 0xFF;
		break;
	default:
		cmd_data = 0xFF;
		break;
	}
	sx1278_write_register(handle, SX1278_LR_RegPaConfig, &cmd_data, 1);

	/* Close current protection */
	cmd_data = 0x0B;
	sx1278_write_register(handle, SX1278_LR_RegOcp, &cmd_data, 1);

	/* Configure LNA */
	cmd_data = 0x23;
	sx1278_write_register(handle, SX1278_LR_RegLna, &cmd_data, 1);


	if (handle->spread_factor == SX1278_SPREAD_FACTOR_6)
	{
		/* Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04) */
		cmd_data = (handle->bandwidth << 4) + ((handle->coding_rate + 1) << 1) + 0x01;
		sx1278_write_register(handle, SX1278_LR_RegModemConfig1, &cmd_data, 1);

		cmd_data = ((handle->spread_factor + 6) << 4) + (handle->crc_en << 2) + 0x03;
		sx1278_write_register(handle, SX1278_LR_RegModemConfig2, &cmd_data, 1);

		sx1278_read_register(handle, 0x31, &cmd_data, 1);
		cmd_data &= 0xF8;
		cmd_data |= 0x05;
		sx1278_write_register(handle, 0x31, &cmd_data, 1);

		cmd_data = 0x0C;
		sx1278_write_register(handle, 0x37, &cmd_data, 1);
	}
	else
	{
		/* Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04) */
		cmd_data = (handle->bandwidth << 4) + ((handle->coding_rate) << 1) + 0x00;
		sx1278_write_register(handle, SX1278_LR_RegModemConfig1, &cmd_data, 1);

		cmd_data = (((handle->spread_factor + 6) << 4) + (handle->crc_en << 2) + 0x00);
		sx1278_write_register(handle, SX1278_LR_RegModemConfig2, &cmd_data, 1);
	}

	cmd_data = 0x04;
	sx1278_write_register(handle, SX1278_LR_RegModemConfig3, &cmd_data, 1);

	cmd_data = 0x08;
	sx1278_write_register(handle, SX1278_LR_RegSymbTimeoutLsb, &cmd_data, 1);

	cmd_data = 0x00;
	sx1278_write_register(handle, SX1278_LR_RegPreambleMsb, &cmd_data, 1);

	cmd_data = 0x08;
	sx1278_write_register(handle, SX1278_LR_RegPreambleLsb, &cmd_data, 1);

	cmd_data = 0x01;
	sx1278_write_register(handle, SX1278_LR_REG_DIOMAPPING2, &cmd_data, 1);

	ret = sx1278_set_transceiver_mode(handle, handle->mode);

	return ret;
}

err_code_t sx1278_lora_transmit(sx1278_handle_t handle, uint8_t *data)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if (handle->mode != SX1278_TRANSCEIVER_MODE_TX)
	{
		return ERR_CODE_FAIL;
	}

	sx1278_write_register(handle, SX1278_LR_RegFifo, data, handle->packet_len);

	uint8_t cmd_data = 0x8B;
	sx1278_write_register(handle, SX1278_LR_RegOpMode, &cmd_data, 1);

	return ERR_CODE_SUCCESS;
}

err_code_t sx1278_lora_transmit_polling(sx1278_handle_t handle, uint8_t *data, uint32_t timeout_ms)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if ((handle->mode != SX1278_TRANSCEIVER_MODE_TX) || (handle->get_irq == NULL) || (handle->delay == NULL))
	{
		return ERR_CODE_FAIL;
	}

	sx1278_write_register(handle, SX1278_LR_RegFifo, data, handle->packet_len);

	uint8_t cmd_data = 0x8B;
	sx1278_write_register(handle, SX1278_LR_RegOpMode, &cmd_data, 1);

	uint8_t irq_level;
	while (1)
	{
		handle->get_irq(&irq_level);
		if (irq_level)
		{
			sx1278_lora_clear_irq_flags(handle);

			return ERR_CODE_SUCCESS;
		}

		if (--timeout_ms == 0)
		{
			return ERR_CODE_FAIL;
		}

		handle->delay(1);
	}

	return ERR_CODE_SUCCESS;
}

err_code_t sx1278_lora_receive(sx1278_handle_t handle, uint8_t *data, uint16_t *num_bytes)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if (handle->mode != SX1278_TRANSCEIVER_MODE_RX)
	{
		return ERR_CODE_FAIL;
	}

	uint8_t addr;
	uint8_t packet_size;

	memset(data, 0x00, handle->packet_len);

	/* Get last packet address */
	sx1278_read_register(handle, SX1278_LR_RegFifoRxCurrentaddr, &addr, 1);
	sx1278_write_register(handle, SX1278_LR_RegFifoAddrPtr, &addr, 1);

	/* If spread factor is 6, implicit header mode will be used (excluding internal packet length) */
	if (handle->spread_factor == SX1278_SPREAD_FACTOR_6)
	{
		packet_size = handle->packet_len;
	}
	else
	{
		sx1278_read_register(handle, SX1278_LR_RegRxNbBytes, &packet_size, 1);
	}

	/* Get data */
	sx1278_read_register(handle, SX1278_LR_RegFifo, data, packet_size);

	/* Clear interrupt flags */
	sx1278_lora_clear_irq_flags(handle);

	*num_bytes = packet_size;

	return ERR_CODE_SUCCESS;
}

err_code_t sx1278_lora_receive_polling(sx1278_handle_t handle, uint8_t *data, uint16_t *num_bytes, uint32_t timeout_ms)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if ((handle->mode != SX1278_TRANSCEIVER_MODE_RX) || (handle->get_irq == NULL) || (handle->delay == NULL))
	{
		return ERR_CODE_FAIL;
	}

	uint8_t irq_level;
	uint8_t addr;
	uint8_t packet_size;

	while (1)
	{
		handle->get_irq(&irq_level);
		if (irq_level)
		{
			memset(data, 0x00, handle->packet_len);

			/* Get last packet address */
			sx1278_read_register(handle, SX1278_LR_RegFifoRxCurrentaddr, &addr, 1);
			sx1278_write_register(handle, SX1278_LR_RegFifoAddrPtr, &addr, 1);

			/* If spread factor is 6, implicit header mode will be used (excluding internal packet length) */
			if (handle->spread_factor == SX1278_SPREAD_FACTOR_6)
			{
				packet_size = handle->packet_len;
			}
			else
			{
				sx1278_read_register(handle, SX1278_LR_RegRxNbBytes, &packet_size, 1);
			}

			/* Get data */
			sx1278_read_register(handle, SX1278_LR_RegFifo, data, packet_size);

			/* Clear interrupt flags */
			sx1278_lora_clear_irq_flags(handle);

			*num_bytes = packet_size;

			return ERR_CODE_SUCCESS;
		}

		if (--timeout_ms == 0)
		{
			return ERR_CODE_FAIL;
		}

		handle->delay(1);
	}

	return ERR_CODE_SUCCESS;
}

err_code_t sx1278_lora_clear_irq_flags(sx1278_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t cmd = 0xFF;
	sx1278_write_register(handle, SX1278_LR_RegIrqFlags, &cmd, 1);

	return ERR_CODE_SUCCESS;
}

err_code_t sx1278_lora_get_rssi(sx1278_handle_t handle, uint8_t *rssi)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	sx1278_read_register(handle, SX1278_LR_RegRssiValue, rssi, 1);

	return ERR_CODE_SUCCESS;
}

err_code_t sx1278_set_transceiver_mode(sx1278_handle_t handle, sx1278_transceiver_mode_t mode)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t ret;

	switch (handle->mode)
	{
	case SX1278_TRANSCEIVER_MODE_SLEEP:
		ret = sx1278_enter_sleep(handle);
		break;
	case SX1278_TRANSCEIVER_MODE_STANDBY:
		ret = sx1278_enter_standby(handle);
		break;
	case SX1278_TRANSCEIVER_MODE_FS_TX:
		ret = ERR_CODE_FAIL;
		break;
	case SX1278_TRANSCEIVER_MODE_TX:
		ret = sx1278_enter_lora_transmitter(handle);
		break;
	case SX1278_TRANSCEIVER_MODE_FS_RX:
		ret = ERR_CODE_FAIL;
		break;
	case SX1278_TRANSCEIVER_MODE_RX:
		ret = sx1278_enter_lora_receiver(handle);
		break;
	default:
		ret = sx1278_enter_sleep(handle);
		break;
	}

	return ret;
}

err_code_t sx1278_hw_reset(sx1278_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if (handle->set_rst == NULL)
	{
		return ERR_CODE_FAIL;
	}

	handle->set_rst(SX1278_RST_ACTIVE);
	handle->delay(1);
	handle->set_rst(SX1278_RST_UNACTIVE);

	handle->delay(100);

	return ERR_CODE_SUCCESS;
}
