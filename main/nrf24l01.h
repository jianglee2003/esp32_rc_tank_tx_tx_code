/*
  File:		  NRF24L01.h
  Author:     jianglee
  Updated:    07-08-2025
*/

#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "stdio.h"
#include "stdio.h"
#include "stdint.h"
#include "stddef.h"
#include "string.h"
#include "sys/unistd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_timer.h"

#define TAG                 "NRF24"

#define GPIO_NRF24_MOSI     23
#define GPIO_NRF24_MISO     19
#define GPIO_NRF24_SCLK     18
#define GPIO_NRF24_CSN      5
#define GPIO_NRF24_CE       4 // Chip enable pin, in NRF module.

/* SPI host choice: use SPI3_HOST (alias of VSPI_HOST works too) */
#define NRF_SPI_HOST        SPI3_HOST

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

#define SUCCESS       (1)
#define FAIL          (0)

/**
 * @brief Sets up the SPI bus and attaches the NRF24L01 device.
 * @note Without this, no communication over SPI will work.
 * @note this function MUST BE CALLED before anything else.
 */
esp_err_t setup_nrf24l01_bus(void);

/**
 * @brief Common setup for the NRF24.
 * @note This Function only initializes the most basic config param for the NRF24.
 * @note This Function MUST BE CALLED AFTER nrf24l01_init().
 * @note User should modify this base on using purpose.
 */
void NRF24_Init (void);

/**
 * @brief Setup the TX Mode.
 * @param[in] Address Pointer to address of receiver pipe.
 * @param[in] channel channel number.
 */
void NRF24_TxMode (uint8_t *Address, uint8_t channel);

/**
 * @brief Transmit a string.
 * @param[in] *data pointer to data buffer.
 * @note Maximum length: 32-Bytes.
 */
uint8_t NRF24_Transmit (uint8_t *data);

/**
 * @brief Setup the RX Mode.
 * @param[in] Address Pointer to address of receiver pipe. By default, pipe1 will be selected.
 * @param[in] channel channel number.
 */
void NRF24_RxMode (uint8_t *Address, uint8_t channel);

/**
 * @brief Check if there is any data in FIFO Buffer of specific data pipe.
 * @param[in] pipenum pipeline index number.
 * @return SUCCESS (1) - there is received data in the pipeline.
 * @return FAIL (0) - no data received in the pipeline.
 */
uint8_t isDataAvailable (int pipenum);

/**
 * @brief Read data from RX FIFO Buffer.
 * @param[out] *data pointer to buffer which will be used to store data.
 */
void NRF24_Receive (uint8_t *data);

/**
 * @brief Read all the data register.
 * @param[out] *data pointer to buffer which will be used to store data.
 */
void NRF24_ReadAll (uint8_t *data);

#endif /* INC_NRF24L01_H_ */

