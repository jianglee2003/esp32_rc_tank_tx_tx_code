/*
  File:		  NRF24L01.c
  Author:     jianglee
  Updated:    07-08-2025
  Notice: this Library set the default pipeline to pipe1, in RX Mode, in:
                                        void NRF24_RxMode (uint8_t *Address, uint8_t channel);
  User need to change pipeline if need.
*/

#include "nrf24l01.h"


static spi_device_handle_t nrf_spi_handle = NULL;

/* helpers to control CSN and CE */
static inline void CS_Select(void)   { gpio_set_level(GPIO_NRF24_CSN, 0); }  // active low
static inline void CS_UnSelect(void) { gpio_set_level(GPIO_NRF24_CSN, 1); }
static inline void CE_Enable(void)   { gpio_set_level(GPIO_NRF24_CE, 1); }
static inline void CE_Disable(void)  { gpio_set_level(GPIO_NRF24_CE, 0); }

/* --- low-level SPI transmit/receive helpers --- */
/* transmit-only (no rx) */
static esp_err_t spi_tx(const uint8_t *txbuf, int txlen_bytes) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = 0;
    t.length = txlen_bytes * 8;
    t.tx_buffer = txbuf;
    return spi_device_transmit(nrf_spi_handle, &t);
}

/* full-duplex tx/rx for equal length */
static esp_err_t spi_txrx(const uint8_t *txbuf, uint8_t *rxbuf, int len_bytes) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = 0;
    t.length = len_bytes * 8;
    t.tx_buffer = txbuf;
    t.rx_buffer = rxbuf;
    return spi_device_transmit(nrf_spi_handle, &t);
}

/* transmit then receive: send command byte(s) then read response bytes */
static esp_err_t spi_cmd_then_read(const uint8_t *cmd, int cmd_len, uint8_t *rxbuf, int rx_len) {
    /* We'll send command bytes then receive rx_len bytes. To closely match the STM32
       behavior (separate HAL calls), we'll do two transactions:
       1) tx command (no cs release)
       2) rx data (no cs release)
       But spi_device_transmit toggles CS automatically; because we control CS manually
       this is fine — simply pull CS low before and high after the two transactions. */
    esp_err_t ret;

    CS_Select();
    ret = spi_tx(cmd, cmd_len);
    if (ret != ESP_OK) {
        CS_UnSelect();
        return ret;
    }
    /* Create dummy tx buffer of 0xFF for clocking in rx data */
    static uint8_t dummy[64];
    if (rx_len > (int)sizeof(dummy)) {
        CS_UnSelect();
        return ESP_ERR_INVALID_SIZE;
    }
    memset(dummy, 0xFF, rx_len);
    ret = spi_txrx(dummy, rxbuf, rx_len);
    CS_UnSelect();
    return ret;
}

/* --- ported functions --- */

/* write a single byte to the particular register */
void nrf24_WriteReg (uint8_t Reg, uint8_t Data) {
    uint8_t buf[2];
    buf[0] = (Reg | (1 << 5));  // W_REGISTER | reg
    buf[1] = Data;

    CS_Select();
    esp_err_t ret = spi_tx(buf, 2);
    CS_UnSelect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WriteReg failed: %s", esp_err_to_name(ret));
    }
}

/* write multiple bytes starting from a particular register */
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size) {
    uint8_t cmd = (Reg | (1 << 5));
    CS_Select();
    esp_err_t ret = spi_tx(&cmd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WriteRegMulti cmd tx failed: %s", esp_err_to_name(ret));
        CS_UnSelect();
        return;
    }
    ret = spi_tx(data, size);
    CS_UnSelect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WriteRegMulti data tx failed: %s", esp_err_to_name(ret));
    }
}

/* Read a specific register in NRF24. */
uint8_t nrf24_ReadReg (uint8_t Reg) {
    uint8_t tx = (Reg & 0x1F); /* R_REGISTER | reg (R_REGISTER is 0) */
    uint8_t rx = 0;
    esp_err_t ret = spi_cmd_then_read(&tx, 1, &rx, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ReadReg failed: %s", esp_err_to_name(ret));
        return 0xFF;
    }
    return rx;
}

/* Read multiple bytes from the register */
void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size) {
    uint8_t tx = (Reg & 0x1F);
    esp_err_t ret = spi_cmd_then_read(&tx, 1, data, size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ReadReg_Multi failed: %s", esp_err_to_name(ret));
    }
}

/* send the special command to the NRF. */
void nrfsendCmd (uint8_t cmd) {
    CS_Select();
    esp_err_t ret = spi_tx(&cmd, 1);
    CS_UnSelect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nrfsendCmd failed: %s", esp_err_to_name(ret));
    }
}

/* Reset the NRF24 to default setting.  */
void nrf24_reset(uint8_t REG) {
    if (REG == STATUS) {
        nrf24_WriteReg(STATUS, 0x00);
    }
    else if (REG == FIFO_STATUS) {
        nrf24_WriteReg(FIFO_STATUS, 0x11);
    }
    else {
        nrf24_WriteReg(CONFIG, 0x08);
        nrf24_WriteReg(EN_AA, 0x3F);
        nrf24_WriteReg(EN_RXADDR, 0x03);
        nrf24_WriteReg(SETUP_AW, 0x03);
        nrf24_WriteReg(SETUP_RETR, 0x03);
        nrf24_WriteReg(RF_CH, 0x02);
        nrf24_WriteReg(RF_SETUP, 0x0E);
        nrf24_WriteReg(STATUS, 0x00);
        nrf24_WriteReg(OBSERVE_TX, 0x00);
        nrf24_WriteReg(0x09, 0x00); /* OBSERVE_TX or CD depending on mapping */
        uint8_t rx_addr_p0_def[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
        nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
        uint8_t rx_addr_p1_def[5] = {0xC2,0xC2,0xC2,0xC2,0xC2};
        nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
        nrf24_WriteReg(RX_ADDR_P2, 0xC3);
        nrf24_WriteReg(RX_ADDR_P3, 0xC4);
        nrf24_WriteReg(RX_ADDR_P4, 0xC5);
        nrf24_WriteReg(RX_ADDR_P5, 0xC6);
        uint8_t tx_addr_def[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
        nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
        nrf24_WriteReg(RX_PW_P0, 0);
        nrf24_WriteReg(RX_PW_P1, 0);
        nrf24_WriteReg(RX_PW_P2, 0);
        nrf24_WriteReg(RX_PW_P3, 0);
        nrf24_WriteReg(RX_PW_P4, 0);
        nrf24_WriteReg(RX_PW_P5, 0);
        nrf24_WriteReg(FIFO_STATUS, 0x11);
        nrf24_WriteReg(DYNPD, 0);
        nrf24_WriteReg(FEATURE, 0);
    }
}

/* --- SPI init and helpers (exposed) --- */
esp_err_t setup_nrf24l01_bus(void) {
    esp_err_t ret;

    /* Set up CE and CSN GPIOs */
    gpio_reset_pin(GPIO_NRF24_CE);
    gpio_set_direction(GPIO_NRF24_CE, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_NRF24_CSN);
    gpio_set_direction(GPIO_NRF24_CSN, GPIO_MODE_OUTPUT);

    CE_Disable();
    CS_UnSelect();

    /* SPI bus config */
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_NRF24_MOSI,
        .miso_io_num = GPIO_NRF24_MISO,
        .sclk_io_num = GPIO_NRF24_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64  // adjust if needed
    };

    /* Device config: we set spics_io_num = -1 (manual CS control) */
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 2000000, /* 2 MHz as in your original code */
        .duty_cycle_pos = 128,
        .mode = 0,
        .spics_io_num = -1, /* manual CS */
        .queue_size = 3,
        .flags = 0,
    };

    ret = spi_bus_initialize(NRF_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = spi_bus_add_device(NRF_SPI_HOST, &devcfg, &nrf_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "NRF SPI initialized, device handle: %p", nrf_spi_handle);
    return ESP_OK;
}

/* Convenience wrapper matching your original nrf24l01_init */
void nrf24l01_init(void) {
    esp_err_t ret = setup_nrf24l01_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setup_nrf24l01_bus failed");
        return;
    }
}

/* Common setup for NRF24 (NRF24_Init) */
void NRF24_Init (void) {
    CE_Disable();
    nrf24_reset(0);
    nrf24_WriteReg(CONFIG, 0);  // configured later
    nrf24_WriteReg(EN_AA, 0);  // No Auto ACK
    nrf24_WriteReg(EN_RXADDR, 0); // Not Enabling any data pipe right now, so we can configurate it later in TX/RX Mode.
    nrf24_WriteReg(SETUP_AW, 0x03); // 5 Bytes for the TX/RX address
    nrf24_WriteReg(SETUP_RETR, 0); // No retransmission
    nrf24_WriteReg(RF_CH, 0);  // will be setup during Tx or RX
    nrf24_WriteReg(RF_SETUP, 0x0E); // Power= 0db, data rate = 2Mbps
    CE_Enable();
}

/* Initialize in TX mode */
void NRF24_TxMode (uint8_t *Address, uint8_t channel) {
    CE_Disable();
    nrf24_WriteReg(RF_CH, channel);
    nrf24_WriteRegMulti(TX_ADDR, Address, 5);

    /* power up PWR_UP bit (bit1) */
    uint8_t config = nrf24_ReadReg(CONFIG);
    config = config | (1<<1);
    nrf24_WriteReg(CONFIG, config);
    CE_Enable();
}

/* Transmit payload (32 bytes) */
uint8_t NRF24_Transmit (uint8_t *data) {
    uint8_t cmd = W_TX_PAYLOAD;

    CS_Select();
    esp_err_t ret = spi_tx(&cmd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TX: cmd tx failed: %s", esp_err_to_name(ret));
        CS_UnSelect();
        return FAIL;
    }

    /* Send 32 bytes payload (like original code) */
    ret = spi_tx(data, 32);
    CS_UnSelect();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TX: payload tx failed: %s", esp_err_to_name(ret));
        return FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

    /**
	 * - Check the fourth bit (TX_EMPTY) of FIFO_STATUS register to know if the TX fifo is empty or not:
	 * 		+) 1 - TX FIFO empty.
	 * 		+) 0 - data in TX FIFO.
	 * - Also check the 3th or 2rd bit to assume that the data was successfully transfered or not,
	 * because these two bits are reserved, so the value is always 0. but when we disconnect the device, all the bits in this register will be set,
	 * so it is hard to check if the data was successfully transfered, or the device was disconnected to the MCU.
	 * */
    if ((fifostatus & (1<<4)) && (!(fifostatus & (1<<3))))
    {
        /* flush tx and reset FIFO_STATUS */
        nrfsendCmd(FLUSH_TX);
        nrf24_reset(FIFO_STATUS);
        return SUCCESS;
    } else {
        return FAIL;
    }
}

/* Initialize in RX mode */
void NRF24_RxMode (uint8_t *Address, uint8_t channel) {
    CE_Disable();
    nrf24_reset(STATUS);
    nrf24_WriteReg(RF_CH, channel); // select the channel

    // select data pipe 1
    uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
    en_rxaddr |= (1 << 1); /* enable pipe1 */
    nrf24_WriteReg(EN_RXADDR, en_rxaddr);

    /* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
	 * The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
	 * Their 4 MSB Bytes will still be same as Data Pipe 1
	 *
	 * For Example:
	 * Pipe 1 ADDR = 0xAABBCCDD11
	 * Pipe 2 ADDR = 0xAABBCCDD22
	 * Pipe 3 ADDR = 0xAABBCCDD33
	 */
    // By default, pipe1 will be selected.
    nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5); // Write the Pipe1 address
    nrf24_WriteReg(RX_PW_P1, 32);  // 32 bit payload size for pipe 1.

    /* power up the device in Rx mode, set PWR_UP and PRIM_RX bits (bit1 = PWR_UP, bit0 = PRIM_RX) */ 
    uint8_t config = nrf24_ReadReg(CONFIG);
    config = config | (1 << 1) | (1 << 0);
    nrf24_WriteReg(CONFIG, config);
    CE_Enable();
}

/* Check if data available for reading for a given pipe number */
uint8_t isDataAvailable (int pipenum) {
    uint8_t status = nrf24_ReadReg(STATUS);
    /* pipenum is expected as a pipe number 1..5 in original code bit mask check */
    if ((status & (1<<6)) && (status & (pipenum << 1))) {
        /* clear RX_DR (bit6) by writing 1 to STATUS */
        nrf24_WriteReg(STATUS, (1<<6));
        return SUCCESS;
    }
    return FAIL;
}

/* Read RX payload into data (32 bytes) */
void NRF24_Receive (uint8_t *data) {
    uint8_t cmd = R_RX_PAYLOAD;
    CS_Select();
    esp_err_t ret = spi_tx(&cmd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RX: cmd failed: %s", esp_err_to_name(ret));
        CS_UnSelect();
        return;
    }
    /* read 32 bytes */
    ret = spi_txrx((const uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"
                        "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",
                   data, 32);
    CS_UnSelect();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RX payload read failed: %s", esp_err_to_name(ret));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    nrfsendCmd(FLUSH_RX);
}

/* Read all registers (like original) */
void NRF24_ReadAll (uint8_t *data) {
    for (int i = 0; i < 10; i++) {
        data[i] = nrf24_ReadReg(i);
    }
    nrf24_ReadReg_Multi(RX_ADDR_P0, data + 10, 5);
    nrf24_ReadReg_Multi(RX_ADDR_P1, data + 15, 5);
    data[20] = nrf24_ReadReg(RX_ADDR_P2);
    data[21] = nrf24_ReadReg(RX_ADDR_P3);
    data[22] = nrf24_ReadReg(RX_ADDR_P4);
    data[23] = nrf24_ReadReg(RX_ADDR_P5);
    nrf24_ReadReg_Multi(RX_ADDR_P0, data + 24, 5);
    for (int i = 29; i < 38; i++) {
        data[i] = nrf24_ReadReg(i - 12);
    }
}

