#include "brf24.h"
#include "spi.h"
#include "stm8_utility.h"

/*INTERFACES*/
void nrf24_delay_milliseconds(uint8_t ms) {
  util_delay_milliseconds(ms);
}
void nrf24_chip_select() {
  SPI_chip_select();
}
void nrf24_chip_deselect() {
  SPI_chip_deselect();
}
void nrf24_spi_write(uint8_t data) {
  SPI_transfer(data);
}
uint8_t nrf24_spi_read() {
  return SPI_transfer(0xff);
}
//You should init spi and or csn pin
uint8_t nrf24_spi_init() {}

/*INTERNAL*/

/*PRIVATE FUNCTIONS*/
void nrf24_write_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
  nrf24_chip_select();
  nrf24_spi_write(W_REGISTER | (REGISTER_MASK & reg));
  while(size--) {
    nrf24_spi_write(*(buffer++));
  }
  nrf24_chip_deselect();
}
void nrf24_read_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
  nrf24_chip_select();
  nrf24_spi_write(R_REGISTER | (REGISTER_MASK & reg));
  while(size--) {
    *(buffer++) = nrf24_spi_read();
  }
  nrf24_chip_deselect();
}
void nrf24_read_payload_data (uint8_t command, uint8_t * buffer, uint8_t size) {
  nrf24_chip_select();
  nrf24_spi_write(command);
  while(size--) {
    *(buffer++) = nrf24_spi_read();
  }
  nrf24_chip_deselect();
}
void nrf24_flush_tx() {
  nrf24_chip_select();
  nrf24_spi_write(FLUSH_TX);
  nrf24_chip_deselect();
}
void nrf24_flush_rx() {
  nrf24_chip_select();
  nrf24_spi_write(FLUSH_RX);
  nrf24_chip_deselect();
}
uint8_t nrf24_get_status () {
  nrf24_chip_select();
  nrf24_spi_write(R_REGISTER | (REGISTER_MASK & STATUS_NRF));
  uint8_t ret = nrf24_spi_read();
  nrf24_chip_deselect();
  return ret;
}
void nrf24_clear_status () {
  nrf24_chip_select();
  nrf24_spi_write(W_REGISTER | (REGISTER_MASK & STATUS_NRF));
  nrf24_spi_write(_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
  nrf24_chip_deselect();
}

/*PUBLIC FUNCTIONS*/
uint8_t nrf24_init(uint8_t * rx_address, uint8_t channel) {

  nrf24_spi_init();


  nrf24_delay_milliseconds(100); //OFF_TO_POWERDOWN_MILLIS

  //uint8_t channel = 100;
  nrf24_write_register_block(RF_CH, &channel, 1);

  uint8_t rf_setup = RF_TX_PWR_MAX | BITRATE2MBPS;
  nrf24_write_register_block(RF_SETUP, &rf_setup, 1);

  //uint8_t address[] = {7,7,7,7,7};
  nrf24_write_register_block(RX_ADDR_P1, rx_address, 5);

  uint8_t dynpd = _BV(DPL_P0) | _BV(DPL_P1);
  nrf24_write_register_block(DYNPD, &dynpd, 1);

  uint8_t feature = _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK);
  nrf24_write_register_block(FEATURE, &feature, 1);

  nrf24_flush_rx();
  nrf24_flush_tx();

  nrf24_clear_status();

  uint8_t config = CONFIG_REG_SETTINGS_FOR_RX_MODE;
  nrf24_write_register_block(CONFIG, &config, 1);

  nrf24_delay_milliseconds(5);

  uint8_t config_check = 0;
  nrf24_read_register_block(CONFIG, &config_check, 1);

  return config_check == CONFIG_REG_SETTINGS_FOR_RX_MODE;
}
uint8_t nrf24_has_rx_data() {
  return (nrf24_get_status() & 0b1110) != 0b1110;
}
void nrf24_read_data (void * data_buffer, uint8_t size) {
  uint8_t * buffer = data_buffer;
  nrf24_chip_select();
  nrf24_spi_write(R_RX_PAYLOAD);
  while(size--) {
    *(buffer++) = nrf24_spi_read();
  }
  nrf24_chip_deselect();

  if(nrf24_get_status() & _BV(RX_DR)) {
    nrf24_clear_status();
  }
}
