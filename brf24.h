#ifndef BRF24_H
#define BRF24_H

#include <stdint.h>
#include "nRF24L01.h"

#define CSN_PIN 4

#ifndef _BV
  #define _BV(bit) (1<<(bit))
#endif

#define PAYLOAD_SIZE 5
#define CONFIG_REG_SETTINGS_FOR_RX_MODE (_BV(PWR_UP) | _BV(PRIM_RX) | _BV(EN_CRC))
#define RF_TX_PWR_MAX  0b110


uint8_t nrf24_init(uint8_t * rx_address, uint8_t channel);
uint8_t nrf24_has_rx_data();
void nrf24_read_data (void * data_buffer, uint8_t size);

extern uint8_t payload_buffer[PAYLOAD_SIZE];

#endif
