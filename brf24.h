#ifndef BRF24_H
#define BRF24_H

#include <stdint.h>

uint8_t nrf24_init(uint8_t * rx_address, uint8_t channel);
uint8_t nrf24_has_rx_data();
void nrf24_read_data (void * data_buffer, uint8_t size);


#endif
