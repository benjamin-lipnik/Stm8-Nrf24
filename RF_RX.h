#ifndef RF_RX_H
#define RF_RX_H

#include "nRF24L01.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm8s003.h"
#include "spi.h"

void delay_milliseconds(unsigned long milliseconds);
void writeRegister(uint8_t reg, uint8_t data);
uint8_t readRegister(uint8_t reg);
void ce(uint8_t state);
void csn(uint8_t state);

void preInit (void * spi_handle);
uint8_t initRadio (uint8_t * receive_address, uint8_t bitrate, uint8_t channel);
uint8_t hasData();
void readData (void * data, uint8_t size);


#define CE_PIN  3
#define CSN_PIN 4

#define OFF_TO_POWERDOWN_MILLIS        100
#define POWERDOWN_TO_RXTX_MODE_MILLIS  5
#define CE_TRANSMISSION_MICROS         10
#define CONFIG_REG_SETTINGS_FOR_RX_MODE (_BV(PWR_UP) | _BV(PRIM_RX) | _BV(EN_CRC))

#define BITRATE250KBPS  0b00100000
#define BITRATE1MBPS    0b00000000
#define BITRATE2MBPS    0b00001000

#define RF_TX_PWR_MAX  0b110
#define RF_TX_PWR_HIGH 0b100
#define RF_TX_PWR_LOW  0b010
#define RF_TX_PWR_MIN  0b000

#ifndef _BV
#define _BV(bit)			       (1<<(bit))
#endif


#define flushTx()            writeRegister(FLUSH_TX, 0xFF)
#define flushRx()            writeRegister(FLUSH_RX, 0xFF)
#define getStatus()          readRegister(STATUS_NRF)
#define resetStatus()        writeRegister(STATUS_NRF, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT))
#define setChannel(channel)  writeRegister(RF_CH,channel)




#endif
