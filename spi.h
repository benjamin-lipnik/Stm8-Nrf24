#ifndef STM8_SPI_H
#define STM8_SPI_H

#include "stm8s003.h"
#include <stdint.h>

#define SPI_CS_PIN 4

void SPI_chip_select();
void SPI_chip_deselect();
void SPI_init();
uint8_t SPI_transfer(uint8_t data);

#endif
