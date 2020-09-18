#include "stm8s003.h"
#include "spi.h"

void SPI_chip_select() {
  PC_ODR &= ~(1 << SPI_CS_PIN);
}
void SPI_chip_deselect() {
  while ((SPI_SR & (1 << SPI_SR_BSY)));
  PC_ODR |= (1 << SPI_CS_PIN);
}

void SPI_init() {
    /* Initialize CS pin */
    PC_DDR |= (1 << SPI_CS_PIN);
    PC_CR1 |= (1 << SPI_CS_PIN);
    PC_ODR |= (1 << SPI_CS_PIN);
    /* Initialize SPI master at 500kHz  */
    SPI_CR2 = (1 << SPI_CR2_SSM) | (1 << SPI_CR2_SSI);
    SPI_CR1 = (1 << SPI_CR1_MSTR) | (1 << SPI_CR1_SPE) | (1 << SPI_CR1_BR0);
}

void SPI_write(uint8_t data) {
    SPI_DR = data;
    while (!(SPI_SR & (1 << SPI_SR_TXE)));
}
uint8_t SPI_read() {
  //uint8_t ret = SPI_DR;
  SPI_write(0xff);
  while (!(SPI_SR & (1 << SPI_SR_RXNE)));
  return SPI_DR;
}
