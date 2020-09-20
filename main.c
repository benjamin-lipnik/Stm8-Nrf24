#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "stm8s003.h"
#include "spi.h"
#include "uart.h"
#include "nRF24L01.h"

void printbin(uint8_t dat) {
  for(uint8_t i = 0; i < 8; i++) {
    uart_write(48 + ((dat&_BV(i))>0));
  }
}

void delay_milliseconds(unsigned long milliseconds) { //for 16Mhz
	//delay(milliseconds);
  for(uint8_t m = 0; m < milliseconds; m++) {
    for(uint8_t i = 0; i < 160; i++) {
      for(uint8_t j = 0; j < 10; j++) {
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
      }
    }
  }
}

#define CSN_PIN SPI_CS_PIN

#define CONFIG_REG_SETTINGS_FOR_RX_MODE (_BV(PWR_UP) | _BV(PRIM_RX) | _BV(EN_CRC))
#define RF_TX_PWR_MAX  0b110

#define PAYLOAD_SIZE 5
uint8_t payload_buffer[PAYLOAD_SIZE];

void write_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
  SPI_chip_select();
  SPI_transfer(W_REGISTER | (REGISTER_MASK & reg));
  while(size--) {
    SPI_transfer(*(buffer++));
  }
  SPI_chip_deselect();
}
void read_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
  SPI_chip_select();
  SPI_transfer(R_REGISTER | (REGISTER_MASK & reg));
  while(size--) {
    *(buffer++) = SPI_transfer(0xff);
  }
  SPI_chip_deselect();
}
void read_payload_data (uint8_t command, uint8_t * buffer, uint8_t size) {
  SPI_chip_select();
  SPI_transfer(command);
  while(size--) {
    *(buffer++) = SPI_transfer(0xff);
  }
  SPI_chip_deselect();
}
void flush_tx() {
  SPI_chip_select();
  SPI_transfer(FLUSH_TX);
  SPI_chip_deselect();
}
void flush_rx() {
  SPI_chip_select();
  SPI_transfer(FLUSH_RX);
  SPI_chip_deselect();
}
uint8_t get_status () {
  SPI_chip_select();
  SPI_transfer(R_REGISTER | (REGISTER_MASK & STATUS_NRF));
  uint8_t ret = SPI_transfer(0xff);
  SPI_chip_deselect();
  return ret;
}
void clear_status () {
  SPI_chip_select();
  SPI_transfer(W_REGISTER | (REGISTER_MASK & STATUS_NRF));
  SPI_transfer(_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
  SPI_chip_deselect();
}

int putchar(int c) {
    uart_write(c);
    return 0;
}

int main () {
    CLK_CKDIVR = 0;//16mhz
    //enable_interrupts();

    uart_init(9600);

    PC_DDR |= _BV(CSN_PIN);
    PC_CR1 |= _BV(CSN_PIN);
    PC_ODR |= _BV(CSN_PIN);

    //PC_DDR |= _BV(CE_PIN);
    //PC_CR1 |= _BV(CE_PIN);

    SPI_init();

    while(1) {
      printf("Initting radio.\n\r");

      delay_milliseconds(100); //OFF_TO_POWERDOWN_MILLIS

      uint8_t channel = 100;
      write_register_block(RF_CH, &channel, 1);

      uint8_t rf_setup = RF_TX_PWR_MAX | _BV(3);
      write_register_block(RF_SETUP, &rf_setup, 1);

      uint8_t address[] = {7,7,7,7,7};
      write_register_block(RX_ADDR_P1, address, 5);

      uint8_t dynpd = _BV(DPL_P0) | _BV(DPL_P1);
      write_register_block(DYNPD, &dynpd, 1);

      uint8_t feature = _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK);
      write_register_block(FEATURE, &feature, 1);

      flush_rx();
      flush_tx();

      clear_status();

      uint8_t config = CONFIG_REG_SETTINGS_FOR_RX_MODE;
      write_register_block(CONFIG, &config, 1);

      delay_milliseconds(5);

      uint8_t config_check = 0;
      read_register_block(CONFIG, &config_check, 1);

      if(config_check == CONFIG_REG_SETTINGS_FOR_RX_MODE) {
        break;
      }
    }

    printf("Init successufull!!!\n\r");

    while(1) {
      uint8_t status = get_status();
      //printf("status: %02x\n\r", status);
      //printf("status: "); printbin(status);
      //printf("\n\r");

      if((status & 0b1110) == 0b1110) {//no data

        printf("No sig.\n\r");

        delay_milliseconds(40);

        continue;
      }
      else {
        read_payload_data(R_RX_PAYLOAD, payload_buffer, PAYLOAD_SIZE);

        if(get_status() & _BV(RX_DR)) {
          clear_status();
        }

        printf("data: ");
        for(uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
          printf("%u, ", payload_buffer[i]);
        }
        printf("\n\r");

        delay_milliseconds(1);
      }
    }
}
