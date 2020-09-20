#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "stm8s003.h"
#include "spi.h"
#include "uart.h"
#include "stm8_utility.h"
#include "brf24.h"

int putchar(int c) {
    uart_write(c);
    return 0;
}

#define PAYLOAD_SIZE 5
uint8_t channel = 100;
uint8_t address[] = {7,7,7,7,7};


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

    while(!nrf24_init(address, channel)) {
      printf("rf24 init error. Trying again.\n\r");
      util_delay_milliseconds(10);
    }

    while(1) {
      if(nrf24_has_rx_data()) {
        uint8_t data[PAYLOAD_SIZE] = {0};
        nrf24_read_data(data, PAYLOAD_SIZE);
        printf("Data: ");
        for(uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
          printf("%u, ", data[i]);
        }
        printf("\n\r");
      }
      else {
        printf("No signal.\n\r");
      }
      util_delay_milliseconds(10);
    }

}
