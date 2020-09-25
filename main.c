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

//!!!! PWM !!!!
//PD4 -> DIR, PD3 -> PWM
#define DIR_PIN 4
#define PWM_PIN 3


#define PAYLOAD_SIZE 5
uint8_t channel = 100;
uint8_t address[] = {7,7,7,7,7};

uint8_t no_sig_counter = 0;

void set_mot_power (uint8_t power, uint8_t dir) {
  if(dir) {
    PD_ODR &= ~_BV(DIR_PIN);
    TIM2_CCR2H = 0;
    TIM2_CCR2L = power; //Duty
    //printf("%u\n\r", power);
  }
  else {
    PD_ODR |= _BV(DIR_PIN);
    TIM2_CCR2H = 0;
    TIM2_CCR2L = 255-power; //Duty
    //printf("-%u\n\r", power);
  }
}

int main () {
    CLK_CKDIVR = 0;//16mhz
    enable_interrupts();

    uart_init(9600);

    PC_DDR |= _BV(CSN_PIN);
    PC_CR1 |= _BV(CSN_PIN);
    PC_ODR |= _BV(CSN_PIN);

    SPI_init();


    //DIR pin
    PD_DDR |= _BV(4);
    PD_CR1 |= _BV(4);
    //pwm pin
    PD_DDR |= _BV(3);
    PD_CR1 |= _BV(3);

    //Timer

    TIM2_PSCR = 0b11;

    TIM2_ARRH = 0;
    TIM2_ARRL = 255; //Max / Top

    //TIM2_IER |= _BV(TIM2_IER_UIE);
    TIM2_CR1 |= _BV(TIM2_CR1_CEN);

    TIM2_CCMR2 |= _BV(5) | _BV(6) | _BV(3); //pwm mode 1 //bita 0 in 1 moreta bit na nic ce ces met izhod
    TIM2_CCER1 |= _BV(4);

    TIM2_CCR2H = 0;
    TIM2_CCR2L = 0; //Duty

    //PD_ODR |= _BV(DIR_PIN);

    while(!nrf24_init(address, channel)) {
      printf("rf24 init error. Trying again.\n\r");
      util_delay_milliseconds(10);
    }

    //PD_ODR |= _BV(4);

    while(1) {
      if(nrf24_has_rx_data()) {
        uint8_t data[PAYLOAD_SIZE] = {0};
        nrf24_read_data(data, PAYLOAD_SIZE);
        //printf("Data: ");
        //for(uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
        //  printf("%u, ", data[i]);
        //}
        //printf("\n\r");
        uint8_t dir = data[0] > 127;
        uint8_t power = 0;

        if(dir) {
          data[0]--;

          power = (data[0]-127)*2;
        }
        else {
          power = (127-data[0])*2;
        }

        set_mot_power(power, dir);
        no_sig_counter = 0;

        //printf("%d\n\r", data[0] - 127);

      }
      else {
        if(++no_sig_counter == 255) {
          set_mot_power(0, 1);
        }
        //printf("No signal.\n\r");
      }
      util_delay_milliseconds(1);
    }

}
