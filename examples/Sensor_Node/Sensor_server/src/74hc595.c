
#include "74hc595.h"
#include "stdint.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

void HC595_Init(void)
{
    nrf_gpio_cfg_output(LATCH_PIN);
    nrf_gpio_cfg_output(CLOCK_PIN);
    nrf_gpio_cfg_output(DATA_PIN);


    nrf_gpio_pin_write(LATCH_PIN, 0);
    nrf_gpio_pin_write(CLOCK_PIN, 0);
    nrf_gpio_pin_write(DATA_PIN, 0);
}

void HC595_Send_bit(char bit)
{
  nrf_gpio_cfg_output(DATA_PIN);

  nrf_gpio_pin_write(DATA_PIN, 0);

  if(bit == 1)
  {
     nrf_gpio_pin_write(DATA_PIN, 1);
   }

   else
   {
   nrf_gpio_pin_write(DATA_PIN, 0);
    }

    nrf_gpio_pin_write(DATA_PIN, 1);
  
}

void HC595_SendByte(uint8_t byte)
{
    nrf_gpio_cfg_output(LATCH_PIN);
    nrf_gpio_cfg_output(CLOCK_PIN);
    nrf_gpio_cfg_output(DATA_PIN);


    //for(int8_t i =0; i <8; i++)
    //{
    //    unsigned char i;
    //    unsigned char x;

    //    for(i=0; i<8; i++)
    //    {
    //      x = data>>i;
    //      x &=0x01;
    //      HC595_Send_bit(x);
    //    }

    for(int8_t i =7; i >=0; --i)
    {
      uint8_t bit = byte & (0x1 << i);	

      nrf_gpio_pin_write(DATA_PIN, bit);

        // Toggle CLOCK
        nrf_gpio_pin_write(CLOCK_PIN, 1);
        //while(clock_timeout--);
        nrf_gpio_pin_write(CLOCK_PIN, 0);

    }

    // Toggle Latch 
    nrf_gpio_pin_write(LATCH_PIN, 1);
    // while (latch_timeout--);
    nrf_gpio_pin_write(LATCH_PIN, 0);

}