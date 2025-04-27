
#include "74hc595.h"
#include "stdint.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

void HC595_Init(void)
{
    nrf_gpio_cfg_output(LATCH_PIN);
    nrf_gpio_cfg_output(CLOCK_PIN);
    nrf_gpio_cfg_output(DATA_PIN);


    nrf_gpio_pin_clear(LATCH_PIN);
    nrf_gpio_pin_clear(CLOCK_PIN);
    nrf_gpio_pin_clear(DATA_PIN);
}

void HC595_SendByte(uint8_t byte)
{
    nrf_gpio_cfg_output(LATCH_PIN);
    nrf_gpio_cfg_output(CLOCK_PIN);
    nrf_gpio_cfg_output(DATA_PIN);


    for(int8_t i =7; i >= 0; --i)
    {
        uint8_t bit = byte & (0x1 << i);
        if(bit == 0)
        {
            nrf_gpio_pin_clear(DATA_PIN);
        }
        else if(bit == 1)
        {
            nrf_gpio_pin_set(DATA_PIN);
        }

        // Toggle CLOCK
        nrf_gpio_pin_set(CLOCK_PIN);
        //while(clock_timeout--);
        nrf_gpio_pin_clear(CLOCK_PIN);

    }

    // Toggle Latch 
    nrf_gpio_pin_set(LATCH_PIN);
    // while (latch_timeout--);
    nrf_gpio_pin_clear(LATCH_PIN);
    nrf_delay_us(100);
}