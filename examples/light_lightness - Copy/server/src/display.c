#include "display.h"
#include "stdbool.h"
#include "stdlib.h"
#include "74hc595.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

static const uint8_t NumberTable[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
static const uint8_t DP = 0x80;
static const uint8_t MN = 0x40;
static uint8_t digBuffer[4] = {0};
static uint8_t posPoint = 5;

uint8_t digIndex = 0;

void displayInt(int i)
{
    memset(digBuffer, '\0', sizeof(digBuffer));

    digBuffer[3] = (uint8_t)(i / 1000);
	digBuffer[2] = (uint8_t)((i % 1000) / 100);
	digBuffer[1] = (uint8_t)((i % 100) / 10);
	digBuffer[0] = (uint8_t)(i % 10);

	posPoint = 5;
}

void displayFloat(float f)
{
	memset(digBuffer, '\0', sizeof(digBuffer));
	int n = (int) f;
	int d = 0;
	if (n >= 10)
	{
		d = abs((f - n) * 100);
		digBuffer[3] = (uint8_t)(n / 10);
		digBuffer[2] = (uint8_t)(n % 10);
		digBuffer[1] = (uint8_t)(d / 10);
		digBuffer[0] = (uint8_t)(d % 10);
		posPoint = 2;
	}
	else if ((n >= 0) && (n < 10))
	{
		d = abs((f - n) * 1000);
		digBuffer[3] = (uint8_t)(n);
		digBuffer[2] = (uint8_t)(d / 100);
		digBuffer[1] = (uint8_t)((d % 100) / 10);
		digBuffer[0] = (uint8_t)(d % 10);
		posPoint = 3;
	}
	else if ((n < 0) && (n > -10))
	{
		d = abs((f - n) * 100);
		digBuffer[3] = MN;
		digBuffer[2] = (uint8_t)(n);
		digBuffer[1] = (uint8_t)(d / 100);
		digBuffer[0] = (uint8_t)((d % 100) / 10);
		posPoint = 2;
	}
}

void displayInit(void)
{
    HC595_Init();
    nrf_gpio_cfg_output(DIG0_PIN);
    nrf_gpio_cfg_output(DIG1_PIN);
    nrf_gpio_cfg_output(DIG2_PIN);
    nrf_gpio_cfg_output(DIG3_PIN);


    nrf_gpio_pin_clear(DIG0_PIN);
    nrf_gpio_pin_clear(DIG1_PIN);
    nrf_gpio_pin_clear(DIG2_PIN);
    nrf_gpio_pin_clear(DIG3_PIN);

    memset(digBuffer, '\0', sizeof(digBuffer));
}

void displayDigit(uint8_t dig)
{
    nrf_gpio_cfg_output(DIG0_PIN);
    nrf_gpio_cfg_output(DIG1_PIN);
    nrf_gpio_cfg_output(DIG2_PIN);
    nrf_gpio_cfg_output(DIG3_PIN);

    switch (dig)
    {
        case 0: {
            HC595_SendByte(NumberTable[digBuffer[0]]);
            nrf_gpio_pin_set(DIG0_PIN);
            nrf_gpio_pin_clear(DIG1_PIN);
            nrf_gpio_pin_clear(DIG2_PIN);
            nrf_gpio_pin_clear(DIG3_PIN);

            break;
        }
        case 1: {
            HC595_SendByte(NumberTable[digBuffer[1]]);
            nrf_gpio_pin_set(DIG1_PIN);
            nrf_gpio_pin_clear(DIG0_PIN);
            nrf_gpio_pin_clear(DIG2_PIN);
            nrf_gpio_pin_clear(DIG3_PIN);

            break;
        }
        case 2: {
            if (posPoint == 2)
                HC595_SendByte(NumberTable[digBuffer[2]] | DP);
            else
                HC595_SendByte(NumberTable[digBuffer[2]]);
            nrf_gpio_pin_set(DIG2_PIN);
            nrf_gpio_pin_clear(DIG1_PIN);
            nrf_gpio_pin_clear(DIG0_PIN);
            nrf_gpio_pin_clear(DIG3_PIN);

            break;
        }
        case 3: {
            if (posPoint == 3)
                HC595_SendByte(NumberTable[digBuffer[3]] | DP);
            else
                HC595_SendByte(NumberTable[digBuffer[3]]);
            nrf_gpio_pin_set(DIG3_PIN);
            nrf_gpio_pin_clear(DIG1_PIN);
            nrf_gpio_pin_clear(DIG2_PIN);
            nrf_gpio_pin_clear(DIG0_PIN);

            break;
        }
        
    
    default:
        break;
    }

}

void displayStop(void)
{
    nrf_gpio_cfg_output(DIG0_PIN);
    nrf_gpio_cfg_output(DIG1_PIN);
    nrf_gpio_cfg_output(DIG2_PIN);
    nrf_gpio_cfg_output(DIG3_PIN);


    nrf_gpio_pin_clear(DIG0_PIN);
    nrf_gpio_pin_clear(DIG1_PIN);
    nrf_gpio_pin_clear(DIG2_PIN);
    nrf_gpio_pin_clear(DIG3_PIN);


}