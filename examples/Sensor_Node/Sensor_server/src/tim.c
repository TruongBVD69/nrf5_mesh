
#include "nrf_delay.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"

#include "display.h"
#include "tim.h"

//APP_TIMER_DEF(m_app_timer_id_1);

//void lfclk_config(void)
//{
//  ret_code_t err_code = nrf_drv_clock_init();
//  APP_ERROR_CHECK(err_code);

//  nrf_drv_clock_lfclk_request(NULL);
//}

//static void app_timer_handler(void * p_context)
//{
//  displayDigit(digIndex++);

//  if(digIndex >= 4) digIndex = 0;
//}

//void timers_init(void)
//{
//  ret_code_t err_code;

//  err_code = app_timer_init();

//  APP_ERROR_CHECK(err_code);

//  err_code = app_timer_create(&m_app_timer_id_1, APP_TIMER_MODE_REPEATED, app_timer_handler);

//  APP_ERROR_CHECK(err_code);

//}