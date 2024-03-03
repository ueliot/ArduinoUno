
// This compile ok with Arduino AND GPIO #define of the input pin works:

#include <driver/mcpwm.h>
#include "esp_clk.h"
//#include "driver/mcpwm.h".  //not work here
//#include <soc/mcpwm_reg.h>
//#include <driver/mcpwm_struct.h>

//#include “mcpwm_periph.h”
//#include “esp_attr.h”
#define GPIO_CAP0_IN 35 //Set GPIO 25 as input CAP0
//=========================================================================
void setup(void)
{
Serial.begin(115200);
//mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_OUT);m

mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN); // MAGIC LINE to define WHICH GPIO

// gpio_pulldown_en(GPIO_CAP0_IN); //Enable pull down on CAP0 signal

//mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
}
//=========================================================================
void loop()
{

mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
uint32_t LOW_VALUE = mcpwm_capture_signal_get_value(MCPWM_UNIT_0,MCPWM_SELECT_CAP0);
float pulse_low_us = LOW_VALUE * (1000000.0 / esp_clk_apb_freq());

//vTaskDelay(pdMS_TO_TICKS(100)); 

mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
uint32_t HIGH_VALUE = mcpwm_capture_signal_get_value(MCPWM_UNIT_0,MCPWM_SELECT_CAP0);
//float pulse_high_us = LOW_VALUE * (1000000.0 / esp_clk_apb_freq());
float pulse_high_us = LOW_VALUE /( esp_clk_apb_freq());

//float pulso =(LOW_VALUE -HIGH_VALUE)/(esp_clk_apb_freq());
uint32_t pulso =( HIGH_VALUE - LOW_VALUE) ;

Serial.println(pulse_low_us);
Serial.println(pulse_high_us);
Serial.println(pulso);
vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1000 milliseconds

}
