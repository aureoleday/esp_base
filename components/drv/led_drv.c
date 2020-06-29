/*
 * led.c
 *
 *  Created on: 2019Äê5ÔÂ6ÈÕ
 *      Author: Administrator
 */
#include "driver/gpio.h"

#define PIN_LED_INT 	32
#define PIN_LED_EXT 	33

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<PIN_LED_INT) | (1ULL<<PIN_LED_EXT))

void usr_led_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(PIN_LED_INT, 0);
    gpio_set_level(PIN_LED_EXT, 1);
}

void set_led(uint8_t pin_num, uint8_t value)
{
    if(pin_num == 0)
    {
        gpio_set_level(PIN_LED_INT,value);
    }
    else if(pin_num == 1)
    {
        gpio_set_level(PIN_LED_EXT,value);
    }
}

void toggle_led(uint8_t pin_num)
{
    esp_err_t value;
    if(pin_num == 0)
    {
        value = gpio_get_level(PIN_LED_INT);
        if(value != 0)
            gpio_set_level(PIN_LED_INT, 0);
        else
            gpio_set_level(PIN_LED_INT, 1);
    }
    else if(pin_num == 1)
    {
        value = gpio_get_level(PIN_LED_EXT);
        if(value != 0)
            gpio_set_level(PIN_LED_EXT, 0);
        else
            gpio_set_level(PIN_LED_EXT, 1);
    }
}

