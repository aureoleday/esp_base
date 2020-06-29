/*
 * led_drv.h
 *
 *  Created on: 2019Äê5ÔÂ6ÈÕ
 *      Author: Administrator
 */

#ifndef COMPONENTS_DRV_LED_DRV_H_
#define COMPONENTS_DRV_LED_DRV_H_

void usr_led_init(void);
void set_led(uint8_t pin_num, uint8_t value);
void toggle_led(uint8_t pin_num);

#endif /* COMPONENTS_DRV_LED_DRV_H_ */
