/*
 * ads131_drv.h
 *
 *  Created on: 2018Äê12ÔÂ27ÈÕ
 *      Author: Administrator
 */

#ifndef COMPONENTS_DRV_ADS131_DRV_H_
#define COMPONENTS_DRV_ADS131_DRV_H_

#include <stdio.h>

#define ADC_FIFO_SIZE 16384 

//typedef struct
//{
//	uint8_t		state;
//	uint16_t	ibuf_cnt;
//    float		ampl_arr[16];
//    float		freq_arr[16];
//    uint16_t	arr_cnt;
//}fft_st;

typedef enum {
   ADC_DEVID_MSB = 0x00,
   ADC_DEVID_LSB = 0x01
} ADC_register_t;

void adc_init(void);
void adc_reset(void);
//void adc_tim_start(int32_t tim_period);
//void adc_tim_stop(void);
int adc_dout(uint8_t * dst_ptr, uint16_t max_len);
uint16_t adc_set_sps(uint16_t freq_mode);
uint16_t adc_rd_reg(uint8_t addr);
uint16_t adc_wr_reg(uint8_t addr, uint8_t data);
int adc_dout(uint8_t * dst_ptr, uint16_t max_len);

#endif /* COMPONENTS_DRV_ADC_DRV_H_ */
