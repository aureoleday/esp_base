/*
 * adxl_drv.h
 *
 *  Created on: 2018Äê12ÔÂ27ÈÕ
 *      Author: Administrator
 */

#ifndef COMPONENTS_DRV_ADXL_DRV_H_
#define COMPONENTS_DRV_ADXL_DRV_H_

#include <stdio.h>
#include <my_fft.h>

#define DEV_GEO_RTX_SIZE    512
#define DEV_GEO_FIFO_SIZE   FFT_MAX_ORD

typedef struct
{
	uint8_t		state;
	uint16_t	ibuf_cnt;
    float		ibuf[DEV_GEO_FIFO_SIZE];
    float		obuf[DEV_GEO_FIFO_SIZE];
    float		ampl_arr[16];
    float		freq_arr[16];
    uint16_t	arr_cnt;
}fft_st;

typedef enum {
   ADXL_DEVID_AD = 0x00,
   ADXL_DEVID_MST = 0x01,
   ADXL_PARTID = 0x02,
   ADXL_REVID = 0x03,
   ADXL_STATUS = 0x04,
   ADXL_FIFO_ENTRIES = 0x05,
   ADXL_TEMP2 = 0x06,
   ADXL_TEMP1 = 0x07,
   ADXL_XDATA3 = 0x08,
   ADXL_XDATA2 = 0x09,
   ADXL_XDATA1 = 0x0A,
   ADXL_YDATA3 = 0x0B,
   ADXL_YDATA2 = 0x0C,
   ADXL_YDATA1 = 0x0D,
   ADXL_ZDATA3 = 0x0E,
   ADXL_ZDATA2 = 0x0F,
   ADXL_ZDATA1 = 0x10,
   ADXL_FIFO_DATA = 0x11,
   ADXL_OFFSET_X_H = 0x1E,
   ADXL_OFFSET_X_L = 0x1F,
   ADXL_OFFSET_Y_H = 0x20,
   ADXL_OFFSET_Y_L = 0x21,
   ADXL_OFFSET_Z_H = 0x22,
   ADXL_OFFSET_Z_L = 0x23,
   ADXL_ACT_EN = 0x24,
   ADXL_ACT_THRESH_H = 0x25,
   ADXL_ACT_THRESH_L = 0x26,
   ADXL_ACT_COUNT = 0x27,
   ADXL_FILTER = 0x28,
   ADXL_FIFO_SAMPLES = 0x29,
   ADXL_INT_MAP = 0x2A,
   ADXL_SYNC = 0x2B,
   ADXL_RANGE = 0x2C,
   ADXL_POWER_CTL = 0x2D,
   ADXL_SELF_TEST = 0x2E,
   ADXL_RESET = 0x2F
} ADXL355_register_t;

void adxl_init(void);
void adxl_register(void);
void adxl355_reset(void);
int16_t adxl355_scanfifo(void);
uint8_t adxl_rd_reg(uint8_t addr, uint8_t * rx_buf, uint8_t cnt);
uint8_t adxl_wr_reg(uint8_t addr, uint8_t data);
float* geo_get_fft(uint16_t* sample_cnts);
int16_t geo_get_time(float* dst_ptr,uint16_t len);
int16_t geo_get_fft_peak(float* freq_arr,float* ampl_arr,uint16_t *arr_cnt);


#endif /* COMPONENTS_DRV_ADXL_DRV_H_ */
