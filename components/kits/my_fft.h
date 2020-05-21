/*
 * my_fft.h
 *
 *  Created on: 2019Äê3ÔÂ7ÈÕ
 *      Author: Administrator
 */


#ifndef COMPONENTS_KITS_MY_FFT_H_
#define COMPONENTS_KITS_MY_FFT_H_

#include "sys_def.h"

#define FFT_MAX_ORD 1024

int16_t fft_init(void);
void fft_new(uint16_t ord);
void fft_calc(float* input_dbuf,float* output_dbuf);

#endif /* COMPONENTS_KITS_MY_FFT_H_ */
