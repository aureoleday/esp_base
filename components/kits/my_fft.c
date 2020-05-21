/*
 * my_fft.c
 *
 *  Created on: 2019Äê3ÔÂ7ÈÕ
 *      Author: Administrator
 */

#define MEOW_FFT_IMPLEMENTAION
#define _USE_MATH_DEFINES
#include "meow_fft.h"
#include <math.h>
#include <stdio.h>
#include <my_fft.h>
//#include "esp_heap_caps.h"

typedef struct
{
	float*					inbuf;
	Meow_FFT_Complex*		out_dbuf_ptr;
	Meow_FFT_Workset_Real*	fft_sbuf_ptr;
	float*					win_buf_ptr;
	uint16_t				ord;
}my_fft_st;

my_fft_st 		my_fft_inst;

static void win_init(uint16_t ord);

int16_t fft_init(void)
{
//	extern float rt_buf[FFT_MAX_ORD];
//	if(my_fft_inst.out_dbuf_ptr != NULL)
//		return -1;
//	else
//	{
//		size_t workset_bytes = meow_fft_generate_workset_real(FFT_MAX_ORD, NULL);
////		my_fft_inst.out_dbuf_ptr = heap_caps_malloc(sizeof(Meow_FFT_Complex) * FFT_MAX_ORD,MALLOC_CAP_SPIRAM);
////		my_fft_inst.fft_sbuf_ptr = (Meow_FFT_Workset_Real*) heap_caps_malloc(workset_bytes,MALLOC_CAP_SPIRAM);
////		my_fft_inst.win_buf_ptr = (float*) heap_caps_malloc(sizeof(float) * FFT_MAX_ORD,MALLOC_CAP_SPIRAM);
//		my_fft_inst.out_dbuf_ptr = malloc(sizeof(Meow_FFT_Complex) * FFT_MAX_ORD);
//		my_fft_inst.fft_sbuf_ptr = (Meow_FFT_Workset_Real*) malloc(workset_bytes);
//		my_fft_inst.win_buf_ptr = (float*) malloc(sizeof(float) * FFT_MAX_ORD);
//		my_fft_inst.inbuf = rt_buf;
//		return 0;
//	}
	return 0;
}

void fft_new(uint16_t ord)
{
	my_fft_inst.ord = ord;
	meow_fft_generate_workset_real(my_fft_inst.ord, my_fft_inst.fft_sbuf_ptr);
	win_init(ord);
}

static void win_init(uint16_t ord)
{
	for(int i=1;i<(ord+1);i++)
		my_fft_inst.win_buf_ptr[i] = (1-cos(2*M_PI*i/(ord+1)))/2;
}

void win_apply(float* inbuf, float* outbuf)
{
	for(int i=0;i<my_fft_inst.ord;i++)
		*(outbuf+i) = *(inbuf+i)*my_fft_inst.win_buf_ptr[i];
}

void fft_calc(float* input_dbuf,float* output_dbuf)
{
	win_apply(input_dbuf,my_fft_inst.inbuf);
	meow_fft_real(my_fft_inst.fft_sbuf_ptr, my_fft_inst.inbuf, my_fft_inst.out_dbuf_ptr);
	for(int i=0;i<my_fft_inst.ord/2;i++)
	{
		*(output_dbuf+i) = sqrt(pow(my_fft_inst.out_dbuf_ptr[i].r,2) + pow(my_fft_inst.out_dbuf_ptr[i].j,2))/(my_fft_inst.ord/2);
	}
}


