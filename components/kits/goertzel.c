/*
 * goertzel.c
 *
 *  Created on: 2019Äê6ÔÂ6ÈÕ
 *      Author: Administrator
 */
//#include "esp_system.h"
//#include "freertos/FreeRTOS.h"
//#include "esp_console.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdlib.h>     /* qsort */
#include <math.h>
#include "sys_conf.h"
#include "esp_log.h"

static const char *TAG = "GTZ";

#define FREQ_SPAN_MAX 16 
#define GTZ_FLOWS_MAX 64 

typedef struct
{
    float 		coef[FREQ_SPAN_MAX];
    float 		q0[GTZ_FLOWS_MAX][FREQ_SPAN_MAX];
    float 		q1[GTZ_FLOWS_MAX][FREQ_SPAN_MAX];
    float 		q2[GTZ_FLOWS_MAX][FREQ_SPAN_MAX];
    float		res[GTZ_FLOWS_MAX][FREQ_SPAN_MAX];
    uint32_t 	icnt[GTZ_FLOWS_MAX];
}gtz_st;

typedef struct
{
    float		snr;
    float 		signal_level;
    float 		noise_level;
    int16_t		offset;
    uint16_t 	rank;
}snr_sts_st;

typedef struct
{
    float		freq_bins[FREQ_SPAN_MAX];
}snr_buf_st;


gtz_st gtz_inst;
snr_buf_st snr_buf_inst;

static float goertzel_coef(uint32_t target_freq, uint32_t sample_freq, uint32_t N)
{
    uint32_t k;
    float w;
    k = N*target_freq/sample_freq;
    w = (2*M_PI*k)/N;
    return 2*cosf(w);
}

inline static float window(uint32_t n, uint32_t ord)
{
    return (1-cosf(2*M_PI*n/(ord-1)))/2;
}


int compare(const void * a, const void * b)
{
    float fa =  *(float*)a;
    float fb =  *(float*)b;
    return (fa-fb)>0? -1:1;
}

static float calc_snr(float* dbuf, uint16_t cnt)
{
    extern sys_reg_st  g_sys;
    float buf[2*FREQ_SPAN_MAX];

    uint16_t i;
    for(i=0;i<cnt;i++)
        buf[i] = *(dbuf+i);

    qsort(buf,cnt,sizeof(float),compare);
    g_sys.stat.gtz.signal_level = *(dbuf+(cnt>>1));
    if((buf[0]-*(dbuf+(cnt>>1))) < 0.0000001)
        g_sys.stat.gtz.noise_level = (*(buf+(cnt>>1)) + *(buf+(cnt>>1)+1))/2;
    else
        g_sys.stat.gtz.noise_level = *(buf+(cnt>>1)); 
    g_sys.stat.gtz.snr = g_sys.stat.gtz.signal_level/g_sys.stat.gtz.noise_level;

    //for(i=0;i<cnt;i++)
    //    ESP_LOGD(TAG," %f ",buf[i]);
    ESP_LOGD(TAG,"sl:%f, nl:%f, snr:%f",g_sys.stat.gtz.signal_level,g_sys.stat.gtz.noise_level,g_sys.stat.gtz.snr);
    return g_sys.stat.gtz.snr;
}


static void goertzel_coef_init(void)
{
    extern sys_reg_st  g_sys;
    int i;
    uint32_t gtz_n;
    gtz_n = g_sys.conf.gtz.n;
    for(i=0;i<(2*g_sys.conf.gtz.target_span+1);i++)
    {
        gtz_inst.coef[i] = goertzel_coef(g_sys.conf.gtz.target_freq-g_sys.conf.gtz.target_span+i,g_sys.conf.gtz.sample_freq, gtz_n);
    }
 
}
int16_t goertzel_lfilt(float din)
{
    extern sys_reg_st  g_sys;
    float x = 0.0;
    uint32_t n = 0;
    uint32_t gtz_n = 0;
    int16_t ret = 0;
    uint32_t i,j;
   
    n = 1<<g_sys.conf.gtz.intv;
    gtz_n = g_sys.conf.gtz.n; 
    //gtz_n = 1<<g_sys.conf.gtz.n; 
    x = din; 
    //x = din * window(gtz_inst.icnt[0],g_sys.conf.gtz.n);
    for(i=0;i<n;i++)
    {
        for(j=0;j<(2*g_sys.conf.gtz.target_span+1);j++)
        {
            gtz_inst.q0[i][j] = gtz_inst.coef[j] * gtz_inst.q1[i][j] - gtz_inst.q2[i][j] + x;
            gtz_inst.q2[i][j] = gtz_inst.q1[i][j];
            gtz_inst.q1[i][j] = gtz_inst.q0[i][j];
        }
        ret = 0;
        gtz_inst.icnt[i]++;
        if(gtz_inst.icnt[i] >= gtz_n)
        {
            for(j=0;j<(2*g_sys.conf.gtz.target_span+1);j++)
            {
                gtz_inst.res[i][j] = sqrtf(gtz_inst.q1[i][j]*gtz_inst.q1[i][j] + gtz_inst.q2[i][j]*gtz_inst.q2[i][j] - gtz_inst.q1[i][j]*gtz_inst.q2[i][j]*gtz_inst.coef[j])/(gtz_n>>1);
                gtz_inst.q1[i][j] = 0.0;
                gtz_inst.q2[i][j] = 0.0;
            }
            calc_snr(gtz_inst.res[i],2*g_sys.conf.gtz.target_span+1);
            ESP_LOGD(TAG,"--%d--",i);
            gtz_inst.icnt[i]=0;
            ret = 1;
        }
    }
    return ret;
}

//float goertzel_calc(float* din)
//{
//    extern sys_reg_st  g_sys;
//    uint32_t gtz_n = 0;
//    gtz_n = g_sys.conf.gtz.n; 
//    float coef; 
//
//    coef = goertzel_coef(g_sys.conf.gtz.target_freq,g_sys.conf.gtz.sample_freq, gtz_n);
//    float q0 = 0.0;
//    float q1 = 0.0;
//    float q2 = 0.0;
//
//    uint32_t i=0;
//    float x = 0.0;
//
//    for(i=0;i<(2<<g_sys.conf.gtz.n);i++)
//    {
//        x = *(din+i); 
//        q0 = coef * q1 - q2 + x;
//        q2 = q1;
//        q1 = q0;
//    }
//    return sqrtf((q1*q1 + q2*q2 - q1*q2*coef)*2/(1<<g_sys.conf.gtz.n));
//}

//int32_t gtz_freq_bins(float* dst_buf, uint16_t *num)
//{
//    extern sys_reg_st  g_sys;
//    uint16_t i;
//    *num = 2*g_sys.conf.gtz.target_span+1;
//    for(i=0;i<*num;i++)
//        *(dst_buf+i) = gtz_inst.res[0][i];
//    return 0;
//}

void goertzel_init(void)
{
    extern sys_reg_st  g_sys;
    int32_t i,n;
    uint32_t gtz_gap = 0;

    gtz_gap = g_sys.conf.gtz.n >> g_sys.conf.gtz.intv;
    
    n = 1<<g_sys.conf.gtz.intv;
    
    for(i=0;i<n;i++)
    {
        gtz_inst.icnt[i]=i*gtz_gap;
        //ESP_LOGD(TAG,"icnt[%d]:%d",i,gtz_inst.icnt[i]);
    }
    goertzel_coef_init();
}

