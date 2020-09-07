/*
 * goertzel.c
 *
 *  Created on: 2019Äê6ÔÂ6ÈÕ
 *      Author: Administrator
 */
//#include "esp_system.h"
//#include "freertos/FreeRTOS.h"
//#include "esp_console.h"
#include <stdlib.h>     /* qsort */
#include <math.h>
#include "sys_conf.h"

#define FREQ_SPAN_MAX 16 
#define GTZ_FLOWS_MAX 16 

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

static float calc_snr(float* dbuf, uint16_t cnt, snr_sts_st* snr_sts_ptr)
{
    float buf[FREQ_SPAN_MAX];
    float buf_2nd[4];
    float signal_psd=0.0;
    float noise_psd=0.0;
    float snr = 0.0;

    uint16_t i;
    int16_t ind = 0;
    uint16_t mid = cnt/2;

    for(i=0;i<cnt;i++)
    {
        buf[i] = *(dbuf+i);
    }

    qsort(buf,cnt,sizeof(float),compare);
    for(i=0;i<cnt;i++)
        if(buf[0] == dbuf[i])
            break;
    snr_sts_ptr->offset = i - mid;

    buf_2nd[0] = *(dbuf+mid-1);
    buf_2nd[1] = *(dbuf+mid);
    buf_2nd[2] = *(dbuf+mid+1);
    buf_2nd[3] = buf[3];

    qsort(buf_2nd,4,sizeof(float),compare);

    for(i=0;i<4;i++)
    {
        if(buf_2nd[i]<=buf[3])
        {
            ind = i;
            break;
        }
    }

    if(ind != 0)
    {
        for(i=0;i<ind;i++)
            signal_psd += buf_2nd[i]*buf_2nd[i];
        signal_psd = sqrtf(signal_psd);
    }
    else
        signal_psd = *(dbuf+mid);

    for(i=ind;i<cnt;i++)
        noise_psd += buf[i];
    noise_psd /=(cnt-ind);

    snr = signal_psd/noise_psd;

    snr_sts_ptr->signal_level = signal_psd;
    snr_sts_ptr->noise_level = noise_psd;
    snr_sts_ptr->rank = ind;
    snr_sts_ptr->snr = snr;

    return snr;
}

static int32_t gtz_snr(float* dbuf, uint16_t cnt)
{
    extern sys_reg_st  g_sys;
    uint16_t i;
   
    printf("\n");
    for(i=0;i<(2*g_sys.conf.gtz.target_span+1);i++)
    {
        //printf(" %f ",gtz_inst.res[i]); 
        printf(" %f ",*(dbuf+i)); 
    }
    printf("\n");
    return 0;
}

static void goertzel_coef_init(void)
{
    extern sys_reg_st  g_sys;
    int i;
    uint32_t gtz_n;
    gtz_n = 1<<g_sys.conf.gtz.n;
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
   
    //x = din * window(gtz_inst.icnt,g_sys.conf.gtz.n);
    n = 1<<g_sys.conf.gtz.intv;
    gtz_n = 1<<g_sys.conf.gtz.n; 
    x = din; 
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
            gtz_snr(gtz_inst.res[i],2*g_sys.conf.gtz.target_span+1);
            printf("--%d--\n",i);
            gtz_inst.icnt[i]=0;
            ret = 1;
        }
    }
    return ret;
}

float goertzel_calc(float* din)
{
    extern sys_reg_st  g_sys;
    
    float coef = goertzel_coef(g_sys.conf.gtz.target_freq,g_sys.conf.gtz.sample_freq, (1<<g_sys.conf.gtz.n));
    float q0 = 0.0;
    float q1 = 0.0;
    float q2 = 0.0;

    uint32_t i=0;
    float x = 0.0;

    for(i=0;i<(2<<g_sys.conf.gtz.n);i++)
    {
        x = *(din+i); 
        q0 = coef * q1 - q2 + x;
        q2 = q1;
        q1 = q0;
    }
    return sqrtf((q1*q1 + q2*q2 - q1*q2*coef)*2/(1<<g_sys.conf.gtz.n));
}

int32_t gtz_freq_bins(float* dst_buf, uint16_t *num)
{
    extern sys_reg_st  g_sys;
    uint16_t i;
    *num = 2*g_sys.conf.gtz.target_span+1;
    for(i=0;i<*num;i++)
        *(dst_buf+i) = gtz_inst.res[0][i];
    return 0;
}

void goertzel_init(void)
{
    extern sys_reg_st  g_sys;
    int32_t i,n;

    n = 1<<g_sys.conf.gtz.intv;
    
    for(i=0;i<n;i++)
    {
        gtz_inst.icnt[i]=i*(1<<(g_sys.conf.gtz.n-g_sys.conf.gtz.intv));
        //printf("icnt[%d]:%d\n",i,gtz_inst.icnt[i]);
    }
    goertzel_coef_init();
}

