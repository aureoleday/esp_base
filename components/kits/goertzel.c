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
#define GTZ_FLOWS_MAX 128 
#define R_QBUF_MAX    1024 

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
    uint64_t 	snr_slv[R_QBUF_MAX];
    uint32_t 	slv[R_QBUF_MAX];
    uint16_t 	cnt;
}rqueue_st;

gtz_st gtz_inst;
rqueue_st rqueue_inst;

static void rqueue_init(void)
{
    int i;
    for(i=0;i<R_QBUF_MAX;i++)
    {
        rqueue_inst.snr_slv[i] = 0;
        rqueue_inst.slv[i] = 0;
    }
    rqueue_inst.cnt = 0;         
}

static int16_t rqueue_append(uint32_t snr, uint32_t slv)
{
    uint64_t snr_t = snr;

    uint64_t temp = (snr_t<<32)|slv; 
    if(rqueue_inst.cnt < R_QBUF_MAX-1) {
        rqueue_inst.snr_slv[rqueue_inst.cnt] = temp;
        rqueue_inst.cnt++;
        return rqueue_inst.cnt;
    }
    else
        return -1;
}

static int compare_uint64(const void * a, const void * b)
{
    uint64_t ua =  *(uint64_t *)a;
    uint64_t ub =  *(uint64_t *)b;
    if(ua < ub)
        return 1;
    else if(ua > ub)
        return -1;
    else
        return 0;
    //return (ua-ub)>0? -1:1;
}

static int16_t rqueue_qsort(void)
{
    int16_t lsnr;
    int16_t i;
    extern sys_reg_st  g_sys;
    qsort(rqueue_inst.snr_slv,rqueue_inst.cnt,sizeof(uint64_t),compare_uint64);
    
    lsnr = g_sys.conf.gtz.snr_th*10;
    //printf("lsnr:%d\n",lsnr);
    for(i=0;i<rqueue_inst.cnt;i++)
    {
        //temp = (int)((rqueue_inst.snr_slv[rqueue_inst.cnt-i-1]>>32)&0x00000000ffffffff);
        //printf("d: %d ",temp);
        if((int)((rqueue_inst.snr_slv[i]>>32)&0x00000000ffffffff) < lsnr) 
            break;
        else
            rqueue_inst.slv[i] = rqueue_inst.snr_slv[i]&0x00000000ffffffff;
    }
    qsort(rqueue_inst.slv,i,sizeof(uint64_t),compare_uint64);
    return i;
    //return rqueue_inst.cnt-i;
}

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


int compare_float(const void * a, const void * b)
{
    float fa =  *(float*)a;
    float fb =  *(float*)b;
    return (fa-fb)>0? -1:1;
}

static void calc_snr(float* dbuf, uint16_t cnt)
{
    extern sys_reg_st  g_sys;
    int16_t vind;
    float buf[2*FREQ_SPAN_MAX];

    uint16_t i;
    for(i=0;i<cnt;i++)
        buf[i] = *(dbuf+i);

    qsort(buf,cnt,sizeof(float),compare_float);
    g_sys.stat.gtz.slv_f= *(dbuf+(cnt>>1));
//    g_sys.stat.gtz.nlv_f = *(buf+cnt-1); 
    if((buf[0]-*(dbuf+(cnt>>1))) < 0.0000001)
        g_sys.stat.gtz.nlv_f = (*(buf+(cnt>>1)) + *(buf+(cnt>>1)+1))/2;
    else
        g_sys.stat.gtz.nlv_f = *(buf+(cnt>>1)); 
    g_sys.stat.gtz.snr_f = g_sys.stat.gtz.slv_f/g_sys.stat.gtz.nlv_f;
    g_sys.stat.gtz.slv_i = (uint32_t)(g_sys.stat.gtz.slv_f*10000000);
    g_sys.stat.gtz.nlv_i = (uint32_t)(g_sys.stat.gtz.nlv_f*10000000);
    g_sys.stat.gtz.snr_i = (uint32_t)(g_sys.stat.gtz.snr_f*100);
    if(g_sys.stat.gtz.res_cd > 0)
    {
        if(0 > rqueue_append(g_sys.stat.gtz.snr_i,g_sys.stat.gtz.slv_i))
            ESP_LOGW(TAG,"rqueue full");
        g_sys.stat.gtz.res_cd--;
        printf("\rres_cd:%2d\t",g_sys.stat.gtz.res_cd);
        fflush(stdout);
        if(g_sys.stat.gtz.res_cd == 0)
        {
            printf("\n");
            vind = rqueue_qsort();
            //g_sys.stat.gtz.res_snr_i = (rqueue_inst.snr_slv[g_sys.conf.gtz.res_pos]>>32)&0x00000000ffffffff; 
            //g_sys.stat.gtz.res_slv_i = rqueue_inst.snr_slv[g_sys.conf.gtz.res_pos]&0x00000000ffffffff; 
            g_sys.stat.gtz.res_snr_i = (rqueue_inst.snr_slv[vind>>1]>>32)&0x00000000ffffffff; 
            g_sys.stat.gtz.res_slv_i = rqueue_inst.slv[vind>>1]; 
            ESP_LOGI(TAG,"final snr:%d,slv:%d,vind:%d",g_sys.stat.gtz.res_snr_i,g_sys.stat.gtz.res_slv_i,vind);
            rqueue_init();
        }
    }
    ESP_LOGD(TAG,"slv:%f, nlv:%f, snr:%f",g_sys.stat.gtz.slv_f,
                                          g_sys.stat.gtz.nlv_f,
                                          g_sys.stat.gtz.snr_f);
}

static void gtz_gap_init(uint32_t gtz_n, uint16_t intv)
{
    extern sys_reg_st  g_sys;
    int32_t i,n;
    uint32_t gtz_gap;

    gtz_gap = gtz_n >> intv;
    
    n = 1<<intv;
    
    for(i=0;i<n;i++)
    {
        gtz_inst.icnt[i]=i*gtz_gap;
    }
}

static void goertzel_coef_init(uint32_t gtz_n, uint16_t span)
{
    extern sys_reg_st  g_sys;
    int i;
    //gtz_n = (g_sys.conf.gtz.n<<mfactor);
    for(i=0;i<(2*span+1);i++)
    {
        gtz_inst.coef[i] = goertzel_coef(g_sys.conf.gtz.target_freq-(g_sys.conf.gtz.target_span<<g_sys.conf.gtz.span_gap)+(i<<g_sys.conf.gtz.span_gap),g_sys.conf.gtz.sample_freq, gtz_n);
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
    
    if(g_sys.conf.gtz.en != 1)
        return -1;

    n = 1<<g_sys.conf.gtz.intv;
    gtz_n = g_sys.conf.gtz.n; 
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
            calc_snr(gtz_inst.res[i],2*g_sys.conf.gtz.target_span+1);
            ESP_LOGD(TAG,"Flow id:%d",i);
            gtz_inst.icnt[i]=0;
            ret = 1;
        }
    }
    return ret;
}

void gtz_reset(uint32_t gtz_n, uint16_t span, uint16_t intv)
{
    goertzel_coef_init(gtz_n,span);
    gtz_gap_init(gtz_n,intv);
}

void goertzel_init(void)
{
    extern sys_reg_st  g_sys;
    rqueue_init();
    gtz_reset(g_sys.conf.gtz.n,g_sys.conf.gtz.target_span,g_sys.conf.gtz.intv);
    esp_log_level_set(TAG,3);
}

