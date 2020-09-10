#ifndef __SYS_CONF
#define	__SYS_CONF
#include "sys_def.h"

//application delay
#define		INIT_THREAD_DELAY           1
#define		CMD_THREAD_DELAY            400
#define		WS_THREAD_DELAY           	500
#define		BKG_THREAD_DELAY            700
#define		GEO_THREAD_DELAY            800
#define		TCPCLIENT_THREAD_DELAY      600
#define		SAMPLE_CHANNEL_MAX          4

typedef struct
{	
    uint32_t    restart;
}conf_gen_st;


typedef struct
{	
    uint32_t    wifi_connect;
    uint32_t    wifi_mode;
}conf_con_st;

typedef struct
{
    uint32_t    enable;
    uint32_t    pkg_en;
    uint32_t    sample_period;
    uint32_t    sample_channel_bm;
    uint32_t    pga_gain[SAMPLE_CHANNEL_MAX];
    uint32_t    pkg_period;
    uint32_t    pkg_size;
    uint32_t    filter;
}conf_daq_st;

typedef struct
{
    uint32_t    framing_type;
    uint32_t    framing_period;
    uint32_t    samples_per_frame;
}conf_framing_st;

typedef struct
{
    uint32_t    mav_cnt;
    uint32_t    up_lim;
    uint32_t    low_lim;
}conf_bat_st;

typedef struct
{
    uint32_t    http_en;
    uint32_t    mqtt_en;
    uint32_t    tcp_en;
    uint32_t    service_bm;
}conf_prt_st;

typedef struct
{
    uint32_t    n;
    uint32_t    win;
    uint32_t    acc_times;
    uint32_t    intv_cnts;
}conf_fft_st;

typedef struct
{
    uint32_t    calc_n;
    uint32_t    n;
    uint32_t    intv;
    uint32_t    target_freq;
    uint32_t    sample_freq;
    uint32_t    target_span;
    uint32_t    signal_th;
    uint32_t    en;
    uint32_t    res_cd;
}conf_gtz_st;

typedef struct
{
    uint32_t    adc_offset;
    uint32_t    dac_offset;
    uint32_t    dac_setval;
}conf_per_st;

typedef struct
{
    uint32_t    enable;
    uint32_t    filter;
    uint32_t    sps;
    uint32_t    gain;
    uint32_t    ch_bm;
    uint32_t    drop;
}conf_adc_st;

typedef struct
{
    uint32_t    enable;
    uint32_t    pkg_en;
    uint32_t    pkg_period;
    uint32_t    scan_period;
    uint32_t    filter;
    uint32_t    sample_rate;
    uint32_t    axis;
    uint32_t    gain;
}conf_geo_st;

typedef struct
{
    uint32_t    enable;
    uint32_t	mod_freq_off;
    uint32_t	symbol_period;
    uint32_t    volum;
    uint32_t    freq;
    uint32_t    setup_time;
    uint32_t    hold_time;
}conf_mod_st;

typedef struct
{
    conf_gen_st gen;
    conf_daq_st daq;
    conf_con_st con;
    conf_prt_st prt;
    conf_geo_st geo;
    conf_adc_st adc;
    conf_per_st per;
    conf_mod_st mod;
    conf_fft_st fft;
    conf_gtz_st gtz;
	conf_bat_st bat;
}config_st;


typedef struct
{	    
    uint32_t    status_reg_num;
    uint32_t    config_reg_num;
    uint32_t    software_ver;
    uint32_t    hardware_ver;
    uint32_t    status_bm;
}stat_gen_st;

typedef struct
{
    uint32_t    serial_no;
    uint32_t    man_date;
    uint32_t    dev_type;
}stat_man_st;

typedef struct
{
    uint32_t   	adc_raw;
    uint32_t   	pwr_val;
    uint32_t   	pwr_sts;
}stat_bat_st;

typedef struct
{
    uint32_t   	drop_cnt;
    uint32_t   	rms;
}stat_adc_st;

typedef struct
{
    uint32_t   	kfifo_drop_cnt;
}stat_geo_st;

typedef struct
{
    float	   	freq_bar[33];
    float	   	snr;
    uint32_t    snr_i;
    uint32_t    slv_i;
    uint32_t    nlv_i;
    uint32_t    res_cd;
    uint32_t    res_snr_i;
    uint32_t    res_slv_i;
    uint32_t    res_nlv_i;
    float	   	snr_f;
    float		slv_f;
    float		nlv_f;
}stat_gtz_st;

typedef struct
{
    stat_gen_st 	gen;
    stat_man_st 	man;
    stat_geo_st		geo;
    stat_adc_st		adc;
    stat_gtz_st		gtz;
    stat_bat_st		bat;
}status_st;

typedef struct
{
    config_st   conf;
    status_st   stat;
}sys_reg_st;

typedef struct 
{
    uint16_t 	id;
    uint32_t*	reg_ptr;
    int32_t	    min;
    uint32_t	max;
    uint32_t	dft;
    uint8_t	    type;                     //0:RW, 1:WO
    int16_t     (*chk_ptr)(uint32_t pram);
}conf_reg_map_st;

typedef struct 
{
    uint16_t 	id;
    uint32_t*	reg_ptr;
    uint32_t  	dft;
}sts_reg_map_st;

uint16_t sys_global_var_init(void);

#endif //	__SYS_CONF




