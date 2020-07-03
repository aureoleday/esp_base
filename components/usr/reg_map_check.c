#include "sys_def.h" 
#include "reg_map_check.h"
#include "global_var.h"
#include "mqtt_tcp.h"
#include "cmd_wifi.h"
#include "daq.h"
#include "adxl_drv.h"
#include "bit_op.h"

extern sys_reg_st g_sys;

int16_t daq_pkg_en(uint32_t pram)
{  
    uint16_t ret = 0;
    if((pram == 1)&&((0 == bit_op_get(g_sys.stat.gen.status_bm,GBM_DAQ))))
    {
        daq_tim_start(g_sys.conf.daq.pkg_period);
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_DAQ,1);
        ret = 1;
    }
    else if((pram == 0)&&((1 == bit_op_get(g_sys.stat.gen.status_bm,GBM_DAQ))))
    {
        daq_tim_stop();
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_DAQ,0);
        ret = 0;
    }
    return ret;
}

int16_t geo_pkg_en(uint32_t pram)
{  
    uint16_t ret = 0;
    if((pram == 1)&&((0 == bit_op_get(g_sys.stat.gen.status_bm,GBM_GEO))))
    {
        adxl_wr_reg(ADXL_FILTER,(g_sys.conf.geo.filter<<4)|g_sys.conf.geo.sample_rate);
        adxl_wr_reg(ADXL_POWER_CTL,0);
        adxl_tim_start(g_sys.conf.geo.scan_period);
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_GEO,1);
        ret = 1;
    }
    else if((pram == 0)&&((1 == bit_op_get(g_sys.stat.gen.status_bm,GBM_GEO))))
    {
        adxl_wr_reg(ADXL_POWER_CTL,1);
        adxl_tim_stop();
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_GEO,0);
        ret = 0;
    }
    return ret;
}

int16_t service_opt(uint32_t pram)
{
    if(bit_op_get(g_sys.conf.prt.service_bm,SERVICE_TCP)&&(0 == bit_op_get(g_sys.stat.gen.status_bm,GBM_TCP)))
    {
        tcp_srv_start();
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_TCP,1);
    }

    if(bit_op_get(g_sys.conf.prt.service_bm,SERVICE_MQTT)&&(0==bit_op_get(g_sys.stat.gen.status_bm,GBM_MQTT)))
    {
        mqtt_connect();
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_MQTT,1);
    }
    else if(0==bit_op_get(g_sys.conf.prt.service_bm,SERVICE_MQTT)&&bit_op_get(g_sys.stat.gen.status_bm,GBM_MQTT))
    {
        mqtt_disconnect();
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_MQTT,0);
    }

    if(bit_op_get(g_sys.conf.prt.service_bm,SERVICE_HTTP)&&(0==bit_op_get(g_sys.stat.gen.status_bm,GBM_HTTP)))
    {}
        
    daq_pkg_en(g_sys.conf.daq.pkg_en);

    geo_pkg_en(g_sys.conf.geo.pkg_en);
    return 1;
}

int16_t set_wifi_con_opt(uint32_t pram)
{
    if(pram == 1)
    {
        wifi_connect();
    }
    else
    {
        wifi_disconnect();
    }
    return 1;
}

int16_t save_conf_opt(uint32_t pram)
{  
    if(pram == 1)
    {
        save_conf("usr");
    }
    return 1;
}

int16_t load_conf_opt(uint32_t pram)
{
    if(pram == 1)
    {
        load_conf("x");
        save_conf("usr");
    }
    return 1;
}



