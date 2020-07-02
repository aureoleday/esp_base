#include "sys_def.h" 
#include "reg_map_check.h"
#include "global_var.h"
#include "mqtt_tcp.h"
#include "cmd_wifi.h"
#include "daq.h"
#include "adxl_drv.h"

extern sys_reg_st g_sys;

uint16_t daq_pkg_en(uint32_t pram)
{  
    if(pram == 1)
       daq_tim_start(g_sys.conf.daq.pkg_period);
    else
       daq_tim_stop();
    return 1;
}

uint16_t geo_sample_en(uint32_t pram)
{  
    if(pram == 1)
    {
       adxl_wr_reg(ADXL_FILTER,g_sys.conf.geo.filter);
       adxl_wr_reg(ADXL_POWER_CTL,0);
       adxl_tim_start(g_sys.conf.geo.scan_period);
    }
    else
    {
       adxl_wr_reg(ADXL_POWER_CTL,1);
       adxl_tim_stop();
    }
    return 1;
}

uint16_t set_mqtt_con_opt(uint32_t pram)
{
    if(pram == 1)
    {    
        mqtt_connect();
    }
    else
    {
        mqtt_disconnect();
    }
    return 1;
}

uint16_t set_wifi_con_opt(uint32_t pram)
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

uint16_t save_conf_opt(uint32_t pram)
{  
    if(pram == 1)
    {
        save_conf("usr");
    }
    return 1;
}

uint16_t load_conf_opt(uint32_t pram)
{
    if(pram == 1)
    {
        load_conf("x");
        save_conf("usr");
    }
    return 1;
}



