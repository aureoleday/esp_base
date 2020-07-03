#ifndef _REG_MAP_CHECK_H
#define _REG_MAP_CHECK_H

#include "sys_conf.h"
int16_t daq_pkg_en(uint32_t pram);
int16_t service_opt(uint32_t pram);
int16_t geo_pkg_en(uint32_t pram);
int16_t set_wifi_con_opt(uint32_t pram);
int16_t set_boot_opt(uint32_t pram);
int16_t save_conf_opt(uint32_t pram);
int16_t load_conf_opt(uint32_t pram);
int16_t geo_timer_opt(uint32_t pram);
int16_t geo_pwr_opt(uint32_t pram);
int16_t geo_filter_opt(uint32_t pram);
int16_t gtz_rst_opt(uint32_t pram);
#endif
