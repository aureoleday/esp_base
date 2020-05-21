#ifndef _REG_MAP_CHECK_H
#define _REG_MAP_CHECK_H

#include "sys_conf.h"
uint16_t dhcp_trigger(uint32_t pram);
//uint16_t sys_reset_opt(uint32_t pram);
uint16_t set_boot_opt(uint32_t pram);
uint16_t save_conf_opt(uint32_t pram);
uint16_t load_conf_opt(uint32_t pram);
uint16_t set_timestamp(uint32_t pram);
uint16_t tcp_timer_opt(uint32_t pram);
uint16_t geo_timer_opt(uint32_t pram);
uint16_t geo_pwr_opt(uint32_t pram);
uint16_t geo_filter_opt(uint32_t pram);
uint16_t mod_en_opt(uint32_t pram);
uint16_t mod_volum_opt(uint32_t pram);
uint16_t mod_freq_opt(uint32_t pram);
uint16_t gtz_rst_opt(uint32_t pram);
#endif
