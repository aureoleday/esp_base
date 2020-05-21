#ifndef __GLOBAL_VAR
#define	__GLOBAL_VAR
#include "sys_conf.h"

uint16_t reg_map_write(uint16_t reg_addr, uint32_t *wr_data, uint8_t wr_cnt);
uint16_t reg_map_read(uint16_t reg_addr, uint32_t* reg_data, uint8_t read_cnt);
uint16_t sys_local_var_init(void);
void gvar_register(void);
void init_load_status(void);
uint8_t reset_runtime(uint16_t param);
int save_conf(const char *save_type);
int load_conf(const char *load_type);
uint8_t load_factory_pram(void);
int32_t gvar_init(void);
void start_mdns_service(void);
int get_wifi_info(char* ssid, char* lcssid, char* pwd, char* lcpwd, size_t* s_len, size_t* ls_len, size_t* p_len, size_t* lp_len);
#endif //__GLOBAL_VAR
