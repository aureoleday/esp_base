#ifndef __GLOBAL_VAR
#define	__GLOBAL_VAR
#include "sys_conf.h"

uint16_t reg_map_write(uint16_t reg_addr, uint32_t *wr_data, uint8_t wr_cnt);
uint16_t reg_map_read(uint16_t reg_addr, uint32_t* reg_data, uint8_t read_cnt);
int32_t gvar_init(void);
void gvar_register(void);
int save_conf(const char *save_type);
int load_conf(const char *load_type);

#endif //__GLOBAL_VAR
