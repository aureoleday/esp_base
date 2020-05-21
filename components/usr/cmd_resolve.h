#ifndef __CMD_H__
#define __CMD_H__
#include "sys_def.h"

void cmd_dev_init(void);
void recv_frame_fsm(void);
uint16_t cmd_frame_recv(void);
uint16_t cmd_frame_resolve(void);
uint16_t report_data(void);

#endif //__CMD_H__
