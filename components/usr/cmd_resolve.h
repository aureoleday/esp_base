#ifndef __CMD_H__
#define __CMD_H__
#include "sys_def.h"

int cmd_stream_in(void * src_data, int data_len);
//int cmd_pkg_out(void *src_data, int data_len, int (* transmitt)(void*,int));
int daq_frame(const void *dbuf_ptr,int d_len);
void cmd_thread(void* param);
#endif //__CMD_H__
