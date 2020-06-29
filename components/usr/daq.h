#ifndef __DAQ_H__
#define __DAQ_H__
#include "sys_def.h"

void daq_init(void);
void daq_tim_start(int32_t tim_period);
void daq_tim_stop(void);

#endif //__DAQ_H__
