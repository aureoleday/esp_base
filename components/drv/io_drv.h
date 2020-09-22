/*
 * pb_drv.h
 *
 *  Created on: 2019Äê5ÔÂ6ÈÕ
 *      Author: Administrator
 */

#ifndef COMPONENTS_DRV_IO_DRV_H_
#define COMPONENTS_DRV_IO_DRV_H_

void io_init(void);
void pwr_fsm_thread(void* param);
void pga_gain(uint8_t gain_ind);
#endif /* COMPONENTS_DRV_IO_DRV_H_ */
