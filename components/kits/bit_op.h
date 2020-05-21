#ifndef __BIT_OP_H
#define	__BIT_OP_H
#include "sys_def.h"

void bit_op_set(uint32_t *data, uint16_t offset, uint8_t option);
int32_t bit_op_get(const uint32_t data, uint16_t offset);

#endif	//__BIT_OP_H

