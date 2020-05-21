#include "bit_op.h"

void bit_op_set(uint32_t *data, uint16_t offset, uint8_t option) 
{		
		if(option)
		{
				*data |= (0x00000001 << offset);
		}
		else
		{
				*data &= (~(0x00000001 << offset));
		}
}

int32_t bit_op_get(const uint32_t data, uint16_t offset) 
{
    uint8_t ret;
    if(data&(0x00000001 << offset))
        ret = 1;
    else
        ret = 0;
    return ret;
}
