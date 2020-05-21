#include "fifo.h"
#include "sys_def.h"

uint32_t fifo32_init(fifo32_cb_td* fifo_cb, uint16_t block_size, uint16_t depth)
{
    uint32_t *mem_ptr;
    mem_ptr = (uint32_t*)malloc(block_size*depth*4);
    if(mem_ptr == NULL)
        return 0;
    fifo_cb->buffer_ptr = mem_ptr;
    fifo_cb->bhead_ptr = mem_ptr;
    fifo_cb->btail_ptr = mem_ptr;
    fifo_cb->block_size = block_size;
    fifo_cb->depth = depth;
    fifo_cb->length = 0;
    return 1;
}

uint32_t	is_fifo32_full(fifo32_cb_td* fifo_cb)
{
    if(fifo_cb->length == fifo_cb->depth)
        return 1;
    else
        return 0;
}

uint32_t	is_fifo32_empty(fifo32_cb_td* fifo_cb)
{
    if(fifo_cb->length == 0)
        return 1;
    else
        return 0;
}

uint32_t	get_fifo32_length(fifo32_cb_td* fifo_cb)
{
    return fifo_cb->length;
}

uint32_t	fifo32_push(fifo32_cb_td* fifo_cb, uint32_t* src_addr)
{
    if(is_fifo32_full(fifo_cb)==1)
        return 0;
    else
    {
        memcpy(fifo_cb->bhead_ptr, src_addr, fifo_cb->block_size*4);
        if(fifo_cb->bhead_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
            fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
        else
            fifo_cb->bhead_ptr += fifo_cb->block_size;
        fifo_cb->length ++;
        return 1;
    }
}

uint32_t	fifo32_pop(fifo32_cb_td* fifo_cb, uint32_t* dest_addr)
{
    if(is_fifo32_empty(fifo_cb)==1)
        return 0;
    else
    {
        memcpy(dest_addr, fifo_cb->btail_ptr, fifo_cb->block_size*4);
        if(fifo_cb->btail_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
            fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
        else
            fifo_cb->btail_ptr += fifo_cb->block_size;
        fifo_cb->length --;
        return 1;
    }
}

void	fifo32_reset(fifo32_cb_td* fifo_cb)
{
    fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
    fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
    fifo_cb->length = 0;
}


uint8_t fifo16_init(fifo16_cb_td* fifo_cb, uint16_t block_size, uint16_t depth)
{
    uint16_t *mem_ptr;
    mem_ptr = (uint16_t*)malloc(block_size*depth*2);
    if(mem_ptr == NULL)
        return 0;
    fifo_cb->buffer_ptr = mem_ptr;
    fifo_cb->bhead_ptr = mem_ptr;
    fifo_cb->btail_ptr = mem_ptr;
    fifo_cb->block_size = block_size;
    fifo_cb->depth = depth;
    fifo_cb->length = 0;
    return 1;
}

uint8_t	is_fifo16_full(fifo16_cb_td* fifo_cb)
{
    if(fifo_cb->length == fifo_cb->depth)
        return 1;
    else
        return 0;
}

uint8_t	is_fifo16_empty(fifo16_cb_td* fifo_cb)
{
    if(fifo_cb->length == 0)
        return 1;
    else
        return 0;
}

uint16_t	get_fifo16_length(fifo16_cb_td* fifo_cb)
{
    return fifo_cb->length;
}

uint8_t	fifo16_push(fifo16_cb_td* fifo_cb, uint16_t* src_addr)
{
    if(is_fifo16_full(fifo_cb)==1)
        return 0;
    else
    {
        memcpy(fifo_cb->bhead_ptr, src_addr, fifo_cb->block_size*2);
        if(fifo_cb->bhead_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
            fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
        else
            fifo_cb->bhead_ptr += fifo_cb->block_size;
        fifo_cb->length ++;
        return 1;
    }
}


uint8_t	fifo16_pop(fifo16_cb_td* fifo_cb, uint16_t* dest_addr)
{
    if(is_fifo16_empty(fifo_cb)==1)
        return 0;
    else
    {
        memcpy(dest_addr, fifo_cb->btail_ptr, fifo_cb->block_size*2);
        if(fifo_cb->btail_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
            fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
        else
            fifo_cb->btail_ptr += fifo_cb->block_size;
        fifo_cb->length --;
        return 1;
    }
}

void	fifo16_reset(fifo16_cb_td* fifo_cb)
{
    fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
    fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
    fifo_cb->length = 0;
}

uint8_t fifo8_init(fifo8_cb_td* fifo_cb, uint16_t block_size, uint16_t depth)
{
    uint8_t *mem_ptr;
    mem_ptr = (uint8_t*)malloc(block_size*depth);
    if(mem_ptr == NULL)
        return 0;
    fifo_cb->buffer_ptr = mem_ptr;
    fifo_cb->bhead_ptr = mem_ptr;
    fifo_cb->btail_ptr = mem_ptr;
    fifo_cb->block_size = block_size;
    fifo_cb->depth = depth;
    fifo_cb->length = 0;
    return 1;
}

uint8_t	is_fifo8_full(fifo8_cb_td* fifo_cb)
{
    if(fifo_cb->length == fifo_cb->depth)
        return 1;
    else
        return 0;
}

uint8_t	is_fifo8_empty(fifo8_cb_td* fifo_cb)
{
    if(fifo_cb->length == 0)
        return 1;
    else
        return 0;
}

uint16_t	get_fifo8_length(fifo8_cb_td* fifo_cb)
{
    return fifo_cb->length;
}

uint16_t	get_fifo8_vacant(fifo8_cb_td* fifo_cb)
{
    return (fifo_cb->depth - fifo_cb->length);
}

uint8_t	fifo8_push(fifo8_cb_td* fifo_cb, uint8_t* src_addr)
{
    if(is_fifo8_full(fifo_cb)==1)
        return 0;
    else
    {
        memcpy(fifo_cb->bhead_ptr, src_addr, fifo_cb->block_size);
        if(fifo_cb->bhead_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
            fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
        else
            fifo_cb->bhead_ptr += fifo_cb->block_size;
        fifo_cb->length ++;
        return 1;
    }
}


uint8_t	fifo8_pop(fifo8_cb_td* fifo_cb, uint8_t* dest_addr)
{
    if(is_fifo8_empty(fifo_cb)==1)
        return 0;
    else
    {
        memcpy(dest_addr, fifo_cb->btail_ptr, fifo_cb->block_size);
        if(fifo_cb->btail_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
            fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
        else
            fifo_cb->btail_ptr += fifo_cb->block_size;
        fifo_cb->length --;
        return 1;
    }
}

void	fifo8_reset(fifo8_cb_td* fifo_cb)
{
    fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
    fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
    fifo_cb->length = 0;
}
