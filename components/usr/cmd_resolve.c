#include "sys_def.h"
#include "global_var.h"
#include "bit_op.h"
#include "fifo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "esp_timer.h"
//#include "esp_log.h"
#include "kfifo.h"
#include "fifo.h"
//#include "tlvparse.h"

//cpad err code
enum
{
    CMD_ERR_NOERR = 0,
    CMD_ERR_ADDR_OR,
    CMD_ERR_DATA_OR,
    CMD_ERR_WR_OR,
    CMD_ERR_CONFLICT_OR,
    CMD_RDFIFO_EMPTY,
    CMD_WRFIFO_FULL,
    CMD_NOT_READY,
    CMD_ERR_UNKNOWN
};

enum
{
    FRAME_SYNC_POS = 0,
    FRAME_CTRL_POS = 2,
    FRAME_C_PLA_POS = 4,
    FRAME_C_PLD_POS = 5,
};
#define FRAME_D_PLD_POS FRAME_C_PLA_POS 

#define CMD_FRAME_OVSIZE        4

#define CMD_RTX_BUF_DEPTH       1024
#define CMD_FSM_TIMEOUT         2

#define CMD_FRAME_TAG_M_SYNC 	0x1bdf
#define CMD_FRAME_TAG_S_SYNC 	0x9bdf

#define CMD_FRAME_FSM_SYNC      0x01
#define CMD_FRAME_FSM_TL        0x02
#define CMD_FRAME_FSM_DATA      0x04

#define CMD_RD_REG	            0x01
#define CMD_WR_REG	            0x02
#define CMD_RP_PKG	            0x80


static uint8_t 	cmd_kbuf_tx[CMD_RTX_BUF_DEPTH*16];
static uint8_t 	cmd_kbuf_rx[CMD_RTX_BUF_DEPTH];
kfifo_t 		kc_buf_tx;
kfifo_t 		kc_buf_rx;

//fifo32_cb_td cmd_rx_fifo;
esp_timer_handle_t geo_timer;

typedef struct
{
    uint8_t tx_buf[CMD_RTX_BUF_DEPTH*2];
    uint8_t rx_buf[CMD_RTX_BUF_DEPTH];
    uint16_t tx_cnt;
    uint16_t tx_cmd;
    uint8_t  tx_errcode;
    uint16_t rx_cnt;
    uint16_t rx_tag;
    uint16_t rtx_timeout;
    uint8_t cmd_fsm_cstate;
}cmd_reg_st;

static cmd_reg_st cmd_reg_inst;


/**
 * @brief  cmd rtx buffer initialization
 * @param  none
 * @retval none
 */
static void cmd_buf_init(void)
{
    uint16_t i;
    //tx buffer initialization
    for(i=0;i<CMD_RTX_BUF_DEPTH;i++)
    {
        cmd_reg_inst.tx_buf[i] = 0;
    }
    for(i=0;i<CMD_RTX_BUF_DEPTH;i++)
    {
        cmd_reg_inst.rx_buf[i] = 0;
    }
    *(uint16_t *)&cmd_reg_inst.tx_buf[0] = CMD_FRAME_TAG_S_SYNC;
    cmd_reg_inst.tx_cnt = 0;
    cmd_reg_inst.tx_cmd = 0;

    cmd_reg_inst.rx_cnt = 0;
    cmd_reg_inst.rx_tag = 0;
    cmd_reg_inst.rtx_timeout = 0;
    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;

    //rx fifo initialization
    //fifo32_init(&cmd_rx_fifo,1,CMD_RTX_BUF_DEPTH);

    memset(&kc_buf_tx, 0, sizeof(kc_buf_tx));
    kfifo_init(&kc_buf_tx, (void *)cmd_kbuf_tx, sizeof(cmd_kbuf_tx));

    memset(&kc_buf_rx, 0, sizeof(kc_buf_rx));
    kfifo_init(&kc_buf_rx, (void *)cmd_kbuf_rx, sizeof(cmd_kbuf_rx));
}


/**
 * @brief  cmd uart send interface
 * @param  tx_buf_ptr: tx src buffer pointer
						tx_cnt: tx src buffer transmitt count
 * @retval none
 */
void cmd_dev_init(void)
{
    cmd_buf_init();
    //geo_timer_init();
}

/**
 * @brief  cmd recieve frame checksum
 * @param  none
 * @retval
			`0: checksum ok
			`1:	checksum fail
 */
static int frame_checksum(void) {
    uint8_t res, i;
    res = 0;
    for (i = 0; i < cmd_reg_inst.rx_cnt; i++)
    {
        res ^= cmd_reg_inst.rx_buf[i];
    }
    if (res == 0)
        return 1;
    else
        return 0;
}

/**
 * @brief  cmd transmmite frame checksum
 * @param  data_ptr: data buffer pointer whose checksum is to be caculated
						data_num: number of data to be caculated
 * @retval caculated checksum
 */
static int frame_checksum_gen(void * data_ptr, uint16_t data_num) {
    uint8_t res, i;
    res = 0;
    for (i = 0; i < data_num; i++)
    {
        res ^= *((uint8_t *)data_ptr + i);
    }
    return res;
}

/**
 * @brief  cmd recieve frame finite state machine
 * @param  none
 * @retval
			`0: cmd frame recieve ok
			`1:	cmd frame recieve ng
 */
uint16_t cmd_get_comm_sts(void)
{
    return cmd_reg_inst.rtx_timeout;
}

uint8_t cmd_get_rx_fsm(void)
{
    return cmd_reg_inst.cmd_fsm_cstate;
}

/**
 * @brief  cmd recieve frame finite state machine
 * @param  none
 * @retval
			`0: cmd frame recieve ok
			`1:	cmd frame recieve ng
 */
uint16_t cmd_frame_recv(void)
{
    if (cmd_reg_inst.rx_tag == 1)//if there is already an unprocessed frame in the rx buffer, quit new frame recieving
    {
        return 1;
    } else {
        return 0;
    }
}


/**
 * @brief  cmd send response frame
 * @param  none
 * @retval none
 */
static void cmd_response(void)
{
    uint32_t check_sum;
    uint32_t ret;

    *((uint16_t *)&cmd_reg_inst.tx_buf[FRAME_SYNC_POS]) = CMD_FRAME_TAG_S_SYNC;

    cmd_reg_inst.tx_buf[FRAME_CTRL_POS] = (cmd_reg_inst.tx_cmd<<12)|cmd_reg_inst.tx_cnt;

    check_sum = frame_checksum_gen(&cmd_reg_inst.tx_buf[0],(cmd_reg_inst.tx_cnt+CMD_FRAME_OVSIZE));	
    //response frame checksum caculate
    cmd_reg_inst.tx_buf[cmd_reg_inst.tx_cnt+CMD_FRAME_OVSIZE] = check_sum;
    ret = kfifo_in(&kc_buf_tx,cmd_reg_inst.tx_buf,(cmd_reg_inst.tx_cnt+CMD_FRAME_OVSIZE)*4);
    if(ret==0)
    {
        kfifo_reset(&kc_buf_tx);
        printf("etf\n");
    }
}

/**
 * @brief  cmd command write reg operation
 * @param  none
 * @retval
	`CMD_ERR_NOERR         : operation OK
	`CMD_ERR_ADDR_OR	   : requested address out of range
    `CMD_ERR_DATA_OR	   : requested data out of range
    `CMD_ERR_PERM_OR	   : request permission denied
    `CMD_ERR_WR_OR		   : write operation prohibited
    `CMD_ERR_UNKNOWN	   : unknown error
 */
static uint16_t cmd_wr_reg(void)
{
    uint8_t err_code;
    //extern sys_reg_st g_sys;
    uint32_t reg_data;
    uint16_t reg_addr;
    err_code = CMD_ERR_NOERR;

    cmd_reg_inst.rx_cnt = 0;								//clear rx_buffer
    cmd_reg_inst.rx_tag = 0;

    reg_addr = cmd_reg_inst.rx_buf[FRAME_C_PLA_POS];
    reg_data = *((uint32_t *)&cmd_reg_inst.rx_buf[FRAME_D_PLD_POS]);

    err_code = reg_map_write(reg_addr, &reg_data, 1); //write conf reg map

    *((uint32_t *)&cmd_reg_inst.tx_buf[FRAME_C_PLA_POS]) = err_code;

    cmd_reg_inst.tx_cnt = 4;
    cmd_reg_inst.tx_cmd = cmd_reg_inst.rx_buf[FRAME_CTRL_POS];
    cmd_reg_inst.tx_errcode = err_code;

    cmd_response();
    return err_code;
}

/**
 * @brief  cmd command write reg operation
 * @param  none
 * @retval
	`CMD_ERR_NOERR		   : operation OK
	`CMD_ERR_ADDR_OR	   : requested address out of range
    `CMD_ERR_DATA_OR	   : requested data out of range
    `CMD_ERR_PERM_OR	   : request permission denied
    `CMD_ERR_WR_OR		   : write operation prohibited
    `CMD_ERR_UNKNOWN	   : unknown error
 */
static uint16_t cmd_rd_reg(void)
{
    uint8_t err_code;
    uint16_t reg_addr;
    uint32_t reg_data;

    err_code = CMD_ERR_NOERR;

    cmd_reg_inst.rx_cnt = 0;								//clear rx_buffer
    cmd_reg_inst.rx_tag = 0;

    reg_addr = cmd_reg_inst.rx_buf[FRAME_C_PLA_POS];

    err_code = reg_map_read(reg_addr, &reg_data, 1);

    *((uint32_t *)&cmd_reg_inst.tx_buf[FRAME_C_PLA_POS]) = reg_data;

    cmd_reg_inst.tx_cmd = (cmd_reg_inst.rx_buf[FRAME_CTRL_POS] >> 12) & 0x000f;
    cmd_reg_inst.tx_cnt = 4;
    cmd_reg_inst.tx_errcode = err_code;
    cmd_response();
    return err_code;
}
/**
 * @brief  cmd command write reg operation
 * @param  none
 * @retval
	`CMD_ERR_NOERR		   : operation OK
	`CMD_ERR_ADDR_OR	   : requested address out of range
    `CMD_ERR_DATA_OR	   : requested data out of range
    `CMD_ERR_PERM_OR	   : request permission denied
    `CMD_ERR_WR_OR		   : write operation prohibited
    `CMD_ERR_UNKNOWN	   : unknown error
 */
int daq_frame(void *dbuf_ptr,int d_len,int size)
{
    uint8_t err_code, check_sum;

    err_code = CMD_ERR_NOERR;

    cmd_reg_inst.rx_cnt = 0;								//clear rx_buffer
    cmd_reg_inst.rx_tag = 0;
    
    cmd_reg_inst.tx_cnt = d_len*size;
    memcpy(&cmd_reg_inst.tx_buf[FRAME_D_PLD_POS],dbuf_ptr,cmd_reg_inst.tx_cnt);

    *((uint16_t *)&cmd_reg_inst.tx_buf[FRAME_SYNC_POS]) = CMD_FRAME_TAG_S_SYNC;
    cmd_reg_inst.tx_buf[FRAME_CTRL_POS] = (CMD_RP_PKG<<12)|(cmd_reg_inst.tx_cnt);

    check_sum = frame_checksum_gen(&cmd_reg_inst.tx_buf[0],(cmd_reg_inst.tx_cnt+CMD_FRAME_OVSIZE));	
    //response frame checksum caculate
    cmd_reg_inst.tx_buf[cmd_reg_inst.tx_cnt+CMD_FRAME_OVSIZE] = check_sum;
    
    return err_code;
}

/**
 * @brief  cmd command write reg operation
 * @param  none
 * @retval
	`CMD_ERR_NOERR	       : operation OK
	`CMD_ERR_ADDR_OR	   : requested address out of range
    `CMD_ERR_DATA_OR	   : requested data out of range
    `CMD_ERR_PERM_OR	   : request permission denied
    `CMD_ERR_WR_OR		   : write operation prohibited
    `CMD_ERR_UNKNOWN	   : unknown error
 */
uint16_t cmd_frame_resolve(void)
{
    uint8_t err_code;
    uint8_t frame_cmd_type;

    err_code = CMD_ERR_NOERR;
    frame_cmd_type = (cmd_reg_inst.rx_buf[FRAME_CTRL_POS] >> 12) & 0x000f;

    if (cmd_reg_inst.rx_tag == 0) {
        err_code = CMD_ERR_NOERR;
        return err_code;
    }

    switch (frame_cmd_type) {
    case (CMD_RD_REG): {
        err_code = cmd_rd_reg();
        printf("console: read reg.\n");
        break;
    }
    case (CMD_WR_REG): {
        err_code = cmd_wr_reg();
        printf("console: write reg.\n");
        break;
    }
    default: {
        err_code = CMD_ERR_UNKNOWN;
        break;
    }
    }
    return err_code;
}

void recv_frame_fsm(void)
{
    uint32_t rx_data;

    if (cmd_reg_inst.rx_tag == 1)
        return;

    if (cmd_reg_inst.cmd_fsm_cstate == CMD_FRAME_FSM_DATA) {
        cmd_reg_inst.rtx_timeout++;
    }

    while ((kfifo_out(&kc_buf_rx, &rx_data, 2) == 2)
    //while ((fifo32_pop(&cmd_rx_fifo, &rx_data) == 1)
            && (cmd_reg_inst.rx_tag == 0))
    {
        printf("rcv:%x\n",rx_data);
        //printf("fsm_len\n ");
        switch (cmd_reg_inst.cmd_fsm_cstate)
        {
            case (CMD_FRAME_FSM_SYNC): {
                cmd_reg_inst.rx_cnt = 0;
                if (rx_data == CMD_FRAME_TAG_M_SYNC) {
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = rx_data;
                    cmd_reg_inst.rx_cnt++;
                    cmd_reg_inst.rx_tag = 0;
                    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_TL;
            printf("sync\n");
                } else {
                    cmd_reg_inst.rx_cnt = 0;
                    cmd_reg_inst.rx_tag = 0;
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
                }
                cmd_reg_inst.rtx_timeout = 0;
                break;
            }
            case (CMD_FRAME_FSM_TL): {
                if (cmd_reg_inst.rtx_timeout < CMD_FSM_TIMEOUT) {
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = rx_data;
                    cmd_reg_inst.rx_cnt++;
                    cmd_reg_inst.rx_tag = 0;
                    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_DATA;
            printf("tl\n");
                } else {
                    cmd_reg_inst.rx_cnt = 0;
                    cmd_reg_inst.rx_tag = 0;
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
                }
                cmd_reg_inst.rtx_timeout = 0;
                printf("fsm_tlen\n ");
                break;
            }
            case (CMD_FRAME_FSM_DATA): {
                if (cmd_reg_inst.rtx_timeout > CMD_FSM_TIMEOUT) {
                    cmd_reg_inst.rx_cnt = 0;
                    cmd_reg_inst.rx_tag = 0;
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                    cmd_reg_inst.rtx_timeout = 0;
                    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
                    printf("cmd timeout\n");
                } else if (((cmd_reg_inst.rx_buf[FRAME_CTRL_POS] & 0x0fff)
                        + CMD_FRAME_OVSIZE ) > cmd_reg_inst.rx_cnt) {
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = rx_data;
                    cmd_reg_inst.rx_cnt++;
                    cmd_reg_inst.rx_tag = 0;
                    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_DATA;
                } else {
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = rx_data;
                    cmd_reg_inst.rx_cnt++;
                    if (frame_checksum() == 1) {
                        cmd_reg_inst.rx_tag = 1;
                        cmd_reg_inst.rx_cnt = cmd_reg_inst.rx_cnt;
                        cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                        printf("chk ok\n");
                    } else {
                        cmd_reg_inst.rx_tag = 0;
                        cmd_reg_inst.rx_cnt = 0;
                        cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                        printf("chk error\n");
                    }
                    cmd_reg_inst.rtx_timeout = 0;
                    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
                }
                printf("fsm_data\n ");
                break;
            }
            default: {
                cmd_reg_inst.rx_cnt = 0;
                cmd_reg_inst.rx_tag = 0;
                cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
                cmd_reg_inst.rtx_timeout = 0;
                //printf("fsm_default\n ");
                break;
            }
        }
    }
}



//void tlv_test(void)
//{
//	unsigned char buf2[] = {
//        0x6F,0x22,0x84,0x0E,0x31,0x50,0x41,0x59,0x2E,0x53,0x59,0x53,
//        0x2E,0x44,0x44,0x46,0x30,0x31,0xA5,0x10,0x88,0x01,0x01,0x5F,
//        0x2D,0x02,0x7A,0x68,0xBF,0x0C,0x05,0x9F,0x4D,0x02,0x0B,0x0A
//	};
//
//	unsigned char buf3[] = {
//		0x70,0x1C,0x61,0x1A,0x4F,0x08,0xA0,0x00,0x00,0x03,0x33,0x01,
//		0x01,0x02,0x50,0x0B,0x50,0x42,0x4F,0x43,0x20,0x43,0x52,0x45,
//		0x44,0x49,0x54,0x87,0x01,0x01
//	};
//
//
//	//parse 1
//	struct TLVNode* node = TLV_Parse(buf2,sizeof(buf2));
//	TLV_Debug(node);
//
//	//parse 2
//	struct TLVNode* node2 = TLV_Parse(buf3,sizeof(buf3));
//	TLV_Debug(node2);
//
//	TLV_Merge(node,node2);
//	TLV_Free(node2);
//
//	struct TLVNode* found = TLV_Find(node,0x9f4d);
//	if(found)
//	{
//		printf("FOUND! 9f4d\n");
//	}
//	else
//	{
//		printf("NOT FOUND! 9f4d\n");
//	}
//
//	found = TLV_Find(node,0x4f);
//	if(found)
//	{
//		printf("FOUND! 4f\n");
//	}
//	else
//	{
//		printf("NOT FOUND! 4f\n");
//	}
//}

void cmd_thread(void* param)
{
    //vTaskDelay(5000 / portTICK_PERIOD_MS);
	//tlv_test();
    vTaskDelay(CMD_THREAD_DELAY);
	cmd_dev_init();
	while(1)
	{
		recv_frame_fsm();
		cmd_frame_resolve();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}


//FINSH_FUNCTION_EXPORT(show_cmd_info, show cmd information.);
