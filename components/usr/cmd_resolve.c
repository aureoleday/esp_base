#include "sys_def.h"
#include "global_var.h"
#include "bit_op.h"
#include "fifo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "esp_timer.h"
//#include "esp_log.h"
#include "kfifo.h"

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
    FRAME_C_ATL_POS,
    FRAME_D_AL_POS,
    FRAME_D_PL_POS
};


#define CMD_FRAME_OVSIZE	3

#define CMD_RTX_BUF_DEPTH	1024
#define CMD_FSM_TIMEOUT	 	2

#define CMD_FRAME_TAG_M_SYNC 	0x1bdf9bdf
#define CMD_FRAME_TAG_S_SYNC 	0x9bdf1bdf

#define CMD_FRAME_FSM_SYNC	0x01
#define CMD_FRAME_FSM_TL	0x02
#define CMD_FRAME_FSM_DATA	0x04


#define CMD_RD_REG		0x0001
#define CMD_WR_REG		0x0002
#define CMD_RP_PKG		0x0080
#define CMD_RP_GEO		0x0100


static uint8_t 	cmd_kbuf_tx[CMD_RTX_BUF_DEPTH*16];
kfifo_t 		kc_buf_tx;

fifo32_cb_td cmd_rx_fifo;
esp_timer_handle_t geo_timer;

typedef struct
{
    uint32_t tx_buf[CMD_RTX_BUF_DEPTH*2];
    uint32_t rx_buf[CMD_RTX_BUF_DEPTH];
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
    cmd_reg_inst.tx_buf[0] = CMD_FRAME_TAG_S_SYNC;
    cmd_reg_inst.tx_cnt = 0;
    cmd_reg_inst.tx_cmd = 0;

    cmd_reg_inst.rx_cnt = 0;
    cmd_reg_inst.rx_tag = 0;
    cmd_reg_inst.rtx_timeout = 0;
    cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;

    //rx fifo initialization
    fifo32_init(&cmd_rx_fifo,1,CMD_RTX_BUF_DEPTH);

    memset(&kc_buf_tx, 0, sizeof(kc_buf_tx));
    kfifo_init(&kc_buf_tx, (void *)cmd_kbuf_tx, sizeof(cmd_kbuf_tx));
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
static uint32_t frame_checksum(void) {
    uint32_t res, i;
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
static uint32_t frame_checksum_gen(uint32_t* data_ptr, uint16_t data_num) {
    uint32_t res, i;
    res = 0;
    for (i = 0; i < data_num; i++)
    {
        res ^= *(data_ptr + i);
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

    cmd_reg_inst.tx_buf[FRAME_SYNC_POS] = CMD_FRAME_TAG_S_SYNC;

    cmd_reg_inst.tx_buf[FRAME_C_ATL_POS] = (cmd_reg_inst.tx_errcode<<24)|(cmd_reg_inst.tx_cmd<<16)|cmd_reg_inst.tx_cnt;

    check_sum = frame_checksum_gen(&cmd_reg_inst.tx_buf[0],(cmd_reg_inst.tx_cnt+CMD_FRAME_OVSIZE-1));		//response frame checksum caculate
    cmd_reg_inst.tx_buf[cmd_reg_inst.tx_cnt+CMD_FRAME_OVSIZE-1] = check_sum;
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
		`CMD_ERR_NOERR			 : operation OK
		`CMD_ERR_ADDR_OR	   : requested address out of range
    `CMD_ERR_DATA_OR	   : requested data out of range
    `CMD_ERR_PERM_OR	   : request permission denied
    `CMD_ERR_WR_OR		   : write operation prohibited
    `CMD_ERR_UNKNOWN	   : unknown error
 */
static uint16_t cmd_wr_reg(void)
{
    uint8_t err_code;
    uint8_t tx_addr, tx_cnt;
    //extern sys_reg_st g_sys;

    err_code = CMD_ERR_NOERR;
    tx_addr = cmd_reg_inst.rx_buf[FRAME_D_AL_POS] >> 16;
    tx_cnt = cmd_reg_inst.rx_buf[FRAME_D_AL_POS] & 0x0000ffff;

    cmd_reg_inst.rx_cnt = 0;								//clear rx_buffer
    cmd_reg_inst.rx_tag = 0;

    err_code = reg_map_write(tx_addr, &cmd_reg_inst.rx_buf[FRAME_D_PL_POS],	tx_cnt); 									//write conf reg map

    cmd_reg_inst.tx_buf[FRAME_D_AL_POS] = cmd_reg_inst.rx_buf[FRAME_D_AL_POS];

    cmd_reg_inst.tx_cnt = 1;
    cmd_reg_inst.tx_cmd = (cmd_reg_inst.rx_buf[FRAME_C_ATL_POS] >> 16) & 0x00ff;
    cmd_reg_inst.tx_errcode = err_code;

    cmd_response();
    return err_code;
}

/**
 * @brief  cmd command write reg operation
 * @param  none
 * @retval
		`CMD_ERR_NOERR			 : operation OK
		`CMD_ERR_ADDR_OR	   : requested address out of range
    `CMD_ERR_DATA_OR	   : requested data out of range
    `CMD_ERR_PERM_OR	   : request permission denied
    `CMD_ERR_WR_OR		   : write operation prohibited
    `CMD_ERR_UNKNOWN	   : unknown error
 */
static uint16_t cmd_rd_reg(void)
{
    uint8_t err_code;
    uint16_t rd_addr, rd_cnt;

    err_code = CMD_ERR_NOERR;

    rd_addr = cmd_reg_inst.rx_buf[FRAME_D_AL_POS] >> 16;
    rd_cnt = cmd_reg_inst.rx_buf[FRAME_D_AL_POS] & 0x0000ffff;

    cmd_reg_inst.rx_cnt = 0;								//clear rx_buffer
    cmd_reg_inst.rx_tag = 0;

    if (rd_cnt > CMD_RTX_BUF_DEPTH) {
        err_code = CMD_ERR_UNKNOWN;
        cmd_reg_inst.tx_cnt = 1;
    } else {
        err_code = reg_map_read(rd_addr, &cmd_reg_inst.tx_buf[FRAME_D_PL_POS],
                rd_cnt);
        cmd_reg_inst.tx_cnt = rd_cnt + 1;
    }
    cmd_reg_inst.tx_buf[FRAME_D_AL_POS] = cmd_reg_inst.rx_buf[FRAME_D_AL_POS];

    cmd_reg_inst.tx_cmd = (cmd_reg_inst.rx_buf[FRAME_C_ATL_POS] >> 16) & 0x00ff;
    cmd_reg_inst.tx_errcode = err_code;
    cmd_response();
    return err_code;
}


/**
 * @brief  cmd command write reg operation
 * @param  none
 * @retval
		`CMD_ERR_NOERR			 : operation OK
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
    frame_cmd_type = (cmd_reg_inst.rx_buf[FRAME_C_ATL_POS] >> 16) & 0x00ff;

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

    while ((fifo32_pop(&cmd_rx_fifo, &rx_data) == 1)
            && (cmd_reg_inst.rx_tag == 0))
    {
        //printf("tcp: %x\n ",rx_data);
        switch (cmd_reg_inst.cmd_fsm_cstate)
        {
        case (CMD_FRAME_FSM_SYNC): {
            cmd_reg_inst.rx_cnt = 0;
            if (rx_data == CMD_FRAME_TAG_M_SYNC) {
                cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = rx_data;
                cmd_reg_inst.rx_cnt++;
                cmd_reg_inst.rx_tag = 0;
                cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_TL;
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
            } else {
                cmd_reg_inst.rx_cnt = 0;
                cmd_reg_inst.rx_tag = 0;
                cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
            }
            cmd_reg_inst.rtx_timeout = 0;
            //printf("fsm_len\n ");
            break;
        }
        case (CMD_FRAME_FSM_DATA): {
            if (cmd_reg_inst.rtx_timeout > CMD_FSM_TIMEOUT) {
                cmd_reg_inst.rx_cnt = 0;
                cmd_reg_inst.rx_tag = 0;
                cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                cmd_reg_inst.rtx_timeout = 0;
                cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
                //printf("cmd timeout\n");
            } else if (((cmd_reg_inst.rx_buf[FRAME_C_ATL_POS] & 0x0000ffff)
                    + CMD_FRAME_OVSIZE - 1) > cmd_reg_inst.rx_cnt) {
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
                    //printf("chk ok\n");
                } else {
                    cmd_reg_inst.rx_tag = 0;
                    cmd_reg_inst.rx_cnt = 0;
                    cmd_reg_inst.rx_buf[cmd_reg_inst.rx_cnt] = 0;
                    //printf("chk error\n");
                }
                cmd_reg_inst.rtx_timeout = 0;
                cmd_reg_inst.cmd_fsm_cstate = CMD_FRAME_FSM_SYNC;
            }
            //printf("fsm_data\n ");
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

uint16_t report_data(void)
{
    extern sys_reg_st	  g_sys;
    uint8_t err_code;
    uint16_t rd_cnt;

    err_code = CMD_ERR_NOERR;
    if(bit_op_get(g_sys.stat.gen.status_bm,GBM_TCP) == 0)
        return CMD_NOT_READY;

    rd_cnt = 0;

    if(rd_cnt == 0)
        return 0;

    cmd_reg_inst.tx_cmd	= CMD_RP_PKG;
    cmd_reg_inst.tx_cnt = rd_cnt;
    cmd_reg_inst.tx_errcode = err_code;
    cmd_response();
    return err_code;
}


static int16_t get_geo_fdata(float* buf_ptr)
{
    extern kfifo_t kf_s;

    return kfifo_out(&kf_s,buf_ptr,sizeof(uint32_t)*256);
}

uint16_t report_geo_data(void)
{
    extern sys_reg_st	  g_sys;
    uint8_t err_code;
    uint16_t rd_cnt;

    err_code = CMD_ERR_NOERR;
    if((bit_op_get(g_sys.stat.gen.status_bm,GBM_BT) == 0)&&(bit_op_get(g_sys.stat.gen.status_bm,GBM_TCP) == 0))
        return CMD_NOT_READY;

    //    rd_cnt = get_geo_data(&cmd_reg_inst.tx_buf[FRAME_D_AL_POS])/4;
    rd_cnt = get_geo_fdata((float*)&cmd_reg_inst.tx_buf[FRAME_D_AL_POS])/4;

    if(rd_cnt == 0)
        return 0;

    cmd_reg_inst.tx_cmd	= CMD_RP_GEO;
    cmd_reg_inst.tx_cnt = rd_cnt;
    cmd_reg_inst.tx_errcode = err_code;
    cmd_response();
    return err_code;
}

void cmd_thread(void* param)
{
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
