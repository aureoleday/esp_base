/*
 * spi_drv.c
 *
 *  Created on: 2018Äê12ÔÂ27ÈÕ
 *      Author: Administrator
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"
#include "adxl_drv.h"
#include "my_fft.h"
#include "sys_conf.h"
#include "bit_op.h"
#include "kfifo.h"
#include "goertzel.h"
#include "reg_map_check.h"

#define min(a,b)  ((a)>(b) ? (b) : (a))            /*  */

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_FF   25 

#define GPIO_INPUT_PIN_SEL  (1ULL<<PIN_NUM_FF)
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "ADXL";

static uint8_t 	rxd_temp[DEV_GEO_RTX_SIZE];
static uint8_t 	kf_buf_s[DEV_GEO_FIFO_SIZE];
kfifo_t 		kf_s;

static SemaphoreHandle_t geospi_mutex = NULL;
static spi_transaction_t adxl_t;

esp_timer_handle_t adxl_timer;

typedef struct
{
    spi_device_handle_t     		spi_device_h;
    uint8_t                         txd[DEV_GEO_RTX_SIZE];
    uint8_t                         rxd[DEV_GEO_RTX_SIZE];
    uint16_t                        adxl_init_done;
}spi_geo_device_st;

spi_geo_device_st spi_geo_dev_inst;

static void adxl_register(void);
static int16_t raw_data_buf(uint32_t din, uint8_t axis);

static void kf_init(void)
{
    memset(&kf_s, 0, sizeof(kf_s));
    kfifo_init(&kf_s, (void *)kf_buf_s, sizeof(kf_buf_s));
}

static int32_t decode(uint32_t din)
{
    uint32_t temp;
    temp = din>>4;
    if((temp&0x80000) != 0)
        temp |= 0xfff00000;
    return (int32_t)temp;
}

void geo_ds_init(void)
{
    kf_init();
    geospi_mutex = xSemaphoreCreateMutex();
    spi_geo_dev_inst.adxl_init_done = 0;
}

uint8_t adxl_wr_reg(uint8_t addr, uint8_t data)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));

    xSemaphoreTake( geospi_mutex, portMAX_DELAY );

    t.length=8*2;
    t.user=(void*)0;

    spi_geo_dev_inst.txd[0] = (addr<<1);

    spi_geo_dev_inst.txd[1] = data;

    t.tx_buffer=spi_geo_dev_inst.txd;

    //esp_err_t ret = spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
    esp_err_t ret = spi_device_queue_trans(spi_geo_dev_inst.spi_device_h, &t, 0);

    xSemaphoreGive( geospi_mutex );
    return ret;
}

uint8_t adxl_rd_reg(uint8_t addr, uint8_t * rx_buf, uint8_t cnt)
{
    uint8_t i;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));

    xSemaphoreTake( geospi_mutex, portMAX_DELAY );

    t.length=8*(cnt+1);
    t.rx_buffer = rx_buf;

    spi_geo_dev_inst.txd[0] = (addr<<1)|0x01;

    for(i=0;i<cnt;i++)
    {
        spi_geo_dev_inst.txd[1+i] = 0;
    }

    t.tx_buffer=spi_geo_dev_inst.txd;

    //spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
    spi_device_queue_trans(spi_geo_dev_inst.spi_device_h, &t, 1);

    xSemaphoreGive( geospi_mutex );

    return *(rx_buf+1);
}

static void IRAM_ATTR adxl_read_pcb(spi_transaction_t* t)
{
	extern sys_reg_st g_sys;
    uint16_t i,rd_cnt;
    uint32_t buf_temp;

    rd_cnt = g_sys.conf.geo.fifo_th*3;

    for(i=0;i<rd_cnt;i++)
    {
        buf_temp = (spi_geo_dev_inst.rxd[1+i*3]<<16)|(spi_geo_dev_inst.rxd[2+i*3]<<8)|(spi_geo_dev_inst.rxd[3+i*3]);
        raw_data_buf(buf_temp,g_sys.conf.geo.axis);
    }
}

static uint16_t adxl_sread(uint16_t rx_len)
{
    uint16_t ret = 0;
    uint16_t i=0;
    uint16_t rx_bytes =0;
    //spi_transaction_t t;
    //memset(&t, 0, sizeof(t));
    memset(&adxl_t, 0, sizeof(adxl_t));
    //xSemaphoreTake( adcspi_mutex, portMAX_DELAY );

    rx_bytes = rx_len*9;
    adxl_t.length= 8*(rx_bytes + 1);
    adxl_t.rx_buffer = spi_geo_dev_inst.rxd;

    spi_geo_dev_inst.txd[0] = (ADXL_FIFO_DATA<<1)|0x01;
    
    for(i=0;i<rx_bytes;i++)
    {
        spi_geo_dev_inst.txd[i+1] = 0;
    }

    adxl_t.tx_buffer=spi_geo_dev_inst.txd;
    spi_device_queue_trans(spi_geo_dev_inst.spi_device_h, &adxl_t, 0);

    ret = rx_bytes;
    return ret;
}

static void IRAM_ATTR int_isr_handler(void* arg)
{
    extern sys_reg_st  g_sys;
    if(bit_op_get(g_sys.stat.gen.status_bm,GBM_GEO) == 1)
        adxl_sread(g_sys.conf.geo.fifo_th); 
}

static void adxl_pin_init(void)
{
    gpio_config_t io_conf;
    //interrupt of falling edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_NUM_FF, int_isr_handler, (void*) PIN_NUM_FF);
}

void adxl_init(void)
{
    esp_err_t ret;
    //    spi_device_handle_t spi;
    spi_bus_config_t buscfg=
    {
            .miso_io_num=PIN_NUM_MISO,
            .mosi_io_num=PIN_NUM_MOSI,
            .sclk_io_num=PIN_NUM_CLK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz=64
    };
    spi_device_interface_config_t devcfg=
    {
            .clock_speed_hz=16*1000*1000,           //Clock out at 16 MHz
            .mode=0,                                //SPI mode 0
            .spics_io_num=PIN_NUM_CS,               //CS pin
            .post_cb = adxl_read_pcb,
            .cs_ena_pretrans = 1,
            .cs_ena_posttrans = 1,
            .queue_size = 8,                          //We want to be able to queue 12 transactions at a time
            //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    geo_ds_init();
    //adxl_timer_init();
    ESP_ERROR_CHECK(ret);
    //Attach the device to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi_geo_dev_inst.spi_device_h);
    ESP_ERROR_CHECK(ret);
    adxl_pin_init();
    adxl_register();
	//geo_pkg_en(1);
    //ESP_LOGI(TAG,"adxl driver initiated!");
}

void adxl355_reset(void)
{
    extern sys_reg_st  g_sys;
    adxl_wr_reg(ADXL_RESET,0x52);
    adxl_wr_reg(ADXL_FILTER,g_sys.conf.geo.filter);
}

static int16_t raw_data_buf(uint32_t din, uint8_t axis)
{
    extern sys_reg_st  g_sys;
    static uint8_t stage = 0;  //0: idle;1:x;2:y;3:z;
    static uint32_t dbuf[3]={0,0,0};
    //float temp;
    int32_t temp;
    uint32_t dummy;
    int16_t ret = 0;

    switch (stage)
    {
    case 0:
    {
        if(din & 0x1)
        {
            dbuf[0] = din;
            stage = 2;
        }
        else
        {
            stage = 0;
        }
        ret = 0;
        break;
    }
    case 1:
    {
        if(din & 0x1)
        {
            dbuf[0] = din;
            stage = 2;
            ret = 0;
        }
        else
        {
            stage = 0;
            ret = -1;
        }
        break;
    }
    case 2:
    {
        if(!(din & 0x1))
        {
            dbuf[1] = din;
            stage = 3;
            ret = 0;
        }
        else
        {
            dbuf[0] = din;
            stage = 2;
            ret = -2;
        }
        break;
    }
    case 3:
    {
        if(!(din & 0x1))
        {
            dbuf[2] = din;
            //temp = (float)decode(dbuf[axis])*0.0000039;
            temp = decode(dbuf[axis]);
            //goertzel_lfilt(temp);
            if(g_sys.conf.geo.pkg_en)
            {
                if(kfifo_len(&kf_s) >= kf_s.size)
                {
                    kfifo_out(&kf_s,&dummy,sizeof(uint32_t));
                    g_sys.stat.geo.kfifo_drop_cnt++;
                    ret = -4;
                }
                else
                    ret = 0;
                kfifo_in(&kf_s,&temp,sizeof(uint32_t));
            }
            stage = 1;
        }
        else
        {
            dbuf[0] = din;
            stage = 2;
            ret = -3;
        }
        break;
    }
    default:
    {
        stage = 0;
        ret = -1;
        break;
    }
    }
    return ret;
}

int adxl_dout(uint8_t * dst_ptr, uint16_t max_len)
{
    int rd_len = 0;
    int fifo_len = kfifo_len(&kf_s);
    rd_len = (fifo_len<max_len)? fifo_len:max_len;
    kfifo_out(&kf_s, dst_ptr ,rd_len);
    //printf("f_len:%d,max_len:%d,r_len:%d\n",fifo_len,max_len,rd_len);
    return rd_len; 
}

static int adxl_info(int argc, char **argv)
{
    printf("Status\tFentry\tFilt\tTemp\tPwr#\n");
    printf("%x\t%x\t%x\t%x\t%x\n",
            adxl_rd_reg(ADXL_STATUS,rxd_temp,1),
            adxl_rd_reg(ADXL_FIFO_ENTRIES,rxd_temp,1),
            adxl_rd_reg(ADXL_FILTER,rxd_temp,1),
            adxl_rd_reg(ADXL_TEMP2,rxd_temp,1),
            adxl_rd_reg(ADXL_POWER_CTL,rxd_temp,1));
    return 0;
}

/** Arguments used by 'join' function */
static struct {
    struct arg_int *addr;
    struct arg_int *data;
    struct arg_end *end;
} adxl_args;

static int ard_reg(int argc, char **argv)
{
    uint8_t rx_buf[16];
    uint8_t i;

    int nerrors = arg_parse(argc, argv, (void**) &adxl_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, adxl_args.end, argv[0]);
        return 1;
    }

    adxl_rd_reg(adxl_args.addr->ival[0],
            rx_buf,
            adxl_args.data->ival[0]);

    i=0;
    for(i=0;i<adxl_args.data->ival[0];i++)
        printf("%d: %x\n",i+adxl_args.addr->ival[0],rx_buf[i+1]);
    return 0;
}

static int awr_reg(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &adxl_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, adxl_args.end, argv[0]);
        return 1;
    }

    adxl_wr_reg(adxl_args.addr->ival[0],
            adxl_args.data->ival[0]);
    return 0;
}

static void register_adxl_rd()
{
    adxl_args.addr = arg_int1(NULL, NULL, "<a>", "reg base addr");
    adxl_args.data = arg_int1(NULL, NULL, "<c>", "read reg count");
    adxl_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "ard_reg",
            .help = "Read adxl regs",
            .hint = NULL,
            .func = &ard_reg,
            .argtable = &adxl_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_adxl_wr()
{
    adxl_args.addr = arg_int1(NULL, NULL, "<a>", "reg base addr");
    adxl_args.data = arg_int1(NULL, NULL, "<d>", "write data");
    adxl_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "awr_reg",
            .help = "Write adxl regs",
            .hint = NULL,
            .func = &awr_reg,
            .argtable = &adxl_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_adxl_info()
{
    const esp_console_cmd_t cmd = {
            .command = "adxl_info",
            .help = "Get adxl_dev infomation",
            .hint = NULL,
            .func = &adxl_info
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void adxl_register(void)
{
    register_adxl_rd();
    register_adxl_wr();
    register_adxl_info();
//    register_fft_info();
}


