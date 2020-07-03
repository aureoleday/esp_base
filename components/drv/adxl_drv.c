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
#include "kfifo.h"
#include "goertzel.h"

#define min(a,b)  ((a)>(b) ? (b) : (a))            /*  */

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

static const char *TAG = "ADXL";

static uint8_t 	rxd_temp[DEV_GEO_RTX_SIZE];
static uint8_t 	kf_buf_s[DEV_GEO_FIFO_SIZE];
kfifo_t 		kf_s;

static SemaphoreHandle_t geospi_mutex = NULL;

esp_timer_handle_t adxl_timer;

typedef struct
{
    spi_device_handle_t     		spi_device_h;
    uint8_t                         txd[DEV_GEO_RTX_SIZE];
    uint8_t                         rxd[DEV_GEO_RTX_SIZE];
}spi_geo_device_st;

spi_geo_device_st spi_geo_dev_inst;
fft_st fft_inst;

static int16_t adxl355_scanfifo(void);

static void adxl_timeout(void* arg)
{
    adxl355_scanfifo();
}

static int adxl_timer_init(void)
{
    const esp_timer_create_args_t adxl_timer_args = {
            .callback = &adxl_timeout,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };

    ESP_ERROR_CHECK(esp_timer_create(&adxl_timer_args, &adxl_timer));
    return 0;
}

void adxl_tim_stop(void)
{
    esp_timer_stop(adxl_timer);
}

void adxl_tim_start(int32_t tim_period)
{
    ESP_ERROR_CHECK(esp_timer_start_periodic(adxl_timer, tim_period));
}

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

int16_t geo_get_time(float* dst_ptr,uint16_t len)
{
    int16_t ret;

    ret = kfifo_out_peek(&kf_s,dst_ptr,(len<<2));
    fft_inst.ibuf_cnt = ret/4;
    memcpy(fft_inst.ibuf,dst_ptr,ret);

    ret /= 4;
    return ret;
}

static void fft_max_ind(void)
{
    extern sys_reg_st  g_sys;
    uint16_t fft_max = 0,i;

    float max_temp;
    float max_ind;
    //	float tt1;

    fft_max = (1<<g_sys.conf.fft.n);
    max_temp = fft_inst.obuf[0];
    max_ind = 0.001;
    for(i=1;i<fft_max;i++)
    {
        if(max_temp < fft_inst.obuf[i])
        {
            max_temp = fft_inst.obuf[i];
            max_ind = ((float)4000/(float)((1<<(g_sys.conf.geo.filter&0x0f))<<g_sys.conf.fft.n))*(float)i;
        }
    }

    if(fft_inst.arr_cnt <16)
    {
        fft_inst.ampl_arr[fft_inst.arr_cnt] = max_temp;
        fft_inst.freq_arr[fft_inst.arr_cnt] = max_ind;
        fft_inst.arr_cnt++;
    }
    else
    {
        for(i=0;i<15;i++)
        {
            fft_inst.ampl_arr[i] = fft_inst.ampl_arr[i+1];
            fft_inst.freq_arr[i] = fft_inst.freq_arr[i+1];
        }
        fft_inst.ampl_arr[15] = max_temp;
        fft_inst.freq_arr[15] = max_ind;
    }
}

float* geo_get_fft(uint16_t* fft_len)
{
    extern sys_reg_st  g_sys;
    uint16_t fft_max = 0,off;

    fft_max = (1<<g_sys.conf.fft.n);

    off = fft_inst.ibuf_cnt - min(fft_max,fft_inst.ibuf_cnt);

    if(fft_inst.ibuf_cnt > 0)
    {
        fft_new(1<<g_sys.conf.fft.n);
        fft_calc((fft_inst.ibuf+off),fft_inst.obuf);
        fft_inst.state = 1;
        *fft_len = fft_inst.ibuf_cnt;
        fft_max_ind();
        return fft_inst.obuf;
    }
    else
    {
        fft_inst.state = 0;
        *fft_len = 0;
        return NULL;
    }
}


void geo_ds_init(void)
{
    kf_init();
    fft_inst.arr_cnt = 0;
    for(int i=0;i<16;i++)
    {
        fft_inst.freq_arr[i] = 0.00000001;
        fft_inst.ampl_arr[i] = 0.00000001;
    }
    geospi_mutex = xSemaphoreCreateMutex();

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

    esp_err_t ret = spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
//    esp_err_t ret = spi_device_polling_transmit(spi_geo_dev_inst.spi_device_h, &t);

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

    spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);

    xSemaphoreGive( geospi_mutex );

    return *(rx_buf+1);
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
            .queue_size=12,                          //We want to be able to queue 12 transactions at a time
            //        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    geo_ds_init();
    adxl_timer_init();
    ESP_ERROR_CHECK(ret);
    //Attach the device to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi_geo_dev_inst.spi_device_h);
    ESP_ERROR_CHECK(ret);

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
    float temp;
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
            temp = (float)decode(dbuf[axis])*0.0000039;
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

static int16_t adxl355_scanfifo(void)
{
    static int16_t cooldown = 0;
    extern sys_reg_st  g_sys;
    int16_t  err_no;
    uint16_t i;
    uint16_t total_cnt;
    uint8_t  sample_cnt;
    uint32_t buf_temp;
    uint8_t status;

    err_no = 0;
    status = adxl_rd_reg(ADXL_STATUS,rxd_temp,2);
    if((status&0x4) != 0)
    {
        if(cooldown==0)
        {
            cooldown = 1000;
            ESP_LOGW(TAG,"F_OVR!");
        }
        adxl_rd_reg(ADXL_FIFO_DATA, rxd_temp, 96*2);
        err_no = -1;
        return err_no;
    }

    if(cooldown > 0)
        cooldown--;

    sample_cnt = rxd_temp[2];

    total_cnt = sample_cnt*3;
    //printf("%d\n",sample_cnt);

    if(status > 0)
    {
        adxl_rd_reg(ADXL_FIFO_DATA, rxd_temp, total_cnt);
        for(i=0;i<sample_cnt;i++)
        {
            buf_temp = (rxd_temp[1+i*3]<<16)|(rxd_temp[2+i*3]<<8)|(rxd_temp[3+i*3]);
            err_no = raw_data_buf(buf_temp,g_sys.conf.geo.axis);
        }
    }
    return err_no;
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

//static int fft_info(int argc, char **argv)
//{
//	uint32_t temp[16];
//	int len;
//	int i;
//	for(i=0;i<16;i++)
//		temp[i] = 0;
//	len = kfifo_out_peek(&kf_s,temp,16*4);
//	for(i=0;i<16;i++)
//	{
//		printf("%d:%x\n",i,temp[i]);
//	}
//	printf("rd len:%d\n",len);
//
//	return 0;
//}

static int fft_info(int argc, char **argv)
{
    int i;
    printf("ind\tfreq\t\tamp\n");
    for(i=0;i<16;i++)
    {
        printf("%d\t%f\t%f\n",i,fft_inst.freq_arr[i],fft_inst.ampl_arr[i]);
    }
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

static void register_fft_info()
{
    const esp_console_cmd_t cmd = {
            .command = "fft_info",
            .help = "Get fft infomation",
            .hint = NULL,
            .func = &fft_info
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


void adxl_register(void)
{
    register_adxl_rd();
    register_adxl_wr();
    register_adxl_info();
    register_fft_info();
}


