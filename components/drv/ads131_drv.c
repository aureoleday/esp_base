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
#include "sys_conf.h"
#include "kfifo.h"
#include "ads131_drv.h"

#define min(a,b)  ((a)>(b) ? (b) : (a))            /*  */

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_DVLD 13 
#define PIN_NUM_NRST 14 

#define GPIO_INPUT_PIN_SEL  (1ULL<<PIN_NUM_DVLD)
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<PIN_NUM_NRST) 

#define CMD_NULL     0x0000
#define CMD_RST      0x0011
#define CMD_LOCK     0x0555
#define CMD_UNLOCK   0x0655
#define CMD_WAKEUP   0x0033
#define CMD_STDBY    0x0022
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "ADS131";

static uint8_t 	kf_buf_s[ADC_FIFO_SIZE];
kfifo_t 		kf_s,kf_dl;
static SemaphoreHandle_t adcspi_mutex = NULL;
static spi_transaction_t adc_t;

typedef struct
{
    uint16_t                        cd;
    uint8_t                         stat;
    int32_t                         buf[64];
    int32_t                         obuf;
}delay_buf_st;

typedef struct
{
    spi_device_handle_t     		spi_device_h;
    uint8_t                         txd[16];
    uint8_t                         rxd[16];
    volatile uint16_t               adc_cmd;
    uint16_t                        adc_init_done;
    int32_t                         adc_val;
}spi_geo_device_st;

spi_geo_device_st   spi_geo_dev_inst;
delay_buf_st        dlbuf_inst;

static void adc_register(void);
static uint16_t adc_sread(void);

static void dlbuf_init(void)
{
    memset(&kf_dl, 0, sizeof(kf_dl));
    kfifo_init(&kf_dl, (void *)dlbuf_inst.buf, sizeof(dlbuf_inst.buf));
    printf("dlbuf size:%d\n",sizeof(dlbuf_inst.buf));
    dlbuf_inst.cd = 0;
    dlbuf_inst.stat = 0;
    dlbuf_inst.obuf = 0;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    if(spi_geo_dev_inst.adc_init_done == 1)
        adc_sread(); 
}
static int32_t decode(uint32_t din)
{
    uint32_t temp;
    temp = din;
    if((temp&0x800000) != 0)
        temp |= 0xff000000;

    return (int32_t)temp;
}

static void sig_mav(int32_t din)
{
	extern sys_reg_st g_sys;
    static int32_t peak_val = 0;
    static int16_t cnt = 0;
    static int16_t init = 0;
    uint16_t atten = g_sys.conf.adc.mav_atten;
    if(cnt < g_sys.conf.gtz.sample_freq)
    {
        if(din > peak_val)
        {
            peak_val = din;
        }
        cnt++;
    }
    else
    {
        if(init == 0)
        {
            init = 1;
            g_sys.stat.adc.peak = peak_val;
        }
        else
        {
            g_sys.stat.adc.peak = g_sys.stat.adc.peak - (g_sys.stat.adc.peak>>atten) + (peak_val>>atten);
        }
        peak_val = 0;
        cnt = 0;
    }
}
static int32_t data_delay(int32_t* dout, int32_t din)
{
	extern sys_reg_st g_sys;
    int32_t temp;
    uint16_t len;
    temp = din; 
    
    if(dlbuf_inst.cd > 0)
    {
        dlbuf_inst.cd--;
        dlbuf_inst.stat = 2;
    }
    else
    {
        if(temp > (((int32_t)g_sys.stat.adc.peak)<<g_sys.conf.adc.drop_th))
        {
            dlbuf_inst.stat = 2;
            dlbuf_inst.cd = g_sys.conf.adc.drop - g_sys.conf.adc.pre_drop - 1;
            kfifo_reset(&kf_dl);
        }
        else

        {
            len = kfifo_len(&kf_dl)>>2;
          
            if(len < g_sys.conf.adc.pre_drop)
            {
                dlbuf_inst.stat = 1;
                kfifo_in(&kf_dl,&temp,sizeof(int32_t));
            }
            else
            {
                dlbuf_inst.stat = 0;
                //kfifo_out(&kf_dl,&dlbuf_inst.obuf,sizeof(int32_t));
                kfifo_out(&kf_dl,dout,sizeof(int32_t));
                kfifo_in(&kf_dl,&temp,sizeof(int32_t));
            }
        }
    }
    return dlbuf_inst.stat; 
}


static void IRAM_ATTR adc_read_pcb(spi_transaction_t* t)
{
	extern sys_reg_st g_sys;
    int32_t temp[4];
    int32_t dummy[4];
    uint8_t cnt=0;
    for(int i=0;i<3;i++)
    {
        if(((g_sys.conf.adc.ch_bm>>i)&0x01) == 1)
        {
            dummy[i]=((spi_geo_dev_inst.rxd[i*3+3]<<16)|(spi_geo_dev_inst.rxd[i*3+4]<<8)|spi_geo_dev_inst.rxd[i*3+5]);
            temp[cnt] = decode(dummy[i]);
            cnt++;
        }
    }

    if(g_sys.conf.adc.enable == 1)
    {
        if(kfifo_len(&kf_s) >= kf_s.size)
        {
            kfifo_out(&kf_s,&dummy,cnt*sizeof(int32_t));
            g_sys.stat.geo.kfifo_drop_cnt++;
        }
        if(g_sys.conf.adc.drop_en == 1)
        {
            if(0 == data_delay(dummy,temp[0]))
            {
                kfifo_in(&kf_s,dummy,cnt*sizeof(int32_t));
                sig_mav(dummy[0]);
            }
        }
        else
        {
            kfifo_in(&kf_s,&temp,cnt*sizeof(int32_t));
            sig_mav(temp[0]);
        }
        spi_geo_dev_inst.adc_val = temp[0];
    }
}

//static void IRAM_ATTR adc_read_pcb(spi_transaction_t* t)
//{
//	extern sys_reg_st g_sys;
//    int32_t temp[4];
//    int32_t dummy[4];
//    uint8_t cnt=0;
//    for(int i=0;i<3;i++)
//    {
//        if(((g_sys.conf.adc.ch_bm>>i)&0x01) == 1)
//        {
//            dummy[i]=((spi_geo_dev_inst.rxd[i*3+3]<<16)|(spi_geo_dev_inst.rxd[i*3+4]<<8)|spi_geo_dev_inst.rxd[i*3+5]);
//            temp[cnt] = decode(dummy[i]);
//            cnt++;
//        }
//    }
//
//    if(g_sys.conf.adc.enable == 1)
//    {
//        if(kfifo_len(&kf_s) >= kf_s.size)
//        {
//            kfifo_out(&kf_s,&dummy,cnt*sizeof(int32_t));
//            g_sys.stat.geo.kfifo_drop_cnt++;
//        }
//        if(g_sys.conf.adc.drop_en == 1)
//        {
//            if(g_sys.stat.adc.drop_cnt>0)
//                g_sys.stat.adc.drop_cnt--;
//            else if(temp[0] > (((int32_t)g_sys.stat.adc.peak)<<g_sys.conf.adc.drop_th))
//            {
//                g_sys.stat.adc.drop_cnt = g_sys.conf.adc.drop-1;
//            }
//            else
//            {
//                kfifo_in(&kf_s,&temp,cnt*sizeof(int32_t));
//                sig_mav(temp[0]);
//            }
//        }
//        else
//        {
//            kfifo_in(&kf_s,&temp,cnt*sizeof(int32_t));
//            sig_mav(temp[0]);
//        }
//        spi_geo_dev_inst.adc_val = temp[0];
//    }
//}

static void kf_init(void)
{
    memset(&kf_s, 0, sizeof(kf_s));
    kfifo_init(&kf_s, (void *)kf_buf_s, sizeof(kf_buf_s));
}

void adc_ds_init(void)
{
    spi_geo_dev_inst.adc_init_done = 0;
    spi_geo_dev_inst.adc_cmd = CMD_NULL;
    kf_init();
    dlbuf_init();
    adcspi_mutex = xSemaphoreCreateMutex();
}

uint16_t adc_wr_reg(uint8_t addr, uint8_t data)
{
    uint16_t ret = 0;
    uint8_t rx_buf[12];
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));

    xSemaphoreTake( adcspi_mutex, portMAX_DELAY );

    t.length=24;
    t.rx_buffer = rx_buf;

    spi_geo_dev_inst.txd[0] = addr|0x40;
    spi_geo_dev_inst.txd[1] = data;
    spi_geo_dev_inst.txd[2] = 0;

    t.tx_buffer=spi_geo_dev_inst.txd;

    spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
    xSemaphoreGive( adcspi_mutex );

    ret = rx_buf[1];
    return ret;
}

uint16_t adc_rd_reg(uint8_t addr)
{
    uint16_t ret = 0;
    uint8_t rx_buf[12];
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));

    xSemaphoreTake( adcspi_mutex, portMAX_DELAY );

    t.length=24;
    t.rx_buffer = rx_buf;

    spi_geo_dev_inst.txd[0] = addr|0x20;
    spi_geo_dev_inst.txd[1] = 0;
    spi_geo_dev_inst.txd[2] = 0;

    t.tx_buffer=spi_geo_dev_inst.txd;
    spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
    xSemaphoreGive( adcspi_mutex );

    ret = rx_buf[1];
    return ret;
}

static uint16_t adc_cmd(uint16_t cmd_type)
{
    uint16_t ret = 0;
    uint8_t rx_buf[12];
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));

    xSemaphoreTake( adcspi_mutex, portMAX_DELAY );

    t.length=24;
    t.rx_buffer = rx_buf;

    spi_geo_dev_inst.txd[0] = (cmd_type>>8)&0x00ff;
    spi_geo_dev_inst.txd[1] = cmd_type&0x00ff;
    spi_geo_dev_inst.txd[2] = 0;

    t.tx_buffer=spi_geo_dev_inst.txd;
    spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
    xSemaphoreGive( adcspi_mutex );

    ret = (rx_buf[0]<<8)|rx_buf[1];
    return ret;
}

static uint16_t adc_sread(void)
{
    uint16_t ret = 0;
    uint16_t i=0;
    //spi_transaction_t t;
    //memset(&t, 0, sizeof(t));
    memset(&adc_t, 0, sizeof(adc_t));
    //xSemaphoreTake( adcspi_mutex, portMAX_DELAY );

    adc_t.length=72;//8*3*3 bits
    adc_t.rx_buffer = spi_geo_dev_inst.rxd;

    spi_geo_dev_inst.txd[0] = (spi_geo_dev_inst.adc_cmd>>8)&0x00ff;
    spi_geo_dev_inst.txd[1] = spi_geo_dev_inst.adc_cmd&0x00ff;
    
    for(i=2;i<9;i++)
    {
        spi_geo_dev_inst.txd[i] = 0;
    }

    adc_t.tx_buffer=spi_geo_dev_inst.txd;
    //spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
    spi_device_queue_trans(spi_geo_dev_inst.spi_device_h, &adc_t, 0);
    //xSemaphoreGive( adcspi_mutex );

    ret = 9;
    return ret;
}

static uint16_t calc_sps(uint16_t freq_mode)
{
    uint16_t clk1_reg,clk2_reg,clk_reg;
    switch(freq_mode)
    {
        case(0):
        {
            //1024 SPS
            clk1_reg = 0x0a;    //      /10
            clk2_reg = 0x89;    //      /8/200
            break;
        }
        case(1):
        {
            //2048 SPS
            clk1_reg = 0x0a;    //      /10
            clk2_reg = 0x49;    //      /4/200
            break;
        }
        case(2):
        {
            //4096 SPS
            clk1_reg = 0x0a;    //      /10
            clk2_reg = 0x29;    //      /2/200
            break;
        }
        case(3):
        {
            //1k SPS
            clk1_reg = 0x08;    //      /8
            clk2_reg = 0x88;    //      /8/256
            break;
        }
        case(4):
        {
            //2k SPS
            clk1_reg = 0x08;    //      /8
            clk2_reg = 0x8b;    //      /8/128
            break;
        }
        case(5):
        {
            //4k SPS
            clk1_reg = 0x08;    //      /8
            clk2_reg = 0x8d;    //      /8/64
            break;
        }
        case(6):
        {
            //8k SPS
            clk1_reg = 0x08;    //      /8
            clk2_reg = 0x8f;    //      /8/32
            break;
        }
        case(7):
        {
            //16k SPS
            clk1_reg = 0x08;    //      /8
            clk2_reg = 0x4f;    //      /4/32
            break;
        }
        case(8):
        {
            //32k SPS
            clk1_reg = 0x08;    //      /8
            clk2_reg = 0x2f;    //      /2/32
            break;
        }
        case(9):
        {
            //64k SPS
            clk1_reg = 0x04;    //      /4
            clk2_reg = 0x2f;    //      /2/32
            break;
        }
        default:
        {
            //4096 SPS
            clk1_reg = 0x0a;    //      /10
            clk2_reg = 0x29;    //      /2/200
            break;
        }
    }
    clk_reg = (clk1_reg<<8)|clk2_reg;
    return clk_reg;
}

uint16_t adc_set_sps(uint16_t adc_sps_mode)
{
    uint16_t ret,freq_mod;
    freq_mod = calc_sps(adc_sps_mode);
    ret = adc_wr_reg(13, (freq_mod>>8)&0x00ff);//f_iclk = f_clkin/10 
    ret = adc_wr_reg(14, freq_mod&0x00ff);//f_mod = f_iclk/2, f_data = f_mode/200
    return ret;
}

static uint16_t adc_init_conf(void)
{
	extern sys_reg_st g_sys;
    uint16_t ret;
    ret = adc_cmd(CMD_NULL);
    //ESP_LOGD(TAG,"adc:%x",ret);
    ret = adc_cmd(CMD_UNLOCK);
    //ESP_LOGD(TAG,"adc:%x",ret);
    adc_set_sps(g_sys.conf.adc.sps);
    ret = adc_wr_reg(15, 0x03);//adc enable 
    //ESP_LOGD(TAG,"adc:%x",ret);
    ret = adc_cmd(CMD_WAKEUP);
    //ESP_LOGD(TAG,"adc:%x",ret);
    ret = adc_cmd(CMD_LOCK);
    //ESP_LOGD(TAG,"adc:%x",ret);
    ret = adc_cmd(CMD_NULL);
    //ESP_LOGD(TAG,"adc:%x",ret);
    //adc_cmd(CMD_UNLOCK);
    spi_geo_dev_inst.adc_init_done = 1;
    return ret;
}

static void adc_pin_init(void)
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
    gpio_isr_handler_add(PIN_NUM_DVLD , gpio_isr_handler, (void*) PIN_NUM_DVLD);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_OUTPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(PIN_NUM_NRST, 1);

}

void adc_init(void)
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
            .clock_speed_hz=24*1000*1000,           //Clock out at 16 MHz
            .mode=1,                                //SPI mode 1
            .spics_io_num=PIN_NUM_CS,               //CS pin
            .post_cb = adc_read_pcb,
            .cs_ena_pretrans = 1,
            .cs_ena_posttrans = 1,
            .queue_size=1,               //We want to be able to queue 12 transactions at a time
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    adc_ds_init();
    //adc_timer_init();
    ESP_ERROR_CHECK(ret);
    //Attach the device to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi_geo_dev_inst.spi_device_h);
    ESP_ERROR_CHECK(ret);
    adc_register();
    //adc_init_conf();
    adc_pin_init();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    adc_init_conf();
}

int adc_dout(uint8_t * dst_ptr, uint16_t max_len)
{
    int rd_len = 0;
    int fifo_len = kfifo_len(&kf_s);
    rd_len = (fifo_len<max_len)? fifo_len:max_len;
    kfifo_out(&kf_s, dst_ptr ,rd_len);
    //ESP_LOGI(TAG,"f_len:%d,max_len:%d,r_len:%d",fifo_len,max_len,rd_len);
    return rd_len; 
}

/** Arguments used by 'join' function */
static struct {
    struct arg_int *addr;
    struct arg_int *data;
    struct arg_end *end;
} adc_wargs;

static struct {
    struct arg_int *addr;
    struct arg_end *end;
} adc_args;

static struct {
    struct arg_int *cmd;
    struct arg_end *end;
} adc_cmd_args;

static int adc_cmd_r(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &adc_cmd_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, adc_cmd_args.end, argv[0]);
        return 1;
    }
    spi_geo_dev_inst.adc_cmd = adc_cmd_args.cmd->ival[0];

    ESP_LOGI(TAG,"%x,%x",spi_geo_dev_inst.adc_cmd,adc_cmd_args.cmd->ival[0]);

    return 0;
}

static int xadc_cmd(int argc, char **argv)
{
    uint16_t rd_data;

    int nerrors = arg_parse(argc, argv, (void**) &adc_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, adc_args.end, argv[0]);
        return 1;
    }

    rd_data = adc_cmd(adc_args.addr->ival[0]);

    ESP_LOGI(TAG,"%x: %x",adc_args.addr->ival[0],rd_data);
    return 0;
}


static int radc_reg(int argc, char **argv)
{
    uint16_t rd_data;

    int nerrors = arg_parse(argc, argv, (void**) &adc_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, adc_args.end, argv[0]);
        return 1;
    }

    rd_data = adc_rd_reg(adc_args.addr->ival[0]);

    ESP_LOGI(TAG,"%d: %x",adc_args.addr->ival[0],rd_data);
    return 0;
}

static int wadc_reg(int argc, char **argv)
{
    uint16_t wr_data;
    int nerrors = arg_parse(argc, argv, (void**) &adc_wargs);
    if (nerrors != 0) {
        arg_print_errors(stderr, adc_wargs.end, argv[0]);
        return 1;
    }

    wr_data = adc_wr_reg(adc_wargs.addr->ival[0],
            adc_wargs.data->ival[0]);

    ESP_LOGI(TAG,"%d: %x",adc_wargs.addr->ival[0],wr_data);
    return 0;
}

static void register_adc_cmd_r()
{
    adc_cmd_args.cmd = arg_int1(NULL, NULL, "<c>", "cmd type");
    adc_cmd_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "adc_cmd_r",
            .help = "cmd adc inline",
            .hint = NULL,
            .func = &adc_cmd_r,
            .argtable = &adc_cmd_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static void register_adc_cmd()
{
    adc_args.addr = arg_int1(NULL, NULL, "<c>", "cmd type");
    adc_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "xadc_cmd",
            .help = "cmd adc",
            .hint = NULL,
            .func = &xadc_cmd,
            .argtable = &adc_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_adc_rd()
{
    adc_args.addr = arg_int1(NULL, NULL, "<a>", "reg base addr");
    adc_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "radc_reg",
            .help = "Read adc regs",
            .hint = NULL,
            .func = &radc_reg,
            .argtable = &adc_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_adc_wr()
{
    adc_wargs.addr = arg_int1(NULL, NULL, "<a>", "reg base addr");
    adc_wargs.data = arg_int1(NULL, NULL, "<d>", "write data");
    adc_wargs.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "wadc_reg",
            .help = "Write adc regs",
            .hint = NULL,
            .func = &wadc_reg,
            .argtable = &adc_wargs
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int sadc_data(int argc, char **argv)
{
    float temp;
    int32_t data;
    for(int i=0;i<9;i++)
        ESP_LOGI(TAG," %x ",spi_geo_dev_inst.rxd[i]);

    data = spi_geo_dev_inst.adc_val;
    temp = (float)data*0.000000596;

    ESP_LOGI(TAG,"val:%x,%f",data,temp);
    return 0;
}

static void register_adc_data()
{
    const esp_console_cmd_t cmd = {
            .command = "sadc_data",
            .help = "read spi adc data",
            .hint = NULL,
            .func = &sadc_data,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static void adc_register(void)
{
    register_adc_rd();
    register_adc_wr();
    register_adc_cmd();
    register_adc_data();
    register_adc_cmd_r();
    ESP_LOGI(TAG,"adc_registered");
}

