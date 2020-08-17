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
kfifo_t 		kf_s;

typedef struct
{
    spi_device_handle_t     		spi_device_h;
    uint8_t                         txd[16];
    uint8_t                         rxd[16];
}spi_geo_device_st;

spi_geo_device_st spi_geo_dev_inst;

static SemaphoreHandle_t adcspi_mutex = NULL;
static void adc_register(void);

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	extern sys_reg_st g_sys;
    static uint16_t scnt = 0;
    scnt++;
    g_sys.stat.gtz.rank = scnt;
    //if(scnt%2 == 0)
    //{
    //    printf("isr:%d\n",scnt);
    //}
}

static void kf_init(void)
{
    memset(&kf_s, 0, sizeof(kf_s));
    kfifo_init(&kf_s, (void *)kf_buf_s, sizeof(kf_buf_s));
}

void adc_ds_init(void)
{
    kf_init();
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

    ret = (rx_buf[0]<<8)|rx_buf[1];
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

    ret = (rx_buf[0]<<8)|rx_buf[1];
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

    spi_geo_dev_inst.txd[0] = (cmd_type>>8)|0x00ff;
    spi_geo_dev_inst.txd[1] = cmd_type|0x00ff;
    spi_geo_dev_inst.txd[2] = 0;

    t.tx_buffer=spi_geo_dev_inst.txd;
    spi_device_transmit(spi_geo_dev_inst.spi_device_h, &t);
    xSemaphoreGive( adcspi_mutex );

    ret = (rx_buf[0]<<8)|rx_buf[1];
    return ret;
}

static int16_t adc_init_conf(void)
{
    int16_t ret;
    ret = adc_cmd(CMD_UNLOCK);
    ret = adc_wr_reg(15, 0x03); 
    ret = adc_cmd(CMD_WAKEUP);
    ret = adc_cmd(CMD_LOCK);
    adc_cmd(CMD_UNLOCK);
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
            .clock_speed_hz=16*1000*1000,           //Clock out at 16 MHz
            .mode=1,                                //SPI mode 1
            .spics_io_num=PIN_NUM_CS,               //CS pin
            .queue_size=6,                          //We want to be able to queue 12 transactions at a time
            //        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
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
}


/** Arguments used by 'join' function */
static struct {
    struct arg_int *addr;
    struct arg_int *data;
    struct arg_end *end;
} adc_args;

static int radc_reg(int argc, char **argv)
{
    uint8_t rd_data;

    int nerrors = arg_parse(argc, argv, (void**) &adc_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, adc_args.end, argv[0]);
        return 1;
    }

    rd_data = adc_rd_reg(adc_args.addr->ival[0]);

    printf("%d: %x\n",adc_args.addr->ival[0],rd_data);
    return 0;
}

static int wadc_reg(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &adc_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, adc_args.end, argv[0]);
        return 1;
    }

    adc_wr_reg(adc_args.addr->ival[0],
            adc_args.data->ival[0]);

    return 0;
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
    adc_args.addr = arg_int1(NULL, NULL, "<a>", "reg base addr");
    adc_args.data = arg_int1(NULL, NULL, "<d>", "write data");
    adc_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "wadc_reg",
            .help = "Write adc regs",
            .hint = NULL,
            .func = &wadc_reg,
            .argtable = &adc_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static void adc_register(void)
{
    register_adc_rd();
    register_adc_wr();
    ESP_LOGI(TAG,"adc_registered");
}

