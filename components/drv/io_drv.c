#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "driver/gpio.h"
#include "sys_conf.h"
#include "bit_op.h"
#include "esp_log.h"

#define     PWR_ON          35 
#define     BAT_CHRG        34	
#define     VI_OD           33 
#define     VI_EF           25	

#define     PWR_EN          27	
#define     LOW_PWR         4	
#define     WIFI_LED0       0	
#define     WIFI_LED1       2 
#define     PGA_GAIN0       22 
#define     PGA_GAIN1       21 
#define     LOCAL_LED       32 

#define     Bit_RESET	    0
#define     Bit_SET		    1	

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL ( (1ULL<<PWR_ON) | (1ULL<<BAT_CHRG) | (1ULL<<VI_EF) )
#define GPIO_OUTPUT_PIN_SEL (  (1ULL<<PWR_EN) | (1ULL<<LOCAL_LED) | (1ULL<<WIFI_LED0) | (1ULL<<WIFI_LED1) | (1ULL<<LOW_PWR) | (1ULL<<VI_OD) | (1ULL<<PGA_GAIN0) | (1ULL<<PGA_GAIN1) ) 

static const char *TAG = "DRV_IO";

enum
{
    WIFI_STDBY = 0,
    WIFI_CONNECTED,
    DATA_TRANSMITTING
};

typedef struct
{	
    uint8_t 		led_sts;
    uint8_t 		pwr_fsm;
}io_st;


static io_st io_inst;
static void io_register(void);

static void io_ds_init(void)
{
    io_inst.led_sts = WIFI_STDBY ;
    io_inst.pwr_fsm = PWR_MODE_IDLE;
}

void power_fsm(void* param)
{
    uint8_t delay_cnt = 0;
	vTaskDelay(100 / portTICK_PERIOD_MS);
    while(1)
    {
        switch(io_inst.pwr_fsm)
        {
            case (PWR_MODE_IDLE ):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    io_inst.pwr_fsm = PWR_MODE_PRON;
                    delay_cnt = 1;
                    ESP_LOGI(TAG,"to PWR_MODE_PRON");
                }
                else
                {
                    io_inst.pwr_fsm = PWR_MODE_IDLE ;
                    delay_cnt = 0;
                }
                break;
            }
            case (PWR_MODE_PRON):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    if(delay_cnt <= 1)
                    {
                        delay_cnt++;
                        io_inst.pwr_fsm = PWR_MODE_PRON;
                    }
                    else
                    {
                        delay_cnt = 0;
                        io_inst.pwr_fsm = PWR_MODE_ON;
                        gpio_set_level(PWR_EN, 1);
                        ESP_LOGI(TAG,"to PWR_MODE_ON");
                    }
                }
                else
                {
                    delay_cnt = 0;
                    io_inst.pwr_fsm = PWR_MODE_IDLE;
                    ESP_LOGI(TAG,"to PWR_MOD_IDLE");
                }
                break;
            }
            case (PWR_MODE_ON):
            {
                if(gpio_get_level(PWR_ON) == 0)
                {
                    io_inst.pwr_fsm = PWR_MODE_ONGAP;
                    delay_cnt = 0;
                    ESP_LOGI(TAG,"to PWR_MOD_ONGAP");
                }
                else
                {
                    io_inst.pwr_fsm = PWR_MODE_ON;
                    delay_cnt = 0;
                }
                break;
            }
            case (PWR_MODE_ONGAP):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    io_inst.pwr_fsm = PWR_MODE_PROFF;
                    delay_cnt = 0;
                    ESP_LOGI(TAG,"to PWR_MODE_PROFF");
                }
                else
                {
                    io_inst.pwr_fsm = PWR_MODE_ONGAP;
                    delay_cnt = 0;
                }
                break;
            }
            case (PWR_MODE_PROFF):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    if(delay_cnt <= 3)
                    {
                        delay_cnt++;
                        io_inst.pwr_fsm = PWR_MODE_PROFF;
                    }
                    else
                    {
                        delay_cnt = 0;
                        io_inst.pwr_fsm = PWR_MODE_OFF;
                        gpio_set_level(PWR_EN, 0);
                        ESP_LOGI(TAG,"to PWR_MODE_OFF");
                    }
                }
                else
                {
                    delay_cnt = 0;
                    io_inst.pwr_fsm = PWR_MODE_ONGAP;
                    ESP_LOGI(TAG,"to PWR_MOD_ONGAP");
                }
                break;
            }
            case (PWR_MODE_OFF):
            {
                delay_cnt = 0;
                io_inst.pwr_fsm = PWR_MODE_OFF;
                gpio_set_level(PWR_EN, 0);
                break;
            }
            default:
            {
                delay_cnt = 0;
                io_inst.pwr_fsm = PWR_MODE_IDLE;
                gpio_set_level(PWR_EN, 0);
                break;
            }
        }
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void output_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19fasdf
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(LOCAL_LED, 1);
    gpio_set_level(WIFI_LED0, 0);
    gpio_set_level(WIFI_LED1, 0);
    gpio_set_level(LOW_PWR, 1);
    gpio_set_level(PWR_EN, 0);
    gpio_set_level(PGA_GAIN0, 0);
    gpio_set_level(PGA_GAIN0, 0);
    gpio_set_level(VI_OD, 0);
} 

static void input_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19fasdf
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

static void led_timer_cb(void* arg)
{
	extern sys_reg_st g_sys;
    static uint8_t flag=0;
    if(io_inst.pwr_fsm == PWR_MODE_OFF) 
    {
        gpio_set_level(WIFI_LED0, 0);
        gpio_set_level(WIFI_LED1, 0);
        gpio_set_level(LOCAL_LED, 1);
    }
    else if(io_inst.pwr_fsm >= PWR_MODE_ON) 
    {
        if(bit_op_get(g_sys.stat.gen.status_bm,GBM_WIFI) == 0)
        {
            gpio_set_level(WIFI_LED0, flag);
        }
        else
        {
            gpio_set_level(WIFI_LED0, 1);
        }

        if(bit_op_get(g_sys.stat.gen.status_bm,GBM_TCP) == 0)
        {
            gpio_set_level(WIFI_LED1, !flag);
        }
        else
        {
            gpio_set_level(WIFI_LED1, 1);
        }
        gpio_set_level(LOCAL_LED, flag);
    }
    else 
    {
        gpio_set_level(WIFI_LED0, 0);
        gpio_set_level(WIFI_LED1, 0);
        gpio_set_level(LOCAL_LED, 1);
    }

    if(flag == 0)
        flag = 1;
    else
        flag = 0;
}

static void led_tim_init(void)
{
    TimerHandle_t led_tim;

    led_tim = xTimerCreate(   
              "Led_Timer",       // Just a text name, not used by the kernel.
               ( 30 ),   // The timer period in ticks.
               pdTRUE,    // The timers will auto-reload themselves when they expire.
               NULL,  // Assign each timer a unique id equal to its array index.
               led_timer_cb // Each timer calls the same callback when it expires.
               );
    xTimerStart(led_tim,0);
}

void pga_gain(uint8_t gain_ind)
{
    gpio_set_level(PGA_GAIN0, gain_ind&0x01);
    gpio_set_level(PGA_GAIN1, (gain_ind>>1)&0x01);
}

void io_init(void)
{
	extern sys_reg_st g_sys;
    io_ds_init();
    input_init();
    output_init();
    led_tim_init();
    io_register(); 
    pga_gain(g_sys.conf.adc.gain);
    xTaskCreate(&power_fsm,
            "Task_pwr_fsm",
            1024,
            NULL,
            31,
            NULL);
}

static struct {
    struct arg_int *gain;
    struct arg_end *end;
} pga_args;

static int cmd_pga_gain(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &pga_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, pga_args.end, argv[0]);
        return 1;
    }
    pga_gain(pga_args.gain->ival[0]);

    ESP_LOGI(TAG,"%d",pga_args.gain->ival[0]);

    return 0;
}

static void register_pga_gain()
{
    pga_args.gain = arg_int1(NULL, NULL, "<g>", "gain ind");
    pga_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "adc_gain",
            .help = "adc gain set",
            .hint = NULL,
            .func = &cmd_pga_gain,
            .argtable = &pga_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void io_register(void)
{
    register_pga_gain();
}

