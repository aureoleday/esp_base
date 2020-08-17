#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "sys_conf.h"

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
//#define GPIO_OUTPUT_PIN_SEL (  (1ULL<<PWR_EN) | (1ULL<<LOCAL_LED) | (1ULL<<WIFI_LED0) | (1ULL<<WIFI_LED1) | (1ULL<<LOW_PWR) | (1ULL<<VI_OD) | (1ULL<<PGA_GAIN0) | (1ULL<<PGA_GAIN1) ) 
#define GPIO_OUTPUT_PIN_SEL (  (1ULL<<PWR_EN) | (1ULL<<LOCAL_LED) | (1ULL<<WIFI_LED1) | (1ULL<<LOW_PWR) | (1ULL<<VI_OD) | (1ULL<<PGA_GAIN0) | (1ULL<<PGA_GAIN1) ) 

enum
{
    WIFI_STDBY = 0,
    WIFI_CONNECTED,
    DATA_TRANSMITTING
};
typedef struct
{	
    uint8_t 		led_sts;
}io_st;

static io_st io_inst;

void power_fsm(void* param)
{
    uint8_t pfsm = 0;
    uint8_t delay_cnt = 0;
    while(1)
    {
        switch(pfsm)
        {
            case (0):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    pfsm = 1;
                    delay_cnt = 1;
                }
                else
                {
                    pfsm = 0;
                    delay_cnt = 0;
                }
                break;
            }
            case (1):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    if(delay_cnt <= 3)
                    {
                        delay_cnt++;
                        pfsm = 1;
                    }
                    else
                    {
                        delay_cnt = 0;
                        pfsm = 2;
                        gpio_set_level(PWR_EN, 1);
                    }
                }
                else
                {
                    delay_cnt = 0;
                    pfsm = 0;
                }
                break;
            }
            case (2):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    pfsm = 3;
                    delay_cnt = 1;
                }
                else
                {
                    pfsm = 2;
                    delay_cnt = 0;
                }
                break;
            }
            case (3):
            {
                if(gpio_get_level(PWR_ON) == 1)
                {
                    if(delay_cnt <= 4)
                    {
                        delay_cnt++;
                        pfsm = 3;
                    }
                    else
                    {
                        delay_cnt = 0;
                        pfsm = 0;
                        gpio_set_level(PWR_EN, 0);
                    }
                }
                else
                {
                    delay_cnt = 0;
                    pfsm = 0;
                }
                break;
            }
            default:
            {
                delay_cnt = 0;
                pfsm = 0;
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
//    gpio_set_level(WIFI_LED0, 0);
    gpio_set_level(WIFI_LED1, 0);
    gpio_set_level(LOW_PWR, 0);
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
    static uint8_t flag=0;
    switch(io_inst.led_sts)
    {
        case(WIFI_STDBY):
        {
            gpio_set_level(WIFI_LED0, flag);
            gpio_set_level(WIFI_LED1, 0);
            break;
        }
        case(WIFI_CONNECTED):
        {
            gpio_set_level(WIFI_LED0, 1);
            gpio_set_level(WIFI_LED1, 0);
            break;
        }
        case(DATA_TRANSMITTING):
        {
            gpio_set_level(WIFI_LED0, 1);
            gpio_set_level(WIFI_LED1, 1);
            break;
        }
        default:
        {
            gpio_set_level(WIFI_LED0, 0);
            gpio_set_level(WIFI_LED1, 0);
            break;
        }
    }
    if(flag == 0)
        flag = 1;
    else
        flag = 0;
    gpio_set_level(LOCAL_LED, flag);
}

static void led_tim_init(void)
{
    TimerHandle_t led_tim;

    led_tim = xTimerCreate(   
              "Led_Timer",       // Just a text name, not used by the kernel.
               ( 20 ),   // The timer period in ticks.
               pdTRUE,    // The timers will auto-reload themselves when they expire.
               NULL,  // Assign each timer a unique id equal to its array index.
               led_timer_cb // Each timer calls the same callback when it expires.
               );
    xTimerStart(led_tim,0);
}

void io_init(void)
{
    input_init();
    output_init();
    led_tim_init();
    xTaskCreate(&power_fsm,
            "Task_pwr_fsm",
            1024,
            NULL,
            31,
            NULL);

}

void pga_gain(uint8_t gain_ind)
{
    gpio_set_level(PGA_GAIN0, gain_ind&0x01);
    gpio_set_level(PGA_GAIN1, (gain_ind>>1)&0x01);
}

