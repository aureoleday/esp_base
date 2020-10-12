#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "kfifo.h"
#include "cmd_resolve.h"
//#include "mqtt_tcp.h"
#include "sys_conf.h"
#include "bit_op.h"
#include "adxl_drv.h"
#include "ads131_drv.h"
#include "goertzel.h"


#define     DAQ_RX_BUF_DEPTH    1024
#define     DAQ_TX_BUF_DEPTH    1536 
#define     DAQ_CHANNEL_MAX     16 

static const char *TAG = "DAQ";

uint8_t test_buf[1024]={0};

typedef struct
{
    uint8_t  tx_buf[DAQ_TX_BUF_DEPTH];
    uint8_t  rx_buf[DAQ_RX_BUF_DEPTH];
    uint16_t sample_rate;
    uint16_t channel_bm;
    uint32_t pkg_period;
    uint16_t gain[DAQ_CHANNEL_MAX];
}daq_st;

static daq_st daq_inst;
esp_timer_handle_t daq_timer;

/**
 * @brief  cmd rtx buffer initialization
 * @param  none
 * @retval none
 */
static void daq_buf_init(void)
{
    memset(daq_inst.tx_buf,0,DAQ_TX_BUF_DEPTH);
    memset(daq_inst.rx_buf,0,DAQ_RX_BUF_DEPTH);

    goertzel_init();

    daq_inst.sample_rate = 4096;
    daq_inst.pkg_period = 1000000;
    daq_inst.channel_bm = 0x0001;
}

static void daq_timeout(void* arg)
{
    extern sys_reg_st  g_sys;
    int out_len = 0;
    int o_len = 0;
#if (DEV_TYPE == DEV_DIGITAL) 
    out_len = adxl_dout(daq_inst.tx_buf ,g_sys.conf.daq.pkg_size);
#else
    out_len = adc_dout(daq_inst.tx_buf ,g_sys.conf.daq.pkg_size);
#endif
    o_len = out_len>>2;
    if(out_len == 0)
        ESP_LOGD(TAG,"No daq data");
    else 
    {
        //if(bit_op_get(g_sys.stat.gen.status_bm,GBM_TCP) != 0)
        if((bit_op_get(g_sys.stat.gen.status_bm,GBM_TCP) != 0)&&(g_sys.conf.daq.pkg_en == 1))
            daq_frame(daq_inst.tx_buf, out_len);
        for(int i=0;i<o_len;i++)
        {
#if (DEV_TYPE == DEV_DIGITAL) 
            goertzel_lfilt(*((int32_t *)daq_inst.tx_buf+i)*0.0000039);
#else
            goertzel_lfilt(*((int32_t *)daq_inst.tx_buf+i)*0.00000009933);
#endif
        }
    }   
    //}
}

void daq_tim_stop(void)
{
    esp_timer_stop(daq_timer);
}

void daq_tim_start(int32_t tim_period)
{
    ESP_ERROR_CHECK(esp_timer_start_periodic(daq_timer, tim_period));
}

static int daq_timer_init(void)
{
    const esp_timer_create_args_t daq_timer_args = {
            .callback = &daq_timeout,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };

    ESP_ERROR_CHECK(esp_timer_create(&daq_timer_args, &daq_timer));
    return 0;
}

void daq_init(void)
{
    daq_buf_init();
    daq_timer_init();
}
