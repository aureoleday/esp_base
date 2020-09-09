#include "driver/adc.h"
#include "driver/dac.h"
#include "esp_adc_cal.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "sys_conf.h"
#include "esp_log.h"

static const char *TAG = "DRV_BAT";

#define DEFAULT_VREF    1100
#define MAV_MAX_CNT		128
#define ADC_CH          ADC1_CHANNEL_0
static esp_adc_cal_characteristics_t *adc_chars;

typedef struct
{
	uint32_t	mav_buffer[MAV_MAX_CNT];
    uint32_t    mav_cnt;
	uint32_t	accum_sum;
	uint32_t	buffer_ff;
}bat_st;

static bat_st bat_inst;

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG,"eFuse Two Point: Supported");
    } else {
        ESP_LOGI(TAG,"eFuse Two Point: NOT supported");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG,"eFuse Vref: Supported");
    } else {
        ESP_LOGI(TAG,"eFuse Vref: NOT supported");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG,"Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG,"Characterized using eFuse Vref");
    } else {
        ESP_LOGI(TAG,"Characterized using Default Vref");
    }
}


static void adc_init(void)
{
    check_efuse();
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CH,ADC_ATTEN_DB_11);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

//static uint32_t adc_get_raw(void)
//{
//    return adc1_get_raw(ADC_CH);
//}

static uint32_t adc_get_volt(void)
{
	extern sys_reg_st g_sys;
    uint32_t temp,volt;
    temp = adc1_get_raw(ADC_CH);
    volt = esp_adc_cal_raw_to_voltage(temp, adc_chars) - g_sys.conf.per.adc_offset;
    return volt;
}

static uint32_t bat_mav_calc(uint16_t mav_cnt_set)
{
	uint32_t bat_mav_volt = 0;
	uint32_t cur_volt = adc_get_volt();
	if(bat_inst.buffer_ff == 0)
	{
		bat_inst.mav_buffer[bat_inst.mav_cnt] = cur_volt; 
		bat_inst.accum_sum += bat_inst.mav_buffer[bat_inst.mav_cnt];
		bat_inst.mav_cnt ++;
		bat_mav_volt = bat_inst.accum_sum/bat_inst.mav_cnt;
		if(bat_inst.mav_cnt >= mav_cnt_set)
	    {
       	 	bat_inst.mav_cnt = 0;
        	bat_inst.buffer_ff = 1;
   		}
	}
	else
	{
		bat_inst.accum_sum -= bat_inst.mav_buffer[bat_inst.mav_cnt];
		bat_inst.mav_buffer[bat_inst.mav_cnt] = cur_volt;
		bat_inst.accum_sum += bat_inst.mav_buffer[bat_inst.mav_cnt]; 
		bat_inst.mav_cnt ++;
		if(bat_inst.mav_cnt >= mav_cnt_set)
       	 	bat_inst.mav_cnt = 0;
		bat_mav_volt = bat_inst.accum_sum/mav_cnt_set;
	}
	return bat_mav_volt*2;
}

static uint32_t bat_pwr_calc(uint32_t up_lim, uint32_t low_lim, uint32_t bat_volt)
{
	uint32_t res_pwr = 0;
	res_pwr = 100*(bat_volt-low_lim)/(up_lim-low_lim);
	if(res_pwr > 100)
		res_pwr = 100;
	return res_pwr;
}

void bat_update(void)
{
	extern sys_reg_st g_sys;
	g_sys.stat.bat.adc_raw = bat_mav_calc(g_sys.conf.bat.mav_cnt);
    //ESP_LOGI(TAG,"bat_volt:%d",g_sys.stat.bat.adc_raw);
	g_sys.stat.bat.pwr_val = bat_pwr_calc(g_sys.conf.bat.up_lim,g_sys.conf.bat.low_lim,g_sys.stat.bat.adc_raw);
}

static void drv_dac_en(uint8_t enable)
{
    if(enable == 1)
        dac_output_enable(DAC_CHANNEL_2);
    else
        dac_output_disable(DAC_CHANNEL_2);
}

static void dac_init(void)
{
	extern sys_reg_st g_sys;
    drv_dac_en(1);
    dac_output_voltage(DAC_CHANNEL_2,g_sys.conf.per.dac_setval);
}

static void bat_register(void);
void bat_init(void) 
{
	uint16_t i;
	for(i=0;i<MAV_MAX_CNT;i++)
	{
		bat_inst.mav_buffer[i] = 0;
	}
	bat_inst.accum_sum = 0;
	bat_inst.mav_cnt = 0;
	adc_init();
	dac_init();
    bat_register();
}

/** Arguments used by 'join' function */
static struct {
    struct arg_int *val;
    struct arg_end *end;
} analog_args;

static int dac_set(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &analog_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, analog_args.end, argv[0]);
        return 1;
    }

    dac_output_voltage(DAC_CHANNEL_2,analog_args.val->ival[0]);

    return 0;
}

static int adc_read(int argc, char **argv)
{
    uint32_t volt;
    volt = adc_get_volt();
    ESP_LOGI(TAG,"Voltage: %dmV", volt);
    return 0;
}

static void register_dac_set()
{
    analog_args.val= arg_int1(NULL, NULL, "<d>", "dac set val");
    analog_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "dac_set",
            .help = "Set dac val",
            .hint = NULL,
            .func = &dac_set,
            .argtable = &analog_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_adc_read()
{
    const esp_console_cmd_t cmd = {
            .command = "adc_read",
            .help = "read local adc data",
            .hint = NULL,
            .func = &adc_read,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static void bat_register(void)
{
    register_dac_set();
    register_adc_read();
}



