//#include "bit_op.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sys_conf.h"
#include "global_var.h"
#include "reg_map_check.h"


#define CONFIG_NAMESPACE 	"config"
//configuration parameters
sys_reg_st  g_sys; 															//global parameter declairation

//configuration register map declairation
const conf_reg_map_st conf_reg_map_inst[CONF_REG_MAP_NUM]=
{//id     mapped registers                 min      max         default  type  chk_prt
    {0,   &g_sys.conf.gen.wifi_mode,       0,       1,          0,       0,    NULL},
    {1,   &g_sys.conf.gen.sample_channel,  0,       2,          2,       0,    NULL},
    {2,   &g_sys.conf.gen.mqtt_start,      0,       1,          0,       0,    NULL},
    {3,   &g_sys.conf.geo.enable,          0,       1,          0,       0,    NULL},
    {4,   &g_sys.conf.geo.pkg_period,      0,       1000000,    10,      0,    NULL},
    {5,   &g_sys.conf.geo.sample_period,   0,       1000,       1,       0,    NULL},
    {6,   &g_sys.conf.geo.filter,          0,       255,        64,      0,    NULL},
    {7,   &g_sys.conf.geo.pkg_en,          0,       1,          1,       0,    NULL},
    {8,   NULL,                            0,       0,          0,       0,    NULL},
    {9,   NULL,                            0,       0,          0,       0,    NULL},
    {10,  &g_sys.conf.eth.http_en,         0,       1,          1,       0,    NULL},
    {11,  &g_sys.conf.eth.tcp_en,          0,       1,          1,       0,    NULL},
    {12,  &g_sys.conf.eth.tcp_period,      1,       0xffffffff, 80,      0,    NULL},
    {13,  &g_sys.conf.fft.acc_times,       1,       128,        1,       0,    NULL},
    {14,  &g_sys.conf.fft.intv_cnts,       1,       1024,       1,       0,    NULL},
    {15,  &g_sys.conf.gtz.n,               32,      65535,      4000,    0,    NULL},
    {16,  &g_sys.conf.gtz.target_freq,     1,       1000,       470,     0,    NULL},
    {17,  &g_sys.conf.gtz.sample_freq,     4000,    4000,       4000,    0,    NULL},
    {18,  &g_sys.conf.gtz.target_span,     0,	    65,         32,      0,    NULL},
    {19,  &g_sys.conf.gtz.acc_q,           2,	    32,         5,       0,    NULL},
    {20,  &g_sys.conf.gtz.reset,           0,       1,          0,       0,    NULL},
    {21,  &g_sys.conf.bat.mav_cnt,         1,	    128,        16,      0,    NULL},
    {22,  &g_sys.conf.bat.up_lim,          3700,    4500,       4150,    0,    NULL},
    {23,  &g_sys.conf.bat.low_lim,         2700,    3500,       3200,    0,    NULL},
    {24,  NULL,                            0,	    0,          0,       0,    NULL},
    {25,  NULL,                            0,	    0,          0,       0,    NULL},
    {26,  NULL,                            0,	    0,          0,       0,    NULL},
    {27,  NULL,                            0,	    0,          0,       0,    NULL},
    {28,  &g_sys.conf.gen.dbg,             0,       1,          0,       0,    NULL},
    {29,  NULL,                            0,	    1,          0,       1,    save_conf_opt},
    {30,  NULL,                            0,	    1,          0,       1,    NULL},
    {31,  &g_sys.conf.gen.restart,         0,	    0xffffffff, 0,       1,    NULL}
};


//status register map declairation
const sts_reg_map_st status_reg_map_inst[STAT_REG_MAP_NUM]=
{//     id      mapped registers
    {	0,      &g_sys.stat.gen.status_reg_num,         STAT_REG_MAP_NUM},
    {	1,      &g_sys.stat.gen.config_reg_num,         CONF_REG_MAP_NUM},
    {	2,      &g_sys.stat.gen.software_ver,           SOFTWARE_VER},
    {	3,      &g_sys.stat.gen.hardware_ver,           HARDWARE_VER},
    {	4,      &g_sys.stat.man.serial_no,              SERIAL_NO},
    {	5,      &g_sys.stat.man.man_date,               MAN_DATE},
    {	6,      &g_sys.stat.man.dev_type,               DEVICE_TYPE},
    {	7,      &g_sys.stat.gen.status_bm,              0},
    {	8,      &g_sys.stat.geo.kfifo_drop_cnt,         0},
    {	9,      NULL,                                   0},
    {	10,     NULL,                                   0},
    {	11,	(void*)&g_sys.stat.gtz.acc_snr,         0},
    {	12,	(void*)&g_sys.stat.gtz.signal_level,    0},
    {	13,	(void*)&g_sys.stat.gtz.noise_level,     0},
    {	14,	&g_sys.stat.gtz.rank,                   0},
    {	15,	&g_sys.stat.gtz.acc_rank,               0},
    {	16,	(void*)&g_sys.stat.gtz.offset,          0},
    {	17,	(void*)&g_sys.stat.gtz.acc_offset,      0},
    {	18,	(void*)&g_sys.stat.gtz.acc_signal_level,0},
    {	19,	(void*)&g_sys.stat.gtz.acc_noise_level, 0},
    {	20,	&g_sys.stat.bat.adc_raw,                0},
    {	21,	&g_sys.stat.bat.pwr_val,                0},
    {	22,	&g_sys.stat.bat.pwr_sts,                0},
    {	23,	NULL,                                   0},
    {	24,	NULL,                                   0},
    {	25,	NULL,                                   0},
    {	26,	NULL,                                   0},
    {	27,	NULL,                                   0},
    {	28,	NULL,                                   0},
    {	29,	NULL,                                   0},
    {	30,	NULL,                                   0},
    {	31,	NULL,                                   0}
};

/**
 * @brief  initialize system status reg data
 * @param  None
 * @retval None
 */
void init_load_status(void)
{
    uint16_t i;
    for (i = 0; i < STAT_REG_MAP_NUM; i++) {
        if (status_reg_map_inst[i].reg_ptr != NULL) {
            *(status_reg_map_inst[i].reg_ptr) = status_reg_map_inst[i].dft;
        }
    }
}


static uint16_t init_load_default(void)
{
    uint16_t i, ret;
    ret = USR_EOK;
    for(i=0;i<CONF_REG_MAP_NUM;i++)		//initialize global variable with default values
    {
        if(conf_reg_map_inst[i].reg_ptr != NULL)
        {
            *(conf_reg_map_inst[i].reg_ptr) = conf_reg_map_inst[i].dft;
        }
    }    
    return ret;
}


/**
 * @brief  write register map with constraints.
 * @param  reg_addr: reg map address.
 * @param  wr_data: write data.
 * @param  permission_flag:
 *   This parameter can be one of the following values:
 *     @arg PERM_PRIVILEGED: write opertion can be performed dispite permission level
 *     @arg PERM_INSPECTION: write operation could only be performed when pass permission check
 * @retval
 *   This parameter can be one of the following values:
 *     @arg 1: write operation success
 *     @arg 0: write operation fail
 */
uint16 reg_map_write(uint16 reg_addr, uint32_t *wr_data, uint8_t wr_cnt)
{
    uint16_t i;
    uint16_t err_code;
    err_code = REGMAP_ERR_NOERR;		

    if((reg_addr+wr_cnt) > CONF_REG_MAP_NUM)	//address range check
    {
        err_code = REGMAP_ERR_ADDR_OR;
        printf("MB_SLAVE REGMAP_ERR_ADDR_OR1 failed\n");
        return err_code;
    }

    for(i=0;i<wr_cnt;i++)										//min_max limit check
    {
        if((*(wr_data+i)>conf_reg_map_inst[reg_addr+i].max)||(*(wr_data+i)<conf_reg_map_inst[reg_addr+i].min))		//min_max limit check
        {
            err_code = REGMAP_ERR_DATA_OR;
            printf("REGMAP_ERR_WR_OR03 failed\n");
            return err_code;	
        }

        if(conf_reg_map_inst[reg_addr+i].chk_ptr != NULL)
        {
            if(conf_reg_map_inst[reg_addr+i].chk_ptr(*(wr_data+i))==0)
            {
                err_code = REGMAP_ERR_CONFLICT_OR;
                printf("CHK_PTR:REGMAP_ERR_WR_OR failed\n");
                return err_code;	
            }
        }
    }

    for(i=0;i<wr_cnt;i++)										//data write
    {
        if(conf_reg_map_inst[reg_addr+i].reg_ptr != NULL)
            *(conf_reg_map_inst[reg_addr+i].reg_ptr) = *(wr_data+i);//write data to designated register
    }	
    return err_code;		
}

/**
 * @brief  read register map.
 * @param  reg_addr: reg map address.
 * @param  *rd_data: read data buffer ptr.
 * @retval
 *   This parameter can be one of the following values:
 *     @arg 1: write operation success
 *     @arg 0: read operation fail
 */
uint16 reg_map_read(uint16 reg_addr,uint32_t* reg_data,uint8_t read_cnt)
{
    uint16_t i;
    uint16_t err_code;
    err_code = REGMAP_ERR_NOERR;
    if((reg_addr&0x8000) != 0)
    {
        reg_addr &= 0x7fff;
        if(reg_addr > STAT_REG_MAP_NUM)	//address out of range
        {
            err_code = REGMAP_ERR_ADDR_OR;
        }
        else
        {
            for(i=0;i<read_cnt;i++)
            {
                *(reg_data+i) = *(status_reg_map_inst[reg_addr+i].reg_ptr);//read data from designated register
            }
        }
    }
    else
    {
        if(reg_addr > CONF_REG_MAP_NUM)	//address out of range
        {
            err_code = REGMAP_ERR_ADDR_OR;
        }
        else
        {
            for(i=0;i<read_cnt;i++)
            {
                if(conf_reg_map_inst[reg_addr+i].reg_ptr == NULL)
                {
                    *(reg_data+i) = 0;
                }
                else
                    *(reg_data+i) = *(conf_reg_map_inst[reg_addr+i].reg_ptr);//read data from designated register
            }
        }		
    }	
    return err_code;
}

int save_conf(const char *save_type)
{
    nvs_handle my_handle;
    esp_err_t err;
    uint32_t* config = NULL;
    uint32_t i;
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS

    // Open
    err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    required_size = CONF_REG_MAP_NUM*sizeof(uint32_t);
    config = malloc(required_size);
    // Write value including previously saved blob if available
    for(i=0;i<CONF_REG_MAP_NUM;i++)
    {
        if(conf_reg_map_inst[i].reg_ptr != NULL)
            config[i] = *(conf_reg_map_inst[i].reg_ptr);
        else
            config[i] = 0xfffffffe;
    }
    err = nvs_set_blob(my_handle, save_type, config, required_size);
    free(config);

    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

int load_conf(const char *load_type)
{
    nvs_handle my_handle;
    esp_err_t err;
    uint32_t* config = NULL;
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS

    err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, load_type, NULL, &required_size);

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    if (required_size == 0) {
        init_load_default();
        printf("Nothing saved as '%s' yet,load default.\n",load_type);

    }
    else if(required_size != CONF_REG_MAP_NUM*sizeof(uint32_t))
    {
        init_load_default();
        err = -1;
        printf("Save data mismatch, '%s',load default.\n",load_type);
    }
    else {
        config = malloc(required_size);
        err = nvs_get_blob(my_handle, load_type, config, &required_size);
        if (err != ESP_OK) {
            free(config);
            return err;
        }
        for (int i = 0; i < required_size / sizeof(uint32_t); i++)
        {
            if(conf_reg_map_inst[i].reg_ptr != NULL)
                *(conf_reg_map_inst[i].reg_ptr) = config[i];
        }
        printf("Load '%s' success.\n",load_type);
        free(config);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}


int32_t gvar_init(void)
{
    initialize_nvs();
    load_conf("usr");
    init_load_status();
    return 0;
}

/** Arguments used by 'blob' function */
static struct {
    struct arg_str *data;
    struct arg_end *end;
} savconf_args;

static int save_config_arg(int argc, char **argv)
{
    esp_err_t err;

    int nerrors = arg_parse(argc, argv, (void**) &savconf_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, savconf_args.end, argv[0]);
        return 1;
    }

    err = save_conf(savconf_args.data->sval[0]);
    return err;
}

static int load_config_arg(int argc, char **argv)
{
    esp_err_t err;

    int nerrors = arg_parse(argc, argv, (void**) &savconf_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, savconf_args.end, argv[0]);
        return 1;
    }

    // Open
    err = load_conf(savconf_args.data->sval[0]);
    return err;
}

static int print_config(int argc, char **argv)
{
    nvs_handle my_handle;
    esp_err_t err;

    int nerrors = arg_parse(argc, argv, (void**) &savconf_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, savconf_args.end, argv[0]);
        return 1;
    }

    // Open
    err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read run time blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, savconf_args.data->sval[0], NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    printf("Config '%s': \n",savconf_args.data->sval[0]);
    if (required_size == 0) {
        printf("Nothing saved yet!\n");
    } else {
        uint32_t* config = malloc(required_size);
        err = nvs_get_blob(my_handle, savconf_args.data->sval[0], config, &required_size);
        if (err != ESP_OK) {
            free(config);
            return err;
        }
        for (int i = 0; i < required_size / sizeof(uint32_t); i++) {
            printf("%d: %x\n", i, config[i]);
        }
        free(config);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

static void register_save_conf()
{
    savconf_args.data = arg_str1(NULL, NULL, "<type>", "save type");
    savconf_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
            .command = "save_conf",
            .help = "save conf regs",
            .hint = NULL,
            .func = &save_config_arg,
            .argtable = &savconf_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_load_conf()
{
    savconf_args.data = arg_str1(NULL, NULL, "<type>", "load type");
    savconf_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
            .command = "load_conf",
            .help = "load saved conf regs",
            .hint = NULL,
            .func = &load_config_arg,
            .argtable = &savconf_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_print_conf()
{
    savconf_args.data = arg_str1(NULL, NULL, "<type>", "save type");
    savconf_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
            .command = "print_conf",
            .help = "print saved conf regs",
            .hint = NULL,
            .func = &print_config,
            .argtable = &savconf_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


/** Arguments used by 'reg map' function */
static struct {
    struct arg_int *addr;
    struct arg_int *data;
    struct arg_end *end;
} regmap_args;

static int rd_reg(int argc, char **argv)
{
    uint32_t rx_buf[32];
    uint8_t i;

    int nerrors = arg_parse(argc, argv, (void**) &regmap_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, regmap_args.end, argv[0]);
        return 1;
    }

    reg_map_read(regmap_args.addr->ival[0],
            rx_buf,
            regmap_args.data->ival[0]);

    i=0;
    for(i=0;i<regmap_args.data->ival[0];i++)
        printf("reg %d: %x\n",(i+regmap_args.addr->ival[0])&0x3fff,rx_buf[i]);
    return 0;
}

static int wr_reg(int argc, char **argv)
{
    uint32_t data = 0;
    int nerrors = arg_parse(argc, argv, (void**) &regmap_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, regmap_args.end, argv[0]);
        return 1;
    }
    data = regmap_args.data->ival[0];
    reg_map_write(regmap_args.addr->ival[0],
            &data,
            1);

    return 0;
}

static void register_rd_reg()
{
    regmap_args.addr = arg_int1(NULL, NULL, "<a>", "reg base addr");
    regmap_args.data = arg_int1(NULL, NULL, "<c>", "read reg count");
    regmap_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "rd_reg",
            .help = "Read reg map",
            .hint = NULL,
            .func = &rd_reg,
            .argtable = &regmap_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_wr_reg()
{
    regmap_args.addr = arg_int1(NULL, NULL, "<a>", "reg base addr");
    regmap_args.data = arg_int1(NULL, NULL, "<d>", "write data");
    regmap_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
            .command = "wr_reg",
            .help = "Write reg map",
            .hint = NULL,
            .func = &wr_reg,
            .argtable = &regmap_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}




void gvar_register(void)
{
    register_rd_reg();
    register_wr_reg();
    register_save_conf();
    register_print_conf();
    register_load_conf();
}


