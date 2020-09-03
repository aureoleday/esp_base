/* Console example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cli.h"
#include "cmd_resolve.h"
#include "global_var.h"
#include "bit_op.h"
#include "mqtt_tcp.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "cmd_wifi.h"
#include "daq.h"
#include "ads131_drv.h"
#include "io_drv.h"
#include "bat_drv.h"
#include "adxl_drv.h"
#include "reg_map_check.h"

enum
{
    TEST_THREAD_PRIO=3,
    CLI_THREAD_PRIO,
    CMD_THREAD_PRIO,
    TCP_THREAD_PRIO,
    SPP_THREAD_PRIO,
    USR_MAX_PRIO
};

#define TEST_THREAD_STACK_SIZE 	3072
#define TCP_THREAD_STACK_SIZE 	4096
#define CLI_THREAD_STACK_SIZE 	4096
#define CMD_THREAD_STACK_SIZE 	3072

static void tasks_create(void)
{
    xTaskCreate(&cli_thread,
            "Task_cli",
            CLI_THREAD_STACK_SIZE,
            NULL,
            CLI_THREAD_PRIO,
            NULL);

    xTaskCreate(&cmd_thread,
            "Task_CMD",
            CMD_THREAD_STACK_SIZE,
            NULL,
            CMD_THREAD_PRIO,
            NULL);
}

void app_main()
{
    extern sys_reg_st  g_sys;
    gvar_init();
//    adxl_init();
    adc_init();
    io_init();
    daq_init();
    bat_init();
    gvar_register();
    mqtt_register();
    tasks_create();
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(g_sys.conf.con.wifi_connect == 1)
    {
        wifi_connect();
        service_opt(g_sys.conf.prt.service_bm);
    }
    while(1)
    {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
        bat_update();
		if(g_sys.conf.gen.restart == 9527)
			esp_restart();
	}
}



