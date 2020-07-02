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
#include "adxl_drv.h"
#include "led_drv.h"

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
//    xTaskCreate(&test_thread,
//            "Task_test",
//            TEST_THREAD_STACK_SIZE,
//            NULL,
//            TEST_THREAD_PRIO,
//            NULL);

//    xTaskCreate(&tcp_thread,
//            "Task_tcp",
//            TCP_THREAD_STACK_SIZE,
//            NULL,
//            TCP_THREAD_PRIO,
//            NULL);

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
    adxl_init();
    usr_led_init();
    daq_init();
    gvar_register();
    mqtt_register();
    adxl_register();
    tasks_create();
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(g_sys.conf.con.wifi_connect == 1)
    {
        wifi_connect();
        tcp_srv_start();
    }
    while(1)
    {
 	    if(!bit_op_get(g_sys.stat.gen.status_bm,GBM_TCP)&&bit_op_get(g_sys.stat.gen.status_bm,GBM_WIFI) == 1)
	    {
            tcp_srv_start();
	    }
        toggle_led(0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		if(g_sys.conf.gen.restart == 9527)
			esp_restart();
	}
}



