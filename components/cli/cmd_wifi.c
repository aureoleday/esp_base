/* Console example — WiFi commands

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "cmd_decl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mdns.h"
#include "cmd_wifi.h"
#include "sys_conf.h"
#include "bit_op.h"

#define JOIN_TIMEOUT_MS (10000)

static const char *TAG = "WIFI";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

extern sys_reg_st  g_sys; 															


static void start_mdns_service(void)
{
    esp_err_t err;
    char hostname[20];
    int nrd_len;

    //initialize mDNS service
    err = mdns_init();
    if (err) {
        ESP_LOGW(TAG,"MDNS Init failed: %d\n", err);
        return;
    }
    
    err = wget_value_from_nvs("wifi", "mdns_hostname", "str", hostname, &nrd_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,"No mdns hostname saved. please update and try again.\n");
        return;
    }
    //set hostname
    mdns_hostname_set(hostname);
    ESP_LOGI(TAG,"mDNS hostname:%s", hostname); 

    //set default instance
    mdns_instance_name_set("ez-DAQ");

    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_add(NULL, "_mqtt", "_tcp", 1883, NULL, 0);
    mdns_service_add(NULL, "_usr", "_tcp", 9996, NULL, 0);
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if(g_sys.conf.con.wifi_connect == 1)
            esp_wifi_connect();
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_WIFI,0);
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_WIFI,1);
        start_mdns_service();
    }
    else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_WIFI,1);
        start_mdns_service();
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        bit_op_set(&g_sys.stat.gen.status_bm,GBM_WIFI,0);
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void initialise_wifi(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    static bool initialized = false;
    if (initialized) {
        return;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if(g_sys.conf.con.wifi_mode == 1)
    {
        esp_netif_create_default_wifi_ap();
    }
    else
    {
        esp_netif_create_default_wifi_sta();
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
//<<<<<<< HEAD
//    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL));
//
//    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );
//    ESP_ERROR_CHECK( esp_wifi_start() );
//=======
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

//    if(g_sys.conf.con.wifi_mode == 1) {
//        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
//        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
//    }
//    else
//    {
//        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
//        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
//    }
//
//    ESP_ERROR_CHECK( esp_wifi_start() );
//>>>>>>> digital
    initialized = true;
}

bool wifi_join(const char *ssid, const char *pass, int timeout_ms)
{
    initialise_wifi();

    wifi_config_t wifi_config = { 0 };

    if(g_sys.conf.con.wifi_mode == 1)
    {
        //wifi_config = {
        //    .ap = {
        //        .ssid = *ssid,
        //        .ssid_len = strlen(ssid),
        //        //.channel = CONFIG_ESP_WIFI_CHANNEL,
        //        .password = *pass,
        //        .max_connection = 2,
        //        .authmode = WIFI_AUTH_WPA_WPA2_PSK
        //    },
        //};
        //if (strlen(pass) == 0) {
        //    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        //}

        strlcpy((char *) wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
        strlcpy((char *) wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK; 
        wifi_config.ap.max_connection = 3; 

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
                 ssid, pass, wifi_config.ap.channel);
        //start_mdns_service();
    }
    else
    {
        strlcpy((char *) wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        if (pass) {
            strlcpy((char *) wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
        }

        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK( esp_wifi_start());
        ESP_ERROR_CHECK( esp_wifi_connect() );

        int bits = xEventGroupWaitBits(wifi_event_group, 
                                       CONNECTED_BIT,
                                       pdFALSE, 
                                       pdTRUE, 
                                       timeout_ms / portTICK_PERIOD_MS);
        return (bits & CONNECTED_BIT) != 0;
    }
    return 0;
}

/** Arguments used by 'join' function */
static struct {
    struct arg_int *timeout;
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_end *end;
} join_args;

static int connect(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &join_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, join_args.end, argv[0]);
        return 1;
    }
    ESP_LOGI(__func__, "Connecting to '%s'",
             join_args.ssid->sval[0]);

    /* set default value*/
    if (join_args.timeout->count == 0) {
        join_args.timeout->ival[0] = JOIN_TIMEOUT_MS;
    }

    bool connected = wifi_join(join_args.ssid->sval[0],
                               join_args.password->sval[0],
                               join_args.timeout->ival[0]);
    if (!connected) {
        ESP_LOGW(__func__, "Connection timed out");
        return 1;
    }
    ESP_LOGI(__func__, "Connected");
    return 0;
}

int wifi_connect(void)
{
    esp_err_t err;
    char ssid[32];
    char pwd[32];
    int nrd_len;
    if(g_sys.conf.con.wifi_mode == 0)
    {
        err = wget_value_from_nvs("wifi", "remote_ssid", "str", ssid, &nrd_len);
        if (err != ESP_OK) {
            ESP_LOGD(TAG,"No remote ssid saved. please update and try again.\n");
            return 0;
        }
        err = wget_value_from_nvs("wifi", "remote_pwd", "str", pwd, &nrd_len);
        if (err != ESP_OK) {
            ESP_LOGD(TAG,"No remote pwd saved. please update and try again.\n");
            return 0;
        }
    }
    else
    {
        err = wget_value_from_nvs("wifi", "local_ssid", "str", ssid, &nrd_len);
        if (err != ESP_OK) {
            ESP_LOGD(TAG,"No local ssid saved. please update and try again.\n");
            return 0;
        }
        err = wget_value_from_nvs("wifi", "local_pwd", "str", pwd, &nrd_len);
        if (err != ESP_OK) {
            ESP_LOGD(TAG,"No local pwd saved. please update and try again.\n");
            return 0;
        }
    }
    wifi_join(ssid, pwd, JOIN_TIMEOUT_MS);
    return 1;
}

int wifi_disconnect(void)
{
    esp_wifi_disconnect();
    return 1;
}

void register_wifi(void)
{
    join_args.timeout = arg_int0(NULL, "timeout", "<t>", "Connection timeout, ms");
    join_args.ssid = arg_str1(NULL, NULL, "<ssid>", "SSID of AP");
    join_args.password = arg_str0(NULL, NULL, "<pass>", "PSK of AP");
    join_args.end = arg_end(2);

    const esp_console_cmd_t join_cmd = {
        .command = "join",
        .help = "Join WiFi AP as a station",
        .hint = NULL,
        .func = &connect,
        .argtable = &join_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&join_cmd) );
}
