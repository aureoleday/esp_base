/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "cmd_nvs.h"
#include "mqtt_client.h"
#include "global_var.h"
#include "bit_op.h"
#include "cmd_resolve.h"

#define DEFAULT_URI "mqtt://127.0.0.1:1883"
#define DEFAULT_PUB_TOPIC "/clt/data"
#define DEFAULT_SUB_TOPIC "/srv/data"
#define MQ_NVS_STR_SIZE 32

typedef struct
{
    char broker_uri[MQ_NVS_STR_SIZE];
    char pub_topic[MQ_NVS_STR_SIZE];
    char sub_topic[MQ_NVS_STR_SIZE];
}mq_nvs_st;

static mq_nvs_st mq_nvs_inst;

static const char *TAG = "MQTT";

static esp_mqtt_client_handle_t lc_client;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, mq_nvs_inst.sub_topic, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, mq_nvs_inst.pub_topic, "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG,"TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG,"DATA=%.*s", event->data_len, event->data);
            cmd_stream_in(event->data, event->data_len);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static int mqtt_start(int argc, char **argv)
{
    esp_err_t err;
    int nrd_len;
    extern sys_reg_st  g_sys;
    if(bit_op_get(g_sys.stat.gen.status_bm,GBM_WIFI) == 0)
    {
        ESP_LOGI(TAG,"WIFI not connected, unable to start MQTT protocal.\n");
        return 0;
    }
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    err = wget_value_from_nvs("mqtt", "broker_uri", "str", mq_nvs_inst.broker_uri, &nrd_len);
    if (err != ESP_OK) {
        ESP_LOGD(TAG,"No mqtt broker uri saved, load default.\n");
        strcpy(mq_nvs_inst.broker_uri,DEFAULT_URI);
    }

    err = wget_value_from_nvs("mqtt", "pub_topic", "str", mq_nvs_inst.pub_topic, &nrd_len);
    if (err != ESP_OK) {
        ESP_LOGD(TAG,"No mqtt broker pub topic saved, load default.\n");
        strcpy(mq_nvs_inst.pub_topic,DEFAULT_PUB_TOPIC);
    }

    err = wget_value_from_nvs("mqtt", "sub_topic", "str", mq_nvs_inst.sub_topic, &nrd_len);
    if (err != ESP_OK) {
        ESP_LOGD(TAG,"No mqtt broker sub topic saved, load default.\n");
        strcpy(mq_nvs_inst.sub_topic,DEFAULT_SUB_TOPIC);
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        //.uri = CONFIG_BROKER_URL,
        .uri = mq_nvs_inst.broker_uri,
        .buffer_size = 4096
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    lc_client = client;
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    return 0;
}

int mqtt_connect(void)
{
    mqtt_start(0,NULL);
    return 0;
}

int mqtt_disconnect(void)
{
    esp_mqtt_client_disconnect(lc_client);
    return 0;
}

static struct {
    struct arg_str *topic;
    struct arg_str *data;
    struct arg_int *iteration;
    struct arg_int *repeats;
    struct arg_end *end;
} mqtt_tx_args;

//int mqtt_transmitt(char *topic, void *tx_buf,int tx_len)
//{
//    int msg_id = 0;
//    msg_id = esp_mqtt_client_publish(lc_client, topic, tx_buf, tx_len, 0, 0);
//    return msg_id; 
//}

int mqtt_transmitt(void *tx_buf,int tx_len)
{
    int msg_id = 0;
    msg_id = esp_mqtt_client_publish(lc_client, mq_nvs_inst.pub_topic, tx_buf, tx_len, 0, 0);
    return msg_id; 
}

static int mqtt_rsend(int argc, char **argv)
{
    int msg_id = 0;
    int str_len = 0, tx_len = 0;
    int i = 0;

    int nerrors = arg_parse(argc, argv, (void**) &mqtt_tx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, mqtt_tx_args.end, argv[0]);
        return 1;
    }
    str_len = strlen(mqtt_tx_args.data->sval[0]);
    char *temp = malloc(str_len*mqtt_tx_args.iteration->ival[0]);
    for(i=0;i<mqtt_tx_args.iteration->ival[0];i++)
    {
       memcpy(temp+i*str_len, mqtt_tx_args.data->sval[0],str_len);
    }
       tx_len += i*str_len;

    for(int i=0;i<mqtt_tx_args.repeats->ival[0];i++)
    {
        msg_id = esp_mqtt_client_publish(lc_client, mqtt_tx_args.topic->sval[0], temp, tx_len, 0, 0);
    }

    free(temp);
    return msg_id; 
}


static void register_mq_start()
{
    const esp_console_cmd_t cmd = {
            .command = "mq_start",
            .help = "start mqtt client",
            .hint = NULL,
            .func = &mqtt_start
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_mq_rtx()
{
    mqtt_tx_args.topic = arg_str1(NULL, NULL, "<topic>", "topic");
    mqtt_tx_args.data = arg_str1("d", "data", "<data>", "topic data");
    mqtt_tx_args.iteration = arg_int1(NULL,NULL, "<iteration>", "data iteration");
    mqtt_tx_args.repeats = arg_int1(NULL, NULL, "<repeats>", "transmission repeats");
    mqtt_tx_args.end = arg_end(4);
    
    const esp_console_cmd_t cmd = {
            .command = "mq_rtx",
            .help = "send mq sample msg",
            .hint = NULL,
            .func = &mqtt_rsend,
            .argtable = &mqtt_tx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void mqtt_register(void)
{
    register_mq_start();
    register_mq_rtx();
}


