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
#include "mqtt_client.h"
#include "global_var.h"

static const char *TAG = "MQTT_EXAMPLE";

static esp_mqtt_client_handle_t lc_client;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
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
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
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
//static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        //.uri = CONFIG_BROKER_URL,
        .uri = "mqtt://192.168.3.84:1883",
    };
    printf("mqtt uri:%s\n",mqtt_cfg.uri);
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    lc_client = client;
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    return 0;
}

void mqtt_send(char* tx_buf, int tx_len)
{
    int msg_id = 0;
    msg_id = esp_mqtt_client_publish(lc_client, "/topic/qos0", tx_buf, tx_len , 0, 0);
    printf("mq_msg_id:%d\n",msg_id);
}

//static struct {
//    struct arg_str *ssid;
//    struct arg_end *end;
//} mqtt_tx_args;
//
//static int save_station_arg(int argc, char **argv)
//{
//    esp_err_t err;
//
//    int nerrors = arg_parse(argc, argv, (void**) &wifi_args);
//    if (nerrors != 0) {
//        arg_print_errors(stderr, wifi_args.end, argv[0]);
//        return 1;
//    }
//
//    err = save_station(wifi_args.ssid->sval[0],wifi_args.pwd->sval[0]);
//    return err;
//}

static int mq_tx(int argc, char **argv)
{
    char buf[]="hello mqtt!";
    mqtt_send(buf,sizeof(buf));
    return 0;
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

static void register_mq_tx()
{
    const esp_console_cmd_t cmd = {
            .command = "mq_tx",
            .help = "send mq sample msg",
            .hint = NULL,
            .func = &mq_tx
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void mqtt_register(void)
{
    register_mq_start();
    register_mq_tx();
}

//void mqtt_thread(void* param)
//{
//    extern sys_reg_st  g_sys; 		//global parameter declairation
//    register_mq_tx();
//    ESP_LOGI(TAG, "[APP] Startup..");
//    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
//    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
//
//    esp_log_level_set("*", ESP_LOG_INFO);
//    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
//    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
//    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
//    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
//    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
//    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
//
//    //ESP_ERROR_CHECK(nvs_flash_init());
//    //ESP_ERROR_CHECK(esp_netif_init());
//    //ESP_ERROR_CHECK(esp_event_loop_create_default());
//
//    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
//     * Read "Establishing Wi-Fi or Ethernet Connection" section in
//     * examples/protocols/README.md for more information about this function.
//     */
//    //ESP_ERROR_CHECK(example_connect());
//
//    while(g_sys.conf.gen.mqtt_start != 1)
//    {
//    	vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }
//    mqtt_app_start();
//    while(1)
//    {
//    	vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }
//}


