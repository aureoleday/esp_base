/*
 * tcp_thread.c
 *
 *  Created on: 2019Äê1ÔÂ15ÈÕ
 *      Author: Administrator
 */
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "sys_conf.h"
#include "bit_op.h"
#include "kfifo.h"

#define PORT 9996
#define BUFSZ 1024

static const char *TAG = "tTCP";

static uint32_t tx_buf[2048];
static uint8_t  rx_buf[BUFSZ];

extern  sys_reg_st  g_sys;

static uint8_t  tcp_flag;

static void tcp_rx_thread(void* parameter);

static int sock;

void tcp_thread(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    xTaskHandle rx_xHandle;
    struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
    uint addrLen = sizeof(sourceAddr);

#ifdef CONFIG_EXAMPLE_IPV4
	struct sockaddr_in destAddr;
	destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	destAddr.sin_family = AF_INET;
	destAddr.sin_port = htons(PORT);
	addr_family = AF_INET;
	ip_protocol = IPPROTO_IP;
	inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
	struct sockaddr_in6 destAddr;
	bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
	destAddr.sin6_family = AF_INET6;
	destAddr.sin6_port = htons(PORT);
	addr_family = AF_INET6;
	ip_protocol = IPPROTO_IPV6;
	inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

	int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
	if (listen_sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
	   // break;
	}
	ESP_LOGI(TAG, "Socket created");

	int opt = 1;
	 if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(int))<0)
	 {
		 perror("setsockopt");
		 //exit(EXIT_FAILURE);
	 }

	int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
	if (err != 0) {
		ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
	}
	ESP_LOGI(TAG, "Socket binded");

	err = listen(listen_sock, 1);
	if (err != 0) {
		ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
	}
	ESP_LOGI(TAG, "Socket listening");


	while (1)
	{
		addrLen = sizeof(struct sockaddr_in6);
		sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
		if (sock < 0) {
			ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
			bit_op_set(&g_sys.stat.gen.status_bm,GBM_TCP,0);
			continue;
		}
		else
		{
			ESP_LOGI(TAG, "Socket accepted");
			bit_op_set(&g_sys.stat.gen.status_bm,GBM_TCP,1);

            while(1)
            {
		        bytes_received = recv(r_sock, (char*)rx_buf, BUFSZ - 1, 0);
		        if (bytes_received < 0)
		        {
		        	printf("\nsock err %d.\r\n",bytes_received);
		        	  vTaskDelay(1000000 / portTICK_PERIOD_MS);
		        }
		        else if (bytes_received == 0)
		        {
		        	printf("\nTcp connection close 0.\r\n");
		        	vTaskDelay(1000000 / portTICK_PERIOD_MS);
		        }
		        else
		        {
		        	for(i=0;i<(bytes_received>>2);i++)
		        	{
		        		host_data = ntohl(*(uint32_t *)(rx_buf+4*i));
                        cmd_stream_in(host_data, bytes_received);
		        	}
		        }
            }
			close(sock);
			bit_op_set(&g_sys.stat.gen.status_bm,GBM_TCP,0);
		}
	}
	vTaskDelete(NULL);
}

int tcp_transmitt(void *tx_buf,int tx_len)
{
    int ret;
    struct tcp_info info;
    int len = sizeof(info);

    getsockopt(sock,IPPROTO_TCP TCP_INFO, &info, (socklen_t *)&len);
    if(info.tcpi_state == TCP_ESTABLISHED)
        ret = send(sock, tx_buf, tx_len, 0);
    else
        ret = -1;
    return ret;
}


static void tcp_rx_thread(void* parameter)
{
    int r_sock,i;
    int bytes_received;
    uint32_t host_data;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    r_sock = *(int*)parameter;

	while(1)
	{
		bytes_received = recv(r_sock, (char*)rx_buf, BUFSZ - 1, 0);
		if (bytes_received < 0)
		{
			printf("\nsock err %d.\r\n",bytes_received);
			  tcp_flag |= 1;
			  vTaskDelay(1000000 / portTICK_PERIOD_MS);
		}
		else if (bytes_received == 0)
		{
			printf("\nTcp connection close 0.\r\n");
			tcp_flag |= 1;
			vTaskDelay(1000000 / portTICK_PERIOD_MS);
		}
		else
		{
			for(i=0;i<(bytes_received>>2);i++)
			{
				host_data = ntohl(*(uint32_t *)(rx_buf+4*i));
                cmd_stream_in(host_data, bytes_received);
			}
		}
		vTaskDelay(20 / portTICK_PERIOD_MS);
	}
}

