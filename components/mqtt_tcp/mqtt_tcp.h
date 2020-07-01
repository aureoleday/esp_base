void mqtt_register(void);
int mqtt_transmitt(void *tx_buf,int tx_len);
int tcp_transmitt(void *tx_buf,int tx_len);
//int mqtt_transmitt(char *topic, void *tx_buf,int tx_len);
int mqtt_connect(void);
int mqtt_disconnect(void);
void tcp_thread(void *param);
