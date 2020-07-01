idf_component_register(SRCS "mqtt_tcp.c" "tcp_server.c" 
                    INCLUDE_DIRS "." "../usr" "../kits" 
		    PRIV_REQUIRES nvs_flash mqtt console cmd_nvs
		    )

