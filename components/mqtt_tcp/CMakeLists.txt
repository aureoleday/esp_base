idf_component_register(SRCS "mqtt_tcp.c" 
                    INCLUDE_DIRS "." "../usr" 
		    PRIV_REQUIRES nvs_flash mqtt console
		    )

