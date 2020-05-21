idf_component_register(SRCS "cmd_wifi.c"
                            "cli.c"
                    INCLUDE_DIRS "." "../cmd_system" "../cmd_nvs" "../usr"
		    PRIV_REQUIRES console fatfs nvs_flash)
