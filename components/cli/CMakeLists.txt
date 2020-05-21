idf_component_register(SRCS "cmd_wifi.c"
                            "cli.c"
		    INCLUDE_DIRS  "." "../cmd_nvs" "../cmd_system" "../usr"
		    REQUIRES nvs_flash
		    PRIV_REQUIRES console fatfs)
