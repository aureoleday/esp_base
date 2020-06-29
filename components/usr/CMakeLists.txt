idf_component_register(SRCS "global_var.c" "reg_map_check.c" "cmd_resolve.c" "daq.c" 
                    INCLUDE_DIRS "." "../kits/" "../cli" "../cmd_nvs" "../mqtt_tcp"
		    REQUIRES console nvs_flash 
		    PRIV_REQUIRES kits
		    )

