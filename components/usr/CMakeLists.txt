idf_component_register(SRCS "global_var.c" "reg_map_check.c" "cmd_resolve.c"
                    INCLUDE_DIRS "." "../kits/" 
		    REQUIRES console nvs_flash 
		    #PRIV_REQUIRES bit_op

                    )
