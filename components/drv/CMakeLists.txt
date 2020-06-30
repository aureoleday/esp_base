idf_component_register(SRCS "adxl_drv.c" "led_drv.c"  
                    INCLUDE_DIRS "." "../kits/" "../cli" "../usr" 
                    PRIV_REQUIRES console kits 
		    )

