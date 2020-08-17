idf_component_register(SRCS "adxl_drv.c" "ads131_drv.c" "io_drv.c" "bat_drv.c" 
                    INCLUDE_DIRS "." "../kits/" "../cli" "../usr" 
                    PRIV_REQUIRES console kits esp_adc_cal 
		    )

