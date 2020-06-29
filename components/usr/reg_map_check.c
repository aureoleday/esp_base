#include "sys_def.h" 
#include "reg_map_check.h"
#include "global_var.h"
#include "mqtt_tcp.h"
#include "cmd_wifi.h"
#include "daq.h"



extern sys_reg_st g_sys;
//
uint16_t dhcp_trigger(uint32_t pram)
{  
    //    usr_dhcp_action(pram);
    return 1;
}

uint16_t daq_pkg_en(uint32_t pram)
{  
    if(pram == 1)
       daq_tim_start(g_sys.conf.daq.pkg_period);
    else
       daq_tim_stop();
    return 1;
}

uint16_t set_mqtt_con_opt(uint32_t pram)
{
    if(pram == 1)
    {    
        mqtt_connect();
    }
    else
    {
        mqtt_disconnect();
    }
    return 1;
}

uint16_t set_wifi_con_opt(uint32_t pram)
{
    if(pram == 1)
    {
        wifi_connect();
    }
    else
    {
        wifi_disconnect();
    }
    return 1;
    
}

uint16_t set_boot_opt(uint32_t pram)
{  
    //    set_startup_flag((uint8_t)pram);
    //    rt_kprintf("set boot opt: %d\n", pram);
    return 1;
}

uint16_t save_conf_opt(uint32_t pram)
{  
    if(pram == 1)
    {
        save_conf("usr");
    }
    return 1;
}

uint16_t load_conf_opt(uint32_t pram)
{
    if(pram == 1)
    {
        load_conf("x");
        save_conf("usr");
    }
    return 1;
}

uint16_t set_timestamp(uint32_t pram)
{
    //    rt_device_t device;
    //    rt_err_t ret = -RT_ERROR;
    //
    //    device = rt_device_find("rtc");
    //    /* update to RTC device. */
    //    ret = rt_device_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, &pram);
    //
    //    if(ret == RT_EOK)
    //        return 1;
    //    else
    return 0;
}


uint16_t tcp_timer_opt(uint32_t pram)
{
    //    extern rt_timer_t tm_tcp_repo;
    //    uint32_t period;
    //    period = pram*RT_TICK_PER_SECOND;
    //    rt_timer_control(tm_tcp_repo,RT_TIMER_CTRL_SET_TIME,(void*)&period);
    return 1;
}


uint16_t mod_volum_opt(uint32_t pram)
{
    //    uint8_t data = pram&0x0ff;
    //    uint8_t i;
    //
    //    switch (data)
    //    {
    //    	case 0:
    //    	{
    //    		for(i=0;i<4;i++)
    //    		{
    //    			mod_volum_mdf(0);
    //    			vTaskDelay(5 / portTICK_PERIOD_MS);
    //    		}
    //    		break;
    //    	}
    //    	case 1:
    //    	{
    //   			mod_volum_mdf(0);
    //    		break;
    //    	}
    //    	case 2:
    //    	{
    //   			mod_volum_mdf(1);
    //    		break;
    //    	}
    //    	case 3:
    //    	{
    //    		for(i=0;i<4;i++)
    //    		{
    //    			mod_volum_mdf(1);
    //    			vTaskDelay(5 / portTICK_PERIOD_MS);
    //    		}
    //    		break;
    //    	}
    //    	default:
    //    	{
    //    		for(i=0;i<4;i++)
    //    		{
    //    			mod_volum_mdf(0);
    //    			vTaskDelay(5 / portTICK_PERIOD_MS);
    //    		}
    //    		break;
    //    	}
    //    }

    return 1;
}

uint16_t mod_freq_opt(uint32_t pram)
{
    //    uint8_t data = pram&0x0ff;
    //    uint8_t i;
    //
    //    switch (data)
    //    {
    //    	case 0:
    //    	{
    //    		for(i=0;i<4;i++)
    //    		{
    //    			mod_freq_mdf(0);
    //    			vTaskDelay(5 / portTICK_PERIOD_MS);
    //    		}
    //    		break;
    //    	}
    //    	case 1:
    //    	{
    //    		mod_freq_mdf(0);
    //    		break;
    //    	}
    //    	case 2:
    //    	{
    //    		mod_freq_mdf(1);
    //    		break;
    //    	}
    //    	case 3:
    //    	{
    //    		for(i=0;i<4;i++)
    //    		{
    //    			mod_freq_mdf(1);
    //    			vTaskDelay(5 / portTICK_PERIOD_MS);
    //    		}
    //    		break;
    //    	}
    //    	default:
    //    	{
    //    		for(i=0;i<4;i++)
    //    		{
    //    			mod_freq_mdf(0);
    //    			vTaskDelay(5 / portTICK_PERIOD_MS);
    //    		}
    //    		break;
    //    	}
    //    }

    return 1;
}

