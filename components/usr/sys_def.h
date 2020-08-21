#ifndef	__SYS_DEF
#define	__SYS_DEF
// Define   NULL   pointer   
#ifndef   NULL 
#   ifdef   __cplusplus 
#     define   NULL      0 
#   else 
#     define   NULL      ((void   *)0) 
#   endif 
#endif //   NULL 

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned char 		uint8;
typedef char 				int8;
typedef unsigned short 		uint16;
typedef short				int16;
typedef unsigned long 		uint32;
typedef long				int32;


#define USR_EOK                 0
#define USR_ERR                 (-1)

#define DEFAULT_SDN             "www.baidu.com"

//#define SYS_DEBUG
#define STAT_REG_MAP_NUM        64 
#define CONF_REG_MAP_NUM        64 
#define SOFTWARE_VER            0x00020002
#define HARDWARE_VER            0x00020000
#define SERIAL_NO               0
#define MAN_DATE                0
#define DEVICE_TYPE             0x00000001

#define MBM_TOTAL_SLAVE_NUM     32
#define MBM_HOLDING_REGS_NUM    32

enum
{
    REGMAP_ERR_NOERR = 0,
    REGMAP_ERR_ADDR_OR,
    REGMAP_ERR_DATA_OR,
    REGMAP_ERR_PERM_OR,
    REGMAP_ERR_WR_OR,
    REGMAP_ERR_CONFLICT_OR
};

enum
{    
    INIT_MODE_USR = 0,    
    INIT_MODE_FACTORY,
    INIT_MODE_DEBUT,
    INIT_MODE_DEFAULT
};

enum
{    
    SENSOR_NONE = 0,    
    SENSOR_MULTI,
    SENSOR_ADDR2,
    SENSOR_BT,
    SENSOR_PH,
    SENSOR_DO2
};

enum
{    
    GBM_LINK = 0,
    GBM_BT,
    GBM_WIFI,
    GBM_DHCP,
    GBM_DNS,
    GBM_TCP,
    GBM_MQTT,
    GBM_HTTP,
    GBM_DAQ,
    GBM_GEO,
};

enum
{    
    SERVICE_TCP = 0,    
    SERVICE_MQTT,
    SERVICE_HTTP,
};

enum
{    
    PWR_MODE_IDLE = 0,    
    PWR_MODE_PRON,
    PWR_MODE_ON,
    PWR_MODE_ONGAP,
    PWR_MODE_PROFF,
    PWR_MODE_OFF,
};
#endif //__SYS_DEF
