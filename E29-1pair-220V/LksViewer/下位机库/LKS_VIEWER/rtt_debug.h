#include <stdio.h>

/*用于控制输出调试信息*/
#define PRINTF_DEBUG    1U
#define USE_RTT_PRINTF  1U

#if (PRINTF_DEBUG > 0U)
  #if (USE_RTT_PRINTF > 0U)
    #include "SEGGER_RTT.h"
    /*基于 SEGGER_RTT 不能输出浮点，中文，等一些特殊字符(优点速度快可在中断中调用)*/
    #define   PRINTF_Dbg(...)     \
                    do{\
                        SEGGER_RTT_printf(0,"file:%s,line:%d,",__FILE__,__LINE__);\
                        SEGGER_RTT_printf(0,__VA_ARGS__);\
                      }while(0);
 
    #define   PRINTF(...)     \
                      SEGGER_RTT_printf(0,__VA_ARGS__);
  #else
    /*使用串口格式化到 printf 输出*/
    #define  	PRINTF_Dbg(...)     \
                    do{\
                        printf("file:%s,line:%d,",__FILE__,__LINE__);\
                        printf(__VA_ARGS__);\
                      }while(0);
                      
    #define  	PRINTF(...)     \
                      printf(__VA_ARGS__);
  #endif
                    
#else
  #define 	PRINTF_Dbg(...)
  #define  	PRINTF(...)
#endif
