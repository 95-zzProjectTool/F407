#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  	// 电机旋转状态
  __IO int8_t   dir ;        	// 电机旋转方向
  __IO uint32_t decel_start; 	// 启动减速位置
  __IO int32_t  step_delay;  	// 下个脉冲周期（时间间隔），启动时为加速度
  __IO int32_t  decel_val;   	// 减速阶段步数
  __IO int32_t  min_delay;   	// 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  accel_count; 	// 加减速阶段计数值
  __IO int32_t  medle_delay;
}speedRampData;

/* 宏定义 --------------------------------------------------------------------*/
#define STEPMOTOR_TIMx                        TIM8
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIM_IT_CCx                  TIM_IT_CC1
#define STEPMOTOR_TIM_FLAG_CCx                TIM_FLAG_CC1
#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_IRQHandler             TIM8_CC_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_x               TIM_CHANNEL_1
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出控制脉冲给电机驱动器
#define STEPMOTOR_TIM_PUL_PORT                GPIOI                            // 对应驱动器的PUL-（驱动器使用共阳接法）
#define STEPMOTOR_TIM_PUL_PIN                 GPIO_PIN_5                       // 而PLU+直接接开发板的VCC

#define STEPMOTOR_DIR_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR_PORT                    GPIOD                            // 对应驱动器的DIR-（驱动器使用共阳接法）
#define STEPMOTOR_DIR_PIN                     GPIO_PIN_3                       // 而DIR+直接接开发板的VCC
#define GPIO_PIN_AF_AS_SYS                    GPIO_AF0_RTC_50Hz                // 引脚不作为复用功能使用

#define STEPMOTOR_ENA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA_PORT                    GPIOD                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define STEPMOTOR_ENA_PIN                     GPIO_PIN_7                       // 而ENA+直接接开发板的VCC

#define STEPMOTOR_DIR_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT,STEPMOTOR_DIR_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT,STEPMOTOR_ENA_PIN,GPIO_PIN_SET)

  

#define ORIGIN_GPIO_CLK_ENABLE()        	  	__HAL_RCC_GPIOG_CLK_ENABLE()     // 原点检测输入引脚
#define ORIGIN_PORT                       		GPIOG                            // 
#define ORIGIN_PIN                      		  GPIO_PIN_0  
#define ORIGIN_EXTI_IRQn			    						EXTI0_IRQn
#define ORIGIN_EXTI_IRQHandler                EXTI0_IRQHandler 

#define LIMPOS_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // 正转限位输入引脚
#define LIMPOS_PORT                   				GPIOG                            // 有效电平是低电平,与GND短接即可
#define LIMPOS_PIN                    				GPIO_PIN_1                       // 
#define LIMPOS_EXTI_IRQn											EXTI1_IRQn
#define LIMPOS_EXTI_IRQHandler                EXTI1_IRQHandler                 // 中断服务函数入口

#define LIMNEG_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // 反转限位输入引脚
#define LIMNEG_PORT                   				GPIOG                            // 有效电平是低电平,与GND短接即可
#define LIMNEG_PIN                    				GPIO_PIN_2                       // 
#define LIMNEG_EXTI_IRQn											EXTI2_IRQn
#define LIMNEG_EXTI_IRQHandler                EXTI2_IRQHandler                 // 中断服务函数入口


  
// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER               7  // 步进电机驱动器细分设置为：   32  细分
//#define STEPMOTOR_TIM_PRESCALER               15  // 步进电机驱动器细分设置为：   16  细分
//#define STEPMOTOR_TIM_PRESCALER               31  // 步进电机驱动器细分设置为：   8  细分
//#define STEPMOTOR_TIM_PRESCALER               63  // 步进电机驱动器细分设置为：   4  细分
//#define STEPMOTOR_TIM_PRESCALER               127  // 步进电机驱动器细分设置为：   2  细分
//#define STEPMOTOR_TIM_PRESCALER               255 // 步进电机驱动器细分设置为：   1  细分


// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// 定义高级定时器重复计数寄存器值
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    1  // 顺时针:正方向
#define CCW                                   -1 // 逆时针:反方向
#define LIM_POS_LEVEL						              0	 // 正极限引脚有效电平
#define LIM_NEG_LEVEL						              0	 // 负极限引脚有效电平
#define ORIGIN_LEVEL                          0
#define HOME_POSITION	                        0  // 原点坐标
#define STOP                                  0  // 加减速曲线状态：停止
#define ACCEL                                 1  // 加减速曲线状态：加速阶段
#define DECEL                                 2  // 加减速曲线状态：减速阶段
#define RUN                                   3  // 加减速曲线状态：匀速阶段
#define IDLE	   						                  0	 // 搜索原点状态:空闲
#define FASTSEEK   							              1  // 搜索原点状态:快速搜索
#define SLOWSEEK 							                2  // 搜索原点状态:慢速搜索
#define MOVETOZERO 							              3  // 搜索原点状态:捕获原点
#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // 频率ft值
#define FSPR                                  200// 步进电机单圈步数  步距角:1.8° 360/1.8 = 200 正常情况下需要200步转一圈
#define MICRO_STEP                            32 // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数

// 数学常数
#define ALPHA                                 ((float)(2*3.14159/SPR))       // α= 2*pi/spr步距角
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10))  // 0.676为误差修正值
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))
#define MAX_NUM_LAP 						              INT32_MAX
#define MAX_NUM_STEP 						              UINT32_MAX

#define UNIT_STEP_MM                          (SPR/UNITS_DISTANCE)//步进1mm需要的步数
#define MAX_STEP_MM                           (MAX_DISTANCE/UNITS_DISTANCE)*UNIT_STEP_MM //

#define UNITS_DISTANCE                        5   // 步进电机转一圈,导轨前进5mm
#define MAX_DISTANCE                          400 // 导轨可以移动的最长距离400mm
/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
/* 函数声明 ------------------------------------------------------------------*/
extern __IO int8_t   HomeDir;			  // 电机回原点方向
extern __IO uint8_t  HomeCapture ;		  // 原点捕获标志


void STEPMOTOR_TIMx_Init(void);
void STEPMOTOR_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_AxisMoveAbs(int32_t targert_step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_AxisHome(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_DisMoveRel(int16_t distance, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_DisMoveAbs(uint16_t Target_Dis, uint32_t accel, uint32_t decel, uint32_t speed);
#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
