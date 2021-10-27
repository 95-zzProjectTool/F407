/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 使用串口通信实现发送数据到电脑以及接收电脑端发送的数据
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart/bsp_usartx.h"
#include "string.h"
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "key/bsp_key.h"
#include "usmart.h"
#include "stdio.h"
#include "delay.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
#define  FASTSEEK_SPEED   300		//原点回归速度
#define  SLOWSEEK_SPEED   100		//原点回归爬行速度
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
// 速度最大值由驱动器和电机决定，有些最大是1800，有些可以达到4000
uint32_t set_speed  = 500;         // 速度 单位为0.05rad/sec
// 加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
// 所以加速度和减速度值一般是在实际应用中多尝试出来的结果
uint32_t step_accel = 150;         // 加速度 单位为0.025rad/sec^2
uint32_t step_decel = 50;         // 减速度 单位为0.025rad/sec^2
//uint8_t aRxBuffer;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
 // uint8_t txbuf[50];
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
		delay_init(168);               	//初始化延时函数
  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化串口并配置串口中断优先级 */
  MX_USARTx_Init();
//	KEY_GPIO_Init();
//  /* 配置定时器输出脉冲 */
//  STEPMOTOR_TIMx_Init();
	 usmart_dev.init(168); 
  printf("通信测试！\r\n");
//  memcpy(txbuf,"这是一个串口中断接收回显实验\n",50);
//  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
//  
//  memcpy(txbuf,"输入数据并以回车键结束\n",50);
//  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
//  
  /* 使能接收，进入中断回调函数 */
  //HAL_UART_Receive_IT(&husartx,&aRxBuffer,1);
  
  /* 无限循环 */
  while (1)
  {
				HAL_Delay(1000);
//		 HAL_UART_Transmit(&huart1, (uint8_t *)ZZX,3, 0xffff);
	printf("printf可以使用\r\n");
//    if(KEY1_StateRead() == KEY_DOWN)
//		{
//      STEPMOTOR_DisMoveRel(-100,step_accel,step_decel,set_speed);//向前移动100mm
//		}
//    if(KEY2_StateRead() == KEY_DOWN)
//		{
//      STEPMOTOR_DisMoveRel(100,step_accel,step_decel,set_speed);//向后移动100mm
//		}
  }
}

/**
  * 函数功能: 串口接收完成回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
