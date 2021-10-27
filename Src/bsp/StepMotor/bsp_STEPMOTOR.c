/**
  ******************************************************************************
  * 文件名程: bsp_STEPMOTOR.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-06-05
  * 功    能: 步进电机驱动器控制实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "StepMotor/bsp_STEPMOTOR.h" 
#include <math.h>
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
speedRampData srd               = {STOP,CW,0,0,0,0,0,0};         // 加减速曲线变量

__IO uint8_t  ZeroStep = IDLE;	//搜索原点状态机
__IO uint8_t  LimPosi = FALSE ; //正方向极限标志位  True:到达极限位  False:未到达极限位
__IO uint8_t  LimNega = FALSE ; //负方向极限标志位
__IO uint8_t  DOG         		  = FALSE;		  // 近点信号
__IO uint8_t  HomeCapture       = FALSE;		  // 原点捕获标志
__IO int8_t   HomeDir           = CCW;			  // 电机回原点方向
__IO int32_t  step_position     = 0;          // 当前位置   单位:脉冲数
__IO int16_t  location          = 0;          //当前位置    单位:毫米(mm)
__IO uint8_t  Motionstatus      = 0;           //电机状态  0:停止  1:运动

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 驱动器相关GPIO初始化配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void STEPMOTOR_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* 引脚端口时钟使能 */
  STEPMOTOR_TIM_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA_GPIO_CLK_ENABLE();
  
  LIMPOS_GPIO_CLK_ENABLE();
  LIMNEG_GPIO_CLK_ENABLE();
  
  /* 驱动器脉冲控制引脚IO初始化 */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;        // GPIO引脚用做TIM复用功能
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL_PORT, &GPIO_InitStruct);
  
  /* 驱动器方向控制引脚IO初始化 */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO引脚用做系统默认功能
  HAL_GPIO_Init(STEPMOTOR_DIR_PORT, &GPIO_InitStruct);
  
  /* 驱动器脱机使能控制引脚IO初始化 */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO引脚用做系统默认功能
  HAL_GPIO_Init(STEPMOTOR_ENA_PORT, &GPIO_InitStruct);
  
  STEPMOTOR_DIR_FORWARD();
  STEPMOTOR_OUTPUT_DISABLE();
  
  /* 步进电机正极限输入*/
  GPIO_InitStruct.Pin = LIMPOS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO引脚用做系统默认功能
  HAL_GPIO_Init(LIMPOS_PORT, &GPIO_InitStruct);
	
	/* 步进电机负极限输入*/
  GPIO_InitStruct.Pin = LIMNEG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO引脚用做系统默认功能
  HAL_GPIO_Init(LIMNEG_PORT, &GPIO_InitStruct);
	/* 步进电机原点检测输入*/
  GPIO_InitStruct.Pin = ORIGIN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO引脚用做系统默认功能
	HAL_GPIO_Init(ORIGIN_PORT, &GPIO_InitStruct);
  
	HAL_NVIC_SetPriority(LIMPOS_EXTI_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(LIMPOS_EXTI_IRQn);
  
  HAL_NVIC_SetPriority(LIMNEG_EXTI_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(LIMNEG_EXTI_IRQn);
  
  HAL_NVIC_SetPriority(ORIGIN_EXTI_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(ORIGIN_EXTI_IRQn);
}

/**
  * 函数功能: 驱动器定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void STEPMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // 定时器时钟
  TIM_OC_InitTypeDef sConfigOC;                          // 定时器通道比较输出
  
  STEPMOTOR_TIM_RCC_CLK_ENABLE();
  
  /* STEPMOTOR相关GPIO初始化配置 */
  STEPMOTOR_GPIO_Init();
  
  /* 定时器基本环境配置 */
  htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                          // 定时器编号
  htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;           // 定时器预分频器
  htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;              // 计数方向：向上计数
  htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                 // 定时器周期
  htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;          // 时钟分频
  HAL_TIM_Base_Init(&htimx_STEPMOTOR);

  /* 定时器时钟源配置 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       		// 使用内部时钟源
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

  /* 定时器比较输出配置 */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // 比较输出模式：反转输出
  sConfigOC.Pulse = 0xFFFF;                            // 脉冲数
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;           // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // 互补通道空闲电平
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_TIM_CHANNEL_x);
  /* 使能比较输出通道 */
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  
  /* 配置定时器中断优先级并使能 */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);
  
  __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
  /* 使能定时器比较输出 */
  __HAL_TIM_DISABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htimx_STEPMOTOR);  
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);// 使能定时器
}

/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
}

/**
  * 函数功能: 相对位置运动：运动给定的步数
  * 输入参数: step：移动的步数 (正数为顺时针，负数为逆时针).
  *           accel:  加速度,实际值为accel*0.025*rad/sec^2
  *           decel:  减速度,实际值为decel*0.025*rad/sec^2
  *           speed:  最大速度,实际值为speed*0.05*rad/sec
  * 返 回 值: 无
  * 说    明: 以给定的步数移动步进电机，先加速到最大速度，然后在合适位置开始
  *           减速至停止，使得整个运动距离为指定的步数。如果加减速阶段很短并且
  *           速度很慢，那还没达到最大速度就要开始减速
  */
void STEPMOTOR_AxisMoveRel(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{  
	__IO uint16_t tim_count;//获取定时器的计数值
  // 达到最大速度时的步数
	__IO uint32_t max_s_lim;
  // 必须要开始减速的步数（如果加速没有达到最大速度）
	__IO uint32_t accel_lim;
	
	/* 方向设定 */
	if(step < 0) // 步数为负数
	{
		srd.dir = (int8_t)CCW; // 逆时针方向旋转
		STEPMOTOR_DIR_REVERSAL();
		step = -step;   // 获取步数绝对值
		
	}
	else
	{
		srd.dir = (int8_t)CW; // 顺时针方向旋转
		STEPMOTOR_DIR_FORWARD();
		
	}
	
	if(step == 1)    // 步数为1
	{
		srd.accel_count = -1;   // 只移动一步
		srd.run_state = DECEL;  // 减速状态.
		srd.step_delay = 1000;	// 短延时	
	}
	else if(step != 0)  // 如果目标运动步数不为0
	{
		// 我们的电机控制专题指导手册有详细的计算及推导过程

		// 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
		// min_delay = (alpha / tt)/ w
		srd.min_delay = (int32_t)(A_T_x10/speed);

		// 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
		srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);//C0,初始速度的定时器值
			

		/*计算加减速需要的参数*/
		// 计算多少步之后达到最大速度的限制
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
		// 如果达到最大速度小于0.5步，我们将四舍五入为0
		// 但实际我们必须移动至少一步才能达到想要的速度
		if(max_s_lim == 0)
		{
		  max_s_lim = 1;
		}
			
		// 计算多少步之后我们必须开始减速
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim = (uint32_t)(step*decel/(accel+decel));
		// 我们必须加速至少1步才能才能开始减速.
		if(accel_lim == 0)
		{
		  accel_lim = 1;
		}
		// 使用限制条件我们可以计算出减速阶段步数
		if(accel_lim <= max_s_lim)
		{
		  srd.decel_val = accel_lim - step;
		}
		else{
		  srd.decel_val = -(max_s_lim*accel/decel);
		}
		// 当只剩下一步我们必须减速
		if(srd.decel_val == 0)
		{
		  srd.decel_val = -1;
		}

		// 计算开始减速时的步数
		srd.decel_start = step + srd.decel_val;

		// 如果最大速度很慢，我们就不需要进行加速运动
		if(srd.step_delay <= srd.min_delay)
		{
			srd.step_delay = srd.min_delay;
			srd.run_state = RUN;
		}
		else
		{
			srd.run_state = ACCEL;
		}    
		// 复位加速度计数值
		srd.accel_count = 0;
	}
	if(srd.step_delay>65535)
		srd.step_delay -=65535;
  Motionstatus = 1;
	STEPMOTOR_OUTPUT_ENABLE();
	HAL_Delay(100);
	tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
	__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.step_delay); // 设置定时器比较值
	__HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道 
}

/** 函数功能: 移动到绝对位置:定点移动
  * 输入参数: targert_step:目标的位置
  *			      accel:加速度
  *			      decel:减速度
  *			      speed:最大速度
  * 返 回 值: 无
  * 说    明: 实现定点移动,输入目标位置相对于原点的步数,
  *  		      以加速度加速到最大速度后匀速寻找目标位置,
  *			      然后在合适的位置减速,到达目标位置.
  */
void STEPMOTOR_AxisMoveAbs(__IO int32_t targert_step,__IO uint32_t accel, __IO uint32_t decel,__IO uint32_t speed)
{
	__IO int32_t rel_step = 0;
	__IO int8_t dir = -1;
	rel_step = step_position-targert_step ; 	//获取当前位置和目标位置之间的步数值
	
	if(rel_step == 0)	
	{
		dir = 0;
	}
	else dir = -1;
	STEPMOTOR_AxisMoveRel(dir*rel_step,accel,decel,speed);//
}
/** 函数功能: 相对移动一定距离;单位:mm
  * 输入参数: distance  需要移动的距离,以正负区分方向,正数就是正方向,
  *                    负数就是反方向
  *            accel  加速度
  *            decel  减速度
  *            speed  最大速度
  * 返 回 值: 无
  * 说    明: 从当前位置上在导轨上移动一定的距离,以参数distance的正负区分方向
 */
void STEPMOTOR_DisMoveRel(__IO int16_t distance, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{
  __IO int32_t step;
  step = distance*UNIT_STEP_MM;//获得步进目标距离的步数
  STEPMOTOR_AxisMoveRel(step,accel,decel,speed);
}
/**
  * 函数功能: 移动到目标位置;单位:mm
  * 输入参数: Target_Dis  目标坐标位置
  *           accel  加速度
  *           decel  减速度
  *           speed  最大速度
  * 返 回 值: 最大速度
  * 说    明: 移动到给定的坐标位置,根据当前的位置计算出与目标位置的距离,
  *           并移动到目标位置,
  *             
  */
void STEPMOTOR_DisMoveAbs(__IO uint16_t Target_Dis, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{
  __IO int16_t step; 
    
  if(Target_Dis > MAX_DISTANCE)
    return;
  else if(Target_Dis < UNITS_DISTANCE)
    return;
  step = Target_Dis - location;     //获得当前位置与目标位置的距离
  STEPMOTOR_DisMoveRel(step,accel,decel,speed);
}
/** 
  * 函数功能: 搜索原点
  * 输入参数: FASTSEEK_SPEED 原点回归速度
  *   		    SLOWSEEK_SPEED 爬行速度
  *			      accel   加速度
  *			      decel   减速度
  * 返 回 值: 无
  * 说    明: 实现寻找原点,并在原点位置停下来,近点信号和原点信号以 KEY1 的下降沿
  *  	        和上升沿模拟,检测到下降沿表示捕获到近点信号,开始以slowseek_speed移动,上升
  *			      沿表示捕获到原点,立刻停止.
  */
void STEPMOTOR_AxisHome(__IO int32_t fastseek_speed, __IO uint32_t slowseek_speed,__IO uint32_t accel, __IO uint32_t decel)
{
  __IO  uint8_t static SC_Flag = 0;
	switch(ZeroStep)	
	{
		case FASTSEEK :			
				HAL_Delay(100);
				STEPMOTOR_AxisMoveRel(HomeDir*MAX_NUM_LAP,accel,decel,fastseek_speed);	//以回归速度运行
				srd.decel_start = MAX_NUM_STEP;	//修改srd参数值,使得电机在运动过程中不会减速,这句指令必须在STEPMOTOR_AxisMoveRel函数之后指执行	
				ZeroStep = SLOWSEEK;
			break;
		
		case SLOWSEEK ://检测到近点信号DOG,开始减速到回归速度
			if(DOG  == TRUE)//检测到DOG信号就开始减速
			{
				HAL_Delay(50);
				if( HAL_GPIO_ReadPin( ORIGIN_PORT,ORIGIN_PIN ) == ORIGIN_LEVEL)
				{
					srd.medle_delay = (int32_t)(A_T_x10/slowseek_speed);//回归速度的定时器值
					srd.decel_start = 0;	//设置开始减速的位置
					srd.run_state = RUN;	//这句指令必须在STEPMOTOR_AxisMoveRel函数之后指执行
					ZeroStep  = MOVETOZERO;
				}
				else DOG  = FALSE;
			}
      //特殊状况1:滑块在经过近点信号之后还未到原点的位置起步
      else if(HomeCapture  == TRUE)   
      {
        HomeCapture  = FALSE;
				srd.medle_delay = (int32_t)(A_T_x10/slowseek_speed);//回归速度的定时器值
				srd.decel_start = 0;	//开始减速
        ZeroStep  = SLOWSEEK;
      }
      //特殊状况2:滑块在原点和极限之间起步
      //特殊状况下滑块会接触到反转极限,需要换个方向
      else if(LimNega  == TRUE)      
      {
        if(Motionstatus == STOP)
        {
          SC_Flag = 1;
          HomeDir = CW;            //换个方向继续
          ZeroStep = FASTSEEK;
					HAL_Delay(200);
        }
        else
           ZeroStep = SLOWSEEK;
      }
			break;
		case MOVETOZERO: 								//检测到原点信号,立刻停止
			if(DOG == TRUE )
			{
				if(HomeCapture  == TRUE)
				{
          if(SC_Flag == 1)          //特殊状况2,3//重新开始搜索原点
          {
            SC_Flag = 0;
            srd.run_state = STOP;
            if(Motionstatus == STOP) //等待电机停下来
            {
              HAL_Delay(200);
              ZeroStep = SLOWSEEK;
              HomeDir = CCW;
              DOG = FALSE;
              HomeCapture = FALSE;
              ZeroStep = SLOWSEEK;
              STEPMOTOR_AxisMoveRel(HomeDir*MAX_NUM_LAP,accel,decel,slowseek_speed);	//以爬行速度运行
            }
            else
            {
              SC_Flag = 1;
              ZeroStep = MOVETOZERO;
            }
          }
          else                      //正常情况
          {
            DOG  = FALSE;
//            srd .run_state = STOP;
            step_position  = HOME_POSITION;	//当前位置位于原点
            location  = 0;
						CLEAR_BIT(EXTI->IMR, (uint16_t)ORIGIN_PIN);
            ZeroStep  = IDLE;				          //搜索原点完成
          }
				}
			}
			break;
		case IDLE:
			//两个标志位都没有置位,说明当前原点位置不确定
			if((DOG == FALSE)&&( HomeCapture == FALSE))	
      {
        //特殊状况3:在反转极限位置起步,同特殊状况2
          if( HAL_GPIO_ReadPin( LIMNEG_PORT,LIMNEG_PIN ) ==LIM_NEG_LEVEL)
          {
            SC_Flag = 1;
            HomeDir = CW;     //设置搜索原点方向
          }
          else
          {
            HomeDir = CCW;
          }       		
					SET_BIT(EXTI->IMR, (uint16_t)ORIGIN_PIN);
					HAL_NVIC_SetPriority(ORIGIN_EXTI_IRQn, 1, 0);
					HAL_NVIC_EnableIRQ(ORIGIN_EXTI_IRQn);					
          ZeroStep = FASTSEEK;
      }
      else HAL_NVIC_DisableIRQ(ORIGIN_EXTI_IRQn);
			break;
		default :break;
	}
}

/**
  * 函数功能: 定时器中断服务函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 实现加减速过程
  */
void STEPMOTOR_TIMx_IRQHandler(void)//定时器中断处理
{ 
	__IO uint16_t tim_count = 0;
	// 保存新（下）一个延时周期
	uint16_t new_step_delay = 0;
	__IO static uint16_t last_accel_delay = 0;
	// 总移动步数计数器
	__IO static uint32_t step_count = 0;
	// 记录new_step_delay中的余数，提高下一步计算的精度
	__IO static int32_t rest = 0;
	//定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
	__IO static uint8_t i=0;
  
	if(__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx) != RESET)
	{
		// 清除定时器中断
		__HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
    
		// 设置比较值
		tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
		__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.step_delay);

		i++;       // 定时器中断次数计数值
		if(i == 2) // 2次，说明已经输出一个完整脉冲
		{
			i = 0;   // 清零定时器中断次数计数值
			/* 在这里判断是否处于边缘运动 */
			if(srd.dir == CCW)
			{
				if(LimNega == TRUE)		//逆时针且 遇到反方向的极限,停止运动
				{
					srd.run_state = STOP;
					Motionstatus = STOP;
					LimPosi = FALSE;
				}
				else 
				{
					LimPosi = FALSE;	//没有遇到极限
          LimNega = FALSE;  //清标志位
				}
			}
			else 
			{
				if(LimPosi == TRUE)		//顺时针,遇到极限
				{
					srd.run_state = STOP;
					Motionstatus = STOP;
				}
				else 
				{
          LimPosi = FALSE;	//没有遇到极限
					LimNega = FALSE;  //清标志位
				}
			}
			switch(srd.run_state) // 加减速曲线阶段
			{
				case STOP:
					// 关闭通道
					TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE); 	
					__HAL_TIM_DISABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
					__HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
				
					step_count = 0;  // 清零步数计数器
					rest = 0;        // 清零余值
					last_accel_delay = 0;
          srd.accel_count = 0;
//					srd.step_delay = 0;
//					srd.min_delay = 0;
          Motionstatus = STOP;	
					break;
					
				case ACCEL:
					step_count++;      		// 步数加1
					if(srd.dir == CW)
					{		  	
						step_position++; 	  // 绝对位置加1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location++;
					}
					else
					{
						step_position--; 	  // 绝对位置减1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location--;
					}
					srd.accel_count++; 		// 加速计数值加1
					
					new_step_delay = srd.step_delay - (((2 *srd.step_delay) + rest)/(4 * srd.accel_count + 1));//计算新(下)一步脉冲周期(时间间隔)
					rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
					
					if(step_count >= srd.decel_start)		// 检查是否应该开始减速
					{
						srd.accel_count = srd.decel_val; 	// 加速计数值为减速阶段计数值的初始值
						srd.run_state = DECEL;           	// 下个脉冲进入减速阶段
					}
					else if(new_step_delay <= srd.min_delay)// 检查是否到达期望的最大速度
					{
            srd.accel_count = srd.decel_val; 	// 加速计数值为减速阶段计数值的初始值
						last_accel_delay = new_step_delay;
						new_step_delay = srd.min_delay;    	// 使用min_delay（对应最大速度speed）
						rest = 0;                          	// 清零余值
						srd.run_state = RUN;               	// 设置为匀速运行状态
					}	
          last_accel_delay = new_step_delay; 	// 保存加速过程中最后一次延时（脉冲周期）
					break;
					
				case RUN:
					step_count++; 		 // 步数加1
					if(srd.dir==CW)
					{	  	
						step_position++; // 绝对位置加1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location++;
					}
					else
					{
						step_position--; // 绝对位置减1
            if( 0 == (step_position%UNIT_STEP_MM ))
              location--;
					}
					new_step_delay = srd.min_delay;       // 使用min_delay（对应最大速度speed）
							
					if(step_count >= srd.decel_start)     // 需要开始减速
					{
						srd.accel_count = srd.decel_val;  // 减速步数做为加速计数值
						new_step_delay = last_accel_delay;// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
						srd.run_state = DECEL;            // 状态改变为减速
					}
					break;
					
				case DECEL:
					step_count++; 		 // 步数加1
					if(srd.dir == CW)
					{		  	
						step_position++; // 绝对位置加1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location++;
					}
					else
					{
						step_position--; // 绝对位置减1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location--;
					}
					srd.accel_count++;
					new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest)/(4 * srd.accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
					rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
				  //检查是否为最后一步
					if(srd.accel_count >= 0)
					{
							srd.run_state = STOP;
					}
					if(ZeroStep !=IDLE ) 
					{
						if(new_step_delay >= srd.medle_delay )		//检查是否到达第二速度
						{
							srd.min_delay = srd.medle_delay;
							new_step_delay = srd.medle_delay;
							srd.decel_start = MAX_NUM_STEP;
							rest = 0;       
							srd.run_state = RUN;
						}
					}
					break;
				default :break;
			}      
			srd.step_delay = new_step_delay; // 为下个(新的)延时(脉冲周期)赋值
		}
	}
}

/**
  * 函数功能: 外部中断服务函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 实现判断是否到达极限和检测原点信号
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==ORIGIN_PIN)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(ORIGIN_PIN);	
		if(HAL_GPIO_ReadPin(ORIGIN_PORT,ORIGIN_PIN) == ORIGIN_LEVEL )//下降沿为DOG信号
		{															  
			DOG = TRUE;				//检测到近点信号
		}
		else 				
		{
			HomeCapture = TRUE;		//捕获到原点
			if(DOG == TRUE)
      {
				srd.run_state = STOP;  //正常情况下回到原点
      }
		}
	}
	if(GPIO_Pin == LIMPOS_PIN)	//正方向位的极限引脚
	{
		__HAL_GPIO_EXTI_CLEAR_IT(LIMPOS_PIN);
		if(HAL_GPIO_ReadPin(LIMPOS_PORT,LIMPOS_PIN) == LIM_POS_LEVEL)
		{	
			LimPosi	= TRUE;      
			srd.run_state = STOP;				
		}
	}
	if(GPIO_Pin == LIMNEG_PIN)	//反方向位的极限引脚
	{
		__HAL_GPIO_EXTI_CLEAR_IT(LIMNEG_PIN);
		if(HAL_GPIO_ReadPin(LIMNEG_PORT,LIMNEG_PIN) == LIM_NEG_LEVEL)
		{
			LimNega = TRUE;
			srd.run_state = STOP;		
		}
	}
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
