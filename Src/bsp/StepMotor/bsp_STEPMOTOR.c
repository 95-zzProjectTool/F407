/**
  ******************************************************************************
  * �ļ�����: bsp_STEPMOTOR.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-06-05
  * ��    ��: �����������������ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "StepMotor/bsp_STEPMOTOR.h" 
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
speedRampData srd               = {STOP,CW,0,0,0,0,0,0};         // �Ӽ������߱���

__IO uint8_t  ZeroStep = IDLE;	//����ԭ��״̬��
__IO uint8_t  LimPosi = FALSE ; //�������ޱ�־λ  True:���Ｋ��λ  False:δ���Ｋ��λ
__IO uint8_t  LimNega = FALSE ; //�������ޱ�־λ
__IO uint8_t  DOG         		  = FALSE;		  // �����ź�
__IO uint8_t  HomeCapture       = FALSE;		  // ԭ�㲶���־
__IO int8_t   HomeDir           = CCW;			  // �����ԭ�㷽��
__IO int32_t  step_position     = 0;          // ��ǰλ��   ��λ:������
__IO int16_t  location          = 0;          //��ǰλ��    ��λ:����(mm)
__IO uint8_t  Motionstatus      = 0;           //���״̬  0:ֹͣ  1:�˶�

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void STEPMOTOR_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* ���Ŷ˿�ʱ��ʹ�� */
  STEPMOTOR_TIM_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA_GPIO_CLK_ENABLE();
  
  LIMPOS_GPIO_CLK_ENABLE();
  LIMNEG_GPIO_CLK_ENABLE();
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL_PORT, &GPIO_InitStruct);
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR_PORT, &GPIO_InitStruct);
  
  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA_PORT, &GPIO_InitStruct);
  
  STEPMOTOR_DIR_FORWARD();
  STEPMOTOR_OUTPUT_DISABLE();
  
  /* �����������������*/
  GPIO_InitStruct.Pin = LIMPOS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(LIMPOS_PORT, &GPIO_InitStruct);
	
	/* �����������������*/
  GPIO_InitStruct.Pin = LIMNEG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(LIMNEG_PORT, &GPIO_InitStruct);
	/* �������ԭ��������*/
  GPIO_InitStruct.Pin = ORIGIN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(ORIGIN_PORT, &GPIO_InitStruct);
  
	HAL_NVIC_SetPriority(LIMPOS_EXTI_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(LIMPOS_EXTI_IRQn);
  
  HAL_NVIC_SetPriority(LIMNEG_EXTI_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(LIMNEG_EXTI_IRQn);
  
  HAL_NVIC_SetPriority(ORIGIN_EXTI_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(ORIGIN_EXTI_IRQn);
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void STEPMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  
  STEPMOTOR_TIM_RCC_CLK_ENABLE();
  
  /* STEPMOTOR���GPIO��ʼ������ */
  STEPMOTOR_GPIO_Init();
  
  /* ��ʱ�������������� */
  htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                          // ��ʱ�����
  htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;           // ��ʱ��Ԥ��Ƶ��
  htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;              // �����������ϼ���
  htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                 // ��ʱ������
  htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;          // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htimx_STEPMOTOR);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       		// ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                            // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;           // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_TIM_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  
  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);
  
  __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_DISABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htimx_STEPMOTOR);  
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);// ʹ�ܶ�ʱ��
}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* ������ʱ������ʱ��ʹ�� */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
}

/**
  * ��������: ���λ���˶����˶������Ĳ���
  * �������: step���ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
  *           accel:  ���ٶ�,ʵ��ֵΪaccel*0.025*rad/sec^2
  *           decel:  ���ٶ�,ʵ��ֵΪdecel*0.025*rad/sec^2
  *           speed:  ����ٶ�,ʵ��ֵΪspeed*0.05*rad/sec
  * �� �� ֵ: ��
  * ˵    ��: �Ը����Ĳ����ƶ�����������ȼ��ٵ�����ٶȣ�Ȼ���ں���λ�ÿ�ʼ
  *           ������ֹͣ��ʹ�������˶�����Ϊָ���Ĳ���������Ӽ��ٽ׶κ̲ܶ���
  *           �ٶȺ������ǻ�û�ﵽ����ٶȾ�Ҫ��ʼ����
  */
void STEPMOTOR_AxisMoveRel(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{  
	__IO uint16_t tim_count;//��ȡ��ʱ���ļ���ֵ
  // �ﵽ����ٶ�ʱ�Ĳ���
	__IO uint32_t max_s_lim;
  // ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�
	__IO uint32_t accel_lim;
	
	/* �����趨 */
	if(step < 0) // ����Ϊ����
	{
		srd.dir = (int8_t)CCW; // ��ʱ�뷽����ת
		STEPMOTOR_DIR_REVERSAL();
		step = -step;   // ��ȡ��������ֵ
		
	}
	else
	{
		srd.dir = (int8_t)CW; // ˳ʱ�뷽����ת
		STEPMOTOR_DIR_FORWARD();
		
	}
	
	if(step == 1)    // ����Ϊ1
	{
		srd.accel_count = -1;   // ֻ�ƶ�һ��
		srd.run_state = DECEL;  // ����״̬.
		srd.step_delay = 1000;	// ����ʱ	
	}
	else if(step != 0)  // ���Ŀ���˶�������Ϊ0
	{
		// ���ǵĵ������ר��ָ���ֲ�����ϸ�ļ��㼰�Ƶ�����

		// ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
		// min_delay = (alpha / tt)/ w
		srd.min_delay = (int32_t)(A_T_x10/speed);

		// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
		srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);//C0,��ʼ�ٶȵĶ�ʱ��ֵ
			

		/*����Ӽ�����Ҫ�Ĳ���*/
		// ������ٲ�֮��ﵽ����ٶȵ�����
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
		// ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
		// ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
		if(max_s_lim == 0)
		{
		  max_s_lim = 1;
		}
			
		// ������ٲ�֮�����Ǳ��뿪ʼ����
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim = (uint32_t)(step*decel/(accel+decel));
		// ���Ǳ����������1�����ܲ��ܿ�ʼ����.
		if(accel_lim == 0)
		{
		  accel_lim = 1;
		}
		// ʹ�������������ǿ��Լ�������ٽ׶β���
		if(accel_lim <= max_s_lim)
		{
		  srd.decel_val = accel_lim - step;
		}
		else{
		  srd.decel_val = -(max_s_lim*accel/decel);
		}
		// ��ֻʣ��һ�����Ǳ������
		if(srd.decel_val == 0)
		{
		  srd.decel_val = -1;
		}

		// ���㿪ʼ����ʱ�Ĳ���
		srd.decel_start = step + srd.decel_val;

		// �������ٶȺ��������ǾͲ���Ҫ���м����˶�
		if(srd.step_delay <= srd.min_delay)
		{
			srd.step_delay = srd.min_delay;
			srd.run_state = RUN;
		}
		else
		{
			srd.run_state = ACCEL;
		}    
		// ��λ���ٶȼ���ֵ
		srd.accel_count = 0;
	}
	if(srd.step_delay>65535)
		srd.step_delay -=65535;
  Motionstatus = 1;
	STEPMOTOR_OUTPUT_ENABLE();
	HAL_Delay(100);
	tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
	__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.step_delay); // ���ö�ʱ���Ƚ�ֵ
	__HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
	TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
}

/** ��������: �ƶ�������λ��:�����ƶ�
  * �������: targert_step:Ŀ���λ��
  *			      accel:���ٶ�
  *			      decel:���ٶ�
  *			      speed:����ٶ�
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ֶ����ƶ�,����Ŀ��λ�������ԭ��Ĳ���,
  *  		      �Լ��ٶȼ��ٵ�����ٶȺ�����Ѱ��Ŀ��λ��,
  *			      Ȼ���ں��ʵ�λ�ü���,����Ŀ��λ��.
  */
void STEPMOTOR_AxisMoveAbs(__IO int32_t targert_step,__IO uint32_t accel, __IO uint32_t decel,__IO uint32_t speed)
{
	__IO int32_t rel_step = 0;
	__IO int8_t dir = -1;
	rel_step = step_position-targert_step ; 	//��ȡ��ǰλ�ú�Ŀ��λ��֮��Ĳ���ֵ
	
	if(rel_step == 0)	
	{
		dir = 0;
	}
	else dir = -1;
	STEPMOTOR_AxisMoveRel(dir*rel_step,accel,decel,speed);//
}
/** ��������: ����ƶ�һ������;��λ:mm
  * �������: distance  ��Ҫ�ƶ��ľ���,���������ַ���,��������������,
  *                    �������Ƿ�����
  *            accel  ���ٶ�
  *            decel  ���ٶ�
  *            speed  ����ٶ�
  * �� �� ֵ: ��
  * ˵    ��: �ӵ�ǰλ�����ڵ������ƶ�һ���ľ���,�Բ���distance���������ַ���
 */
void STEPMOTOR_DisMoveRel(__IO int16_t distance, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{
  __IO int32_t step;
  step = distance*UNIT_STEP_MM;//��ò���Ŀ�����Ĳ���
  STEPMOTOR_AxisMoveRel(step,accel,decel,speed);
}
/**
  * ��������: �ƶ���Ŀ��λ��;��λ:mm
  * �������: Target_Dis  Ŀ������λ��
  *           accel  ���ٶ�
  *           decel  ���ٶ�
  *           speed  ����ٶ�
  * �� �� ֵ: ����ٶ�
  * ˵    ��: �ƶ�������������λ��,���ݵ�ǰ��λ�ü������Ŀ��λ�õľ���,
  *           ���ƶ���Ŀ��λ��,
  *             
  */
void STEPMOTOR_DisMoveAbs(__IO uint16_t Target_Dis, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{
  __IO int16_t step; 
    
  if(Target_Dis > MAX_DISTANCE)
    return;
  else if(Target_Dis < UNITS_DISTANCE)
    return;
  step = Target_Dis - location;     //��õ�ǰλ����Ŀ��λ�õľ���
  STEPMOTOR_DisMoveRel(step,accel,decel,speed);
}
/** 
  * ��������: ����ԭ��
  * �������: FASTSEEK_SPEED ԭ��ع��ٶ�
  *   		    SLOWSEEK_SPEED �����ٶ�
  *			      accel   ���ٶ�
  *			      decel   ���ٶ�
  * �� �� ֵ: ��
  * ˵    ��: ʵ��Ѱ��ԭ��,����ԭ��λ��ͣ����,�����źź�ԭ���ź��� KEY1 ���½���
  *  	        ��������ģ��,��⵽�½��ر�ʾ���񵽽����ź�,��ʼ��slowseek_speed�ƶ�,����
  *			      �ر�ʾ����ԭ��,����ֹͣ.
  */
void STEPMOTOR_AxisHome(__IO int32_t fastseek_speed, __IO uint32_t slowseek_speed,__IO uint32_t accel, __IO uint32_t decel)
{
  __IO  uint8_t static SC_Flag = 0;
	switch(ZeroStep)	
	{
		case FASTSEEK :			
				HAL_Delay(100);
				STEPMOTOR_AxisMoveRel(HomeDir*MAX_NUM_LAP,accel,decel,fastseek_speed);	//�Իع��ٶ�����
				srd.decel_start = MAX_NUM_STEP;	//�޸�srd����ֵ,ʹ�õ�����˶������в������,���ָ�������STEPMOTOR_AxisMoveRel����֮��ִָ��	
				ZeroStep = SLOWSEEK;
			break;
		
		case SLOWSEEK ://��⵽�����ź�DOG,��ʼ���ٵ��ع��ٶ�
			if(DOG  == TRUE)//��⵽DOG�źžͿ�ʼ����
			{
				HAL_Delay(50);
				if( HAL_GPIO_ReadPin( ORIGIN_PORT,ORIGIN_PIN ) == ORIGIN_LEVEL)
				{
					srd.medle_delay = (int32_t)(A_T_x10/slowseek_speed);//�ع��ٶȵĶ�ʱ��ֵ
					srd.decel_start = 0;	//���ÿ�ʼ���ٵ�λ��
					srd.run_state = RUN;	//���ָ�������STEPMOTOR_AxisMoveRel����֮��ִָ��
					ZeroStep  = MOVETOZERO;
				}
				else DOG  = FALSE;
			}
      //����״��1:�����ھ��������ź�֮��δ��ԭ���λ����
      else if(HomeCapture  == TRUE)   
      {
        HomeCapture  = FALSE;
				srd.medle_delay = (int32_t)(A_T_x10/slowseek_speed);//�ع��ٶȵĶ�ʱ��ֵ
				srd.decel_start = 0;	//��ʼ����
        ZeroStep  = SLOWSEEK;
      }
      //����״��2:������ԭ��ͼ���֮����
      //����״���»����Ӵ�����ת����,��Ҫ��������
      else if(LimNega  == TRUE)      
      {
        if(Motionstatus == STOP)
        {
          SC_Flag = 1;
          HomeDir = CW;            //�����������
          ZeroStep = FASTSEEK;
					HAL_Delay(200);
        }
        else
           ZeroStep = SLOWSEEK;
      }
			break;
		case MOVETOZERO: 								//��⵽ԭ���ź�,����ֹͣ
			if(DOG == TRUE )
			{
				if(HomeCapture  == TRUE)
				{
          if(SC_Flag == 1)          //����״��2,3//���¿�ʼ����ԭ��
          {
            SC_Flag = 0;
            srd.run_state = STOP;
            if(Motionstatus == STOP) //�ȴ����ͣ����
            {
              HAL_Delay(200);
              ZeroStep = SLOWSEEK;
              HomeDir = CCW;
              DOG = FALSE;
              HomeCapture = FALSE;
              ZeroStep = SLOWSEEK;
              STEPMOTOR_AxisMoveRel(HomeDir*MAX_NUM_LAP,accel,decel,slowseek_speed);	//�������ٶ�����
            }
            else
            {
              SC_Flag = 1;
              ZeroStep = MOVETOZERO;
            }
          }
          else                      //�������
          {
            DOG  = FALSE;
//            srd .run_state = STOP;
            step_position  = HOME_POSITION;	//��ǰλ��λ��ԭ��
            location  = 0;
						CLEAR_BIT(EXTI->IMR, (uint16_t)ORIGIN_PIN);
            ZeroStep  = IDLE;				          //����ԭ�����
          }
				}
			}
			break;
		case IDLE:
			//������־λ��û����λ,˵����ǰԭ��λ�ò�ȷ��
			if((DOG == FALSE)&&( HomeCapture == FALSE))	
      {
        //����״��3:�ڷ�ת����λ����,ͬ����״��2
          if( HAL_GPIO_ReadPin( LIMNEG_PORT,LIMNEG_PIN ) ==LIM_NEG_LEVEL)
          {
            SC_Flag = 1;
            HomeDir = CW;     //��������ԭ�㷽��
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
  * ��������: ��ʱ���жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void STEPMOTOR_TIMx_IRQHandler(void)//��ʱ���жϴ���
{ 
	__IO uint16_t tim_count = 0;
	// �����£��£�һ����ʱ����
	uint16_t new_step_delay = 0;
	__IO static uint16_t last_accel_delay = 0;
	// ���ƶ�����������
	__IO static uint32_t step_count = 0;
	// ��¼new_step_delay�е������������һ������ľ���
	__IO static int32_t rest = 0;
	//��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
	__IO static uint8_t i=0;
  
	if(__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx) != RESET)
	{
		// �����ʱ���ж�
		__HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
    
		// ���ñȽ�ֵ
		tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
		__HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.step_delay);

		i++;       // ��ʱ���жϴ�������ֵ
		if(i == 2) // 2�Σ�˵���Ѿ����һ����������
		{
			i = 0;   // ���㶨ʱ���жϴ�������ֵ
			/* �������ж��Ƿ��ڱ�Ե�˶� */
			if(srd.dir == CCW)
			{
				if(LimNega == TRUE)		//��ʱ���� ����������ļ���,ֹͣ�˶�
				{
					srd.run_state = STOP;
					Motionstatus = STOP;
					LimPosi = FALSE;
				}
				else 
				{
					LimPosi = FALSE;	//û����������
          LimNega = FALSE;  //���־λ
				}
			}
			else 
			{
				if(LimPosi == TRUE)		//˳ʱ��,��������
				{
					srd.run_state = STOP;
					Motionstatus = STOP;
				}
				else 
				{
          LimPosi = FALSE;	//û����������
					LimNega = FALSE;  //���־λ
				}
			}
			switch(srd.run_state) // �Ӽ������߽׶�
			{
				case STOP:
					// �ر�ͨ��
					TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE); 	
					__HAL_TIM_DISABLE_IT(&htimx_STEPMOTOR, STEPMOTOR_TIM_IT_CCx);
					__HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
				
					step_count = 0;  // ���㲽��������
					rest = 0;        // ������ֵ
					last_accel_delay = 0;
          srd.accel_count = 0;
//					srd.step_delay = 0;
//					srd.min_delay = 0;
          Motionstatus = STOP;	
					break;
					
				case ACCEL:
					step_count++;      		// ������1
					if(srd.dir == CW)
					{		  	
						step_position++; 	  // ����λ�ü�1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location++;
					}
					else
					{
						step_position--; 	  // ����λ�ü�1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location--;
					}
					srd.accel_count++; 		// ���ټ���ֵ��1
					
					new_step_delay = srd.step_delay - (((2 *srd.step_delay) + rest)/(4 * srd.accel_count + 1));//������(��)һ����������(ʱ����)
					rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// �����������´μ��㲹���������������
					
					if(step_count >= srd.decel_start)		// ����Ƿ�Ӧ�ÿ�ʼ����
					{
						srd.accel_count = srd.decel_val; 	// ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
						srd.run_state = DECEL;           	// �¸����������ٽ׶�
					}
					else if(new_step_delay <= srd.min_delay)// ����Ƿ񵽴�����������ٶ�
					{
            srd.accel_count = srd.decel_val; 	// ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
						last_accel_delay = new_step_delay;
						new_step_delay = srd.min_delay;    	// ʹ��min_delay����Ӧ����ٶ�speed��
						rest = 0;                          	// ������ֵ
						srd.run_state = RUN;               	// ����Ϊ��������״̬
					}	
          last_accel_delay = new_step_delay; 	// ������ٹ��������һ����ʱ���������ڣ�
					break;
					
				case RUN:
					step_count++; 		 // ������1
					if(srd.dir==CW)
					{	  	
						step_position++; // ����λ�ü�1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location++;
					}
					else
					{
						step_position--; // ����λ�ü�1
            if( 0 == (step_position%UNIT_STEP_MM ))
              location--;
					}
					new_step_delay = srd.min_delay;       // ʹ��min_delay����Ӧ����ٶ�speed��
							
					if(step_count >= srd.decel_start)     // ��Ҫ��ʼ����
					{
						srd.accel_count = srd.decel_val;  // ���ٲ�����Ϊ���ټ���ֵ
						new_step_delay = last_accel_delay;// �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
						srd.run_state = DECEL;            // ״̬�ı�Ϊ����
					}
					break;
					
				case DECEL:
					step_count++; 		 // ������1
					if(srd.dir == CW)
					{		  	
						step_position++; // ����λ�ü�1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location++;
					}
					else
					{
						step_position--; // ����λ�ü�1
            if( 0 == (step_position%UNIT_STEP_MM )) 
              location--;
					}
					srd.accel_count++;
					new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest)/(4 * srd.accel_count + 1)); //������(��)һ����������(ʱ����)
					rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// �����������´μ��㲹���������������
				  //����Ƿ�Ϊ���һ��
					if(srd.accel_count >= 0)
					{
							srd.run_state = STOP;
					}
					if(ZeroStep !=IDLE ) 
					{
						if(new_step_delay >= srd.medle_delay )		//����Ƿ񵽴�ڶ��ٶ�
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
			srd.step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
		}
	}
}

/**
  * ��������: �ⲿ�жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ���ж��Ƿ񵽴Ｋ�޺ͼ��ԭ���ź�
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==ORIGIN_PIN)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(ORIGIN_PIN);	
		if(HAL_GPIO_ReadPin(ORIGIN_PORT,ORIGIN_PIN) == ORIGIN_LEVEL )//�½���ΪDOG�ź�
		{															  
			DOG = TRUE;				//��⵽�����ź�
		}
		else 				
		{
			HomeCapture = TRUE;		//����ԭ��
			if(DOG == TRUE)
      {
				srd.run_state = STOP;  //��������»ص�ԭ��
      }
		}
	}
	if(GPIO_Pin == LIMPOS_PIN)	//������λ�ļ�������
	{
		__HAL_GPIO_EXTI_CLEAR_IT(LIMPOS_PIN);
		if(HAL_GPIO_ReadPin(LIMPOS_PORT,LIMPOS_PIN) == LIM_POS_LEVEL)
		{	
			LimPosi	= TRUE;      
			srd.run_state = STOP;				
		}
	}
	if(GPIO_Pin == LIMNEG_PIN)	//������λ�ļ�������
	{
		__HAL_GPIO_EXTI_CLEAR_IT(LIMNEG_PIN);
		if(HAL_GPIO_ReadPin(LIMNEG_PORT,LIMNEG_PIN) == LIM_NEG_LEVEL)
		{
			LimNega = TRUE;
			srd.run_state = STOP;		
		}
	}
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
