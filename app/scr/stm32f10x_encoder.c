/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_encoder.c 
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : This file contains the software implementation for the
*                      encoder unit
* Update			 : double yang
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
********************************************************************************
*@brife：此文件的功能，得到速度和位置的反馈，提供实时的转子的位置信息，并转换成
		电角度为换向提供依据，电角度分辨率4*PPR/pole_pair，同时提供将定子D轴对齐磁通
		方向的启动方式。
		
		1.配置TIMx（x=2、3、4）为编码器接口模式，将输入的捕获溢出值设置为编码器一圈的
		分辨率4*PPR-1，开启溢出中断，每转一圈产生一个中断。
		
		2.位置采集的原则：在连续的时间内对位置采集两次,并判断是否在这两次之间产生了
		溢出中断,若产生，那么取第二次位置采集信息:转了多少圈+当前计数器的cnt值。
		
		3.速度计算的原则：Delta_angle*Speed_sampling_Frq 区分正负号
		
		4.启动对齐原则：将对齐角设置为90度，线性增大D轴电流，将定子D轴与磁铁的N级也即
		刺痛对齐
		
		5.两个观测变量：absolute_Position  velocity，对齐后位置为0，速度为0.
		
		6.速度检测的精准度问题：如果速度的采样频率是5K，那么在一个采样周期内，编码器有一
		个CNT的误差，那么速度的误差就会放大5K倍，在速度较低的时候，误差就会变得很大，
		在高速时采用M，低速时采用T，待解决。
		
		7.ENC的初始化以后就一直进入对转子的监控状态，电机的运行即使进入故障状态，
		编码器也一直在记录电机的位置，所以对齐的操作只需要在上电之后运行一次即可。
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x_encoder.h"
#include "stm32f10x_it.h"
#include "MC_Globals.h"
#include "stm32f10x_MClib.h"


/* Private define ------------------------------------------------------------*/
#define COUNTER_RESET       (u16) ((((s32)(ALIGNMENT_ANGLE)*4*ENCODER_PPR/360)\
                                                              -1)/POLE_PAIR_NUM)																									//表示90°电角度占的cnt值																												
#define ICx_FILTER          (u8) 8 // 8<-> 670nsec

#define SPEED_SAMPLING_TIME   PID_SPEED_SAMPLING_TIME
#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))

#define ENC_PRE_EMPTION_PRIORITY 2
#define ENC_SUB_PRIORITY 0

/* Private functions 表示只有这个C文件调用的函数---------------------------------------------------------*/
s16 ENC_Calc_Rot_Speed(void);

/* Private variables ---------------------------------------------------------*/
static s16 hPrevious_angle, hSpeed_Buffer[SPEED_BUFFER_SIZE], hRot_Speed;
static u8 bSpeed_Buffer_Index = 0;
static volatile u16 hEncoder_Timer_Overflow; 
static bool bIs_First_Measurement = TRUE;
static bool bError_Speed_Measurement = FALSE;

/*******************************************************************************
* Function Name  : ENC_Init
* Description    : General Purpose Timer x set-up for encoder speed/position 
*                  sensors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void ENC_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM4 clock source enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/* Enable GPIOD, clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
	 
	GPIO_StructInit(&GPIO_InitStructure);
	/* Configure PD12 PD13 as encoder input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Enable the TIM4 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ENC_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ENC_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(ENCODER_TIMER);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = (4*ENCODER_PPR)-1;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(ENCODER_TIMER, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(ENCODER_TIMER, TIM_EncoderMode_TI12, 
							 TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
	TIM_ICInit(ENCODER_TIMER, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(ENCODER_TIMER, &TIM_ICInitStructure);
	  
	// Clear all pending interrupts
	TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
	TIM_ITConfig(ENCODER_TIMER, TIM_IT_Update, ENABLE);
	//Reset counter
	ENCODER_TIMER->CNT = COUNTER_RESET;
	
	absolute_Positon = 0;
	velocity = 0;

	TIM_Cmd(ENCODER_TIMER, ENABLE);
}

/*******************************************************************************
* Function Name  : ENC_Get_Electrical_Angle
* Description    : Returns the absolute electrical Rotor angle 
* Input          : None
* Output         : None
* Return         : Rotor electrical angle: 0 -> 0 degrees, 
*                                          S16_MAX-> 180 degrees, 32767
*                                          S16_MIN-> -180 degrees -32768
*                                          Which  （s16）32767+1=-32768 
*                  Mechanical angle can be derived calling this function and 
*                  dividing by POLE_PAIR_NUM
*******************************************************************************/
s16 ENC_Get_Electrical_Angle(void)
{
	s32 temp;

	temp = (s32)(TIM_GetCounter(ENCODER_TIMER)) * (s32)(U32_MAX / (4*ENCODER_PPR));         
	temp *= POLE_PAIR_NUM;  
	return((s16)(temp/65536)); // s16 result
}

/*******************************************************************************
* Function Name  : ENC_Get_Mechanical_Angle
* Description    : Returns the absolute mechanical Rotor angle 
* Input          : None
* Output         : None
* Return         : Rotor mechanical angle: 0 -> 0 degrees, S16_MAX-> 180 degrees, 
                                            S16_MIN-> -180 degrees
*******************************************************************************/
s16 ENC_Get_Mechanical_Angle(void)
{
	s32 temp;

	temp = (s32)(TIM_GetCounter(ENCODER_TIMER)) * (s32)(U32_MAX / (4*ENCODER_PPR)) ;
	return((s16)(temp/65536)); // s16 result
}

/*******************************************************************************
* Function Name  : ENC_ResetEncoder
* Description    : Write the encoder counter with the value corresponding to
*                  ALIGNMENT_ANGLE
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ENC_ResetEncoder(void)
{
	//Reset counter
	ENCODER_TIMER->CNT = COUNTER_RESET;
	absolute_Positon = 0;
}

             
/*******************************************************************************
* Function Name  : ENC_Clear_Speed_Buffer
* Description    : Clear speed buffer used for average speed calculation  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ENC_Clear_Speed_Buffer(void)
{   
	u32 i;

	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		hSpeed_Buffer[i] = 0;
	}
		bIs_First_Measurement = TRUE;
	}

/*******************************************************************************
* Function Name  : ENC_Calc_Rot_Speed
* Description    : Compute return latest speed measurement 
* Input          : None
* Output         : s16
* Return         : Return motor speed in 0.1 Hz resolution. Since the encoder is
                   used as speed sensor, this routine will return the mechanical
                   speed of the motor (NOT the electrical frequency)
                   Mechanical frequency is equal to electrical frequency/(number 
                   of pair poles).
*******************************************************************************/
s16 ENC_Calc_Rot_Speed(void)
{   
	s32 wDelta_angle;
	u16 hEnc_Timer_Overflow_sample_one, hEnc_Timer_Overflow_sample_two;
	u16 hCurrent_angle_sample_one, hCurrent_angle_sample_two;
	signed long long temp;
	s16 haux;

	if (!bIs_First_Measurement)
	{
		// 1st reading of overflow counter    
		hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow; 
		// 1st reading of encoder timer counter
		hCurrent_angle_sample_one = ENCODER_TIMER->CNT;
		// 2nd reading of overflow counter
		hEnc_Timer_Overflow_sample_two = hEncoder_Timer_Overflow;  
		// 2nd reading of encoder timer counter
		hCurrent_angle_sample_two = ENCODER_TIMER->CNT;      

		// Reset hhEncoder_Timer_Overflow and read the counter value for the next
		// measurement
		hEncoder_Timer_Overflow = 0;
		haux = ENCODER_TIMER->CNT;   

		if (hEncoder_Timer_Overflow != 0) 
		{
		haux = ENCODER_TIMER->CNT; 
		hEncoder_Timer_Overflow = 0;            
		}

		if (hEnc_Timer_Overflow_sample_one != hEnc_Timer_Overflow_sample_two)
		{ //Compare sample 1 & 2 and check if an overflow has been generated right 
		//after the reading of encoder timer. If yes, copy sample 2 result in 
		//sample 1 for next process 
		hCurrent_angle_sample_one = hCurrent_angle_sample_two;
		hEnc_Timer_Overflow_sample_one = hEnc_Timer_Overflow_sample_two;
		}

		if ( (ENCODER_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
		{// encoder timer down-counting
			wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle - 
			(hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
		}
		else  
		{//encoder timer up-counting
			wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle + 
			(hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
		}

		// 速度=wDelta_angle * 速度采样周期
		temp = (signed long long)(wDelta_angle * Speed_sampling_freq); 
		velocity = temp;
		temp *= 10;  // 0.1 Hz resolution
		temp /= (4*ENCODER_PPR);
		
		absolute_Positon = absolute_Positon + wDelta_angle;

	} //is first measurement, discard it
	else
	{
		bIs_First_Measurement = FALSE;
		temp = 0;
		hEncoder_Timer_Overflow = 0;
		haux = ENCODER_TIMER->CNT; 

		// Check if hEncoder_Timer_Overflow is still zero. In case an overflow IT 
		// occured it resets overflow counter and wPWM_Counter_Angular_Velocity
		if (hEncoder_Timer_Overflow != 0) 
		{
			haux = ENCODER_TIMER->CNT; 
			hEncoder_Timer_Overflow = 0;            
		}		
	}

	hPrevious_angle = haux;  
	return((s16) temp);
}

/*******************************************************************************
* Function Name  : ENC_Get_Mechanical_Speed
* Description    : Export the value of the smoothed motor speed computed in 
*                  ENC_Calc_Average_Speed function  
* Input          : None
* Output         : s16
* Return         : Return motor speed in 0.1 Hz resolution. This routine 
                   will return the average mechanical speed of the motor.
*******************************************************************************/
s16 ENC_Get_Mechanical_Speed(void)
{
  return(hRot_Speed);
}

/*******************************************************************************
* Function Name  : ENC_Calc_Average_Speed
* Description    : Compute smoothed motor speed based on last SPEED_BUFFER_SIZE
                   informations and store it variable  
* Input          : None
* Output         : s16
* Return         : Return rotor speed in 0.1 Hz resolution. This routine 
                   will return the average mechanical speed of the motor.
*******************************************************************************/
void ENC_Calc_Average_Speed(void)
{   
	s32 wtemp;
	u16 hAbstemp;
	u32 i;
	u8 static bError_counter;

	wtemp = ENC_Calc_Rot_Speed( );//得到转子的速度,每10秒转，速度PID的计量单位
	hAbstemp = ( wtemp < 0 ? - wtemp :  wtemp);

	/* Checks for speed measurement errors when in RUN State and saturates if 
						necessary*/  
	if (State == RUN)
	{ //在运行时，检查速度测量错误是否为真 
		/*-------------------------------------------*/		   
		if(hAbstemp < MINIMUM_MECHANICAL_SPEED)
		{ //小于最小机械速度
			if (wtemp < 0)
			{
				wtemp = -(s32)(MINIMUM_MECHANICAL_SPEED);
			}
			else
			{
				wtemp = MINIMUM_MECHANICAL_SPEED;
			}
			bError_counter++;//误差计数加一
		}
		else  if (hAbstemp > MAXIMUM_MECHANICAL_SPEED) 
		{//大于最大机械速度
			if (wtemp < 0)
			{
				wtemp = -(s32)(MAXIMUM_MECHANICAL_SPEED);
			}
			else
			{
				wtemp = MAXIMUM_MECHANICAL_SPEED;
			}
			bError_counter++;//误差计数加一
		}
		else
		{ //否则误差计数不变
			bError_counter = 0;
		}
		/*-------------------------------------------*/

		if (bError_counter >= MAXIMUM_ERROR_NUMBER)
		{//大于最大误差计数，返回速度检测错误为真
			bError_Speed_Measurement = TRUE;
		}
		else
		{//否者为假
			bError_Speed_Measurement = FALSE;
		}
	}
	else
	{//不运行时，速度测量错误为假，并将错误计数值设为0
		bError_Speed_Measurement = FALSE;
		bError_counter = 0;
	}

	/* Compute the average of the read speeds */
	//计算读取速度的平均值，Buffer_Size=8
	hSpeed_Buffer[bSpeed_Buffer_Index] = (s16)wtemp;
	bSpeed_Buffer_Index++;

	if (bSpeed_Buffer_Index == SPEED_BUFFER_SIZE) 
	{
		bSpeed_Buffer_Index = 0;
	}

	wtemp=0;

	//对速度求均值
	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtemp += hSpeed_Buffer[i];
	}
	wtemp /= SPEED_BUFFER_SIZE;

	hRot_Speed = ((s16)(wtemp));
}

/*******************************************************************************
* Function Name  : ENC_ErrorOnFeedback
* Description    : Check for possible errors on speed measurement when State is 
*                  RUN. After MAXIMUM_ERROR_NUMBER consecutive speed measurement
*                  errors, the function return TRUE, else FALSE.
*                  Function return 
* Input          : None
* Output         : s16
* Return         : boolean variable
*******************************************************************************/
bool ENC_ErrorOnFeedback(void)
{
	return(bError_Speed_Measurement); 
}

/*******************************************************************************
* Function Name : ENC_Start_Up
* Description   : The purpose of this function is to perform the alignment of 
*                 PMSM torque and flux regulation during the alignment phase.
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
*******************************************************************************/
void ENC_Start_Up(void)
{
	static u32 wTimebase=0;//注意关键字static的修饰，生存周期是全局的，但是只能本函数使用，
	//只初始化一次wTimebase=0，然后每次运行都可保存他的值。

	if ( (wGlobal_Flags & FIRST_START) == FIRST_START)
	{
		// First Motor start-up, alignment must be performed
		wTimebase++;
		if(wTimebase <= T_ALIGNMENT_PWM_STEPS)
		{                  
			hFlux_Reference = I_ALIGNMENT * wTimebase / T_ALIGNMENT_PWM_STEPS;               
			hTorque_Reference = 0;

			Stat_Curr_a_b = SVPWM_3ShuntGetPhaseCurrentValues(); 
			Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b); 
			Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, ALIGNMENT_ANGLE_S16); 
		
			//调整力矩和磁通的输出Vd和Vq
			Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);   		
			Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure); 
			//对Vd和Vq做限制
			RevPark_Circle_Limitation( );

			Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);

			SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);			
		}
		else
		{
			wTimebase = 0;              
			ENC_ResetEncoder( );          
			Stat_Volt_q_d.qV_Component1 = Stat_Volt_q_d.qV_Component2 = 0;
			hTorque_Reference = PID_TORQUE_REFERENCE;
			hFlux_Reference = PID_FLUX_REFERENCE;
			wGlobal_Flags &= ~FIRST_START;   // alignment done only once 
			//Clear the speed acquisition of the alignment phase
			ENC_Clear_Speed_Buffer();	
			State = RUN;
		}
	}
	else
	{	
	State = RUN;
	}
} 

/*******************************************************************************
* Function Name  : TIMx_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
                   Encoder unit connected to TIMx (x = 2,3 or 4)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
  
void TIM4_IRQHandler(void)
{
	TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
	
	if (hEncoder_Timer_Overflow != U16_MAX)  
	{
		hEncoder_Timer_Overflow++;
	}  	
}


/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
