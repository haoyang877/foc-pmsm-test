/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_Timebase.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This module handles time base. It used in display and 
*                      fault management, speed regulation, motor ramp-up  
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
********************************************************************************
*@brife 此文件主要是计时器的工作
		1.systick中断以500us为单位，进行个运行环境的延时
		2.TIM6作为通用计数器，完成伺服系统的采样周期，可调节速度环，位置环，和电流环
		
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"

/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "stm32f10x_it.h"
#include "stm32f10x_Timebase.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SYSTICK_PRE_EMPTION_PRIORITY 3
#define SYSTICK_SUB_PRIORITY 0

#define SPEED_SAMPLING_TIME   PID_SPEED_SAMPLING_TIME

//速度频率5K  72M/72/200=5K
#define Speed_sampling_tim_Prescaler (72-1)
#define Speed_sampling_tim_Period (500-1)
u16 Speed_sampling_freq = (u16)(72000000/((Speed_sampling_tim_Prescaler+1)*(Speed_sampling_tim_Period+1)));


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u16 hStart_Up_TimeLeft_500us =0;
static volatile u16 hTimebase_500us = 0;
static volatile u16 hTimebase_display_500us = 0;
static volatile u16 hKey_debounce_500us = 0;
volatile u8 bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;

/*******************************************************************************
* Function Name  : TB_Init
* Description    : TimeBase peripheral initialization. The base time is set to 
*                  500usec and the related interrupt is enabled  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Init(void)
{   
  	NVIC_InitTypeDef         NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* Enable the TIM6 global Interrupt */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_DeInit(TIM7);
	//500us
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_Period = 500-1;	

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SYSTICK_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SYSTICK_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM7, ENABLE);
}

/*******************************************************************************
* Function Name  : TB_Wait
* Description    : The function wait for a delay to be over.   
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Wait(u16 time)
{
	hTimebase_500us = time;    // delay = 'time' value * 0.5ms
	while (hTimebase_500us != 0) // wait and do nothing!
	{ }  
}
/*******************************************************************************
* Function Name  : TB_Set_Delay_500us
* Description    : Set delay utilized by main.c state machine.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_Delay_500us(u16 hDelay)
{
  hTimebase_500us = hDelay;
}  

/*******************************************************************************
* Function Name  : TB_Delay_IsElapsed
* Description    : Check if the delay set by TB_Set_Delay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_Delay_IsElapsed(void)
{
 if (hTimebase_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
}  

/*******************************************************************************
* Function Name  : TB_Set_DisplayDelay_500us
* Description    : Set Delay utilized by MC_Display.c module.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_DisplayDelay_500us(u16 hDelay)
{
  hTimebase_display_500us = hDelay;
}  

/*******************************************************************************
* Function Name  : TB_DisplayDelay_IsElapsed
* Description    : Check if the delay set by TB_Set_DisplayDelay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_DisplayDelay_IsElapsed(void)
{
 if (hTimebase_display_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

/*******************************************************************************
* Function Name  : TB_Set_DebounceDelay_500us
* Description    : Set Delay utilized by MC_Display.c module.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_DebounceDelay_500us(u8 hDelay)
{
  hKey_debounce_500us = hDelay;
}  

/*******************************************************************************
* Function Name  : TB_DebounceDelay_IsElapsed
* Description    : Check if the delay set by TB_Set_DebounceDelay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_DebounceDelay_IsElapsed(void)
{
 if (hKey_debounce_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

/*******************************************************************************
* Function Name  : TB_Set_StartUp_Timeout(STARTUP_TIMEOUT)
* Description    : Set Start up time out and initialize Start_up torque in  
*                  torque control.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_StartUp_Timeout(u16 hTimeout)
{
  hStart_Up_TimeLeft_500us = 2*hTimeout;  
}  
/*******************************************************************************
* Function Name  : TB_StartUp_Timeout_IsElapsed
* Description    : Set Start up time out.   
* Input          : None
* Output         : True if start up time out is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_StartUp_Timeout_IsElapsed(void)
{
 if (hStart_Up_TimeLeft_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

/*******************************************************************************
* Function Name  : TIM6 INIT
* Description    : This function handles TTIM6 Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Tim6_Init(void)
{
	NVIC_InitTypeDef         NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* Enable the TIM6 global Interrupt */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_DeInit(TIM6);
	
	TIM_TimeBaseStructure.TIM_Prescaler = Speed_sampling_tim_Prescaler;
	TIM_TimeBaseStructure.TIM_Period = Speed_sampling_tim_Period;	

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM6, ENABLE);
}

/*******************************************************************************
* Function Name  : TIM6 Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


void TIM6_IRQHandler(void)
{
	
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	
	ENC_Calc_Average_Speed( );//转子速度：转/(10秒)
	
	//速度模式
	if((wGlobal_Flags & SPEED_CONTROL) == SPEED_CONTROL)
	{
		if (State == RUN) 
		{
			PID_Speed_Coefficients_update(ENC_Get_Mechanical_Speed( ),&PID_Speed_InitStructure);
			FOC_CalcFluxTorqueRef( );        
		}                                                                      
	}
	//力矩模式：关键问题，在力矩模式下不超速，并且在电机没有停止的时候不能切换控制模式
	else
	{
		if(State == RUN)
		{
			FOC_TorqueCtrl( );
		}
	}	
}


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{ 
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	if (hTimebase_500us != 0)  
	{
		hTimebase_500us --;
	}
	//显示延时
	if (hTimebase_display_500us != 0)  
	{
		hTimebase_display_500us --;
	}
	//防抖动延时
	if (hKey_debounce_500us != 0)  
	{
		hKey_debounce_500us --;
	}
	//电机启动延时
	if (hStart_Up_TimeLeft_500us != 0)
	{
		hStart_Up_TimeLeft_500us--;
	}
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

