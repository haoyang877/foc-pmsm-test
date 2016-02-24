/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * *@brife：此文件的功能包含3个中断

		1.ADC1_2_IRQHandler:完成从Start到RUN的切换，完成FOC运算，响应模拟看门狗中断
		模拟看门狗中断可以用来控制刹车信号
		
		2.TIM1_BRK_IRQHandler：响应过流中断
		
		3.TIM1_UP_IRQHandler：响应TIM1计数器溢出中断，使能ADC外部转换触发，在ADC中断
		FOC运行时关闭了ADC触发转化，在每个计数的溢出时，开启。
		
		PWM_FRQ=20Khz,中心对齐模式，Period=1800,那么开启的周期是1/(2*20K)=25us
		FOC运行时间时间是多久呢，按理来说一次FOC运算一次PWM周期，或者FOC运算，一次PWM周期
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

extern unsigned char Res_f;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BRAKE_HYSTERESIS (u16)((OVERVOLTAGE_THRESHOLD/16)*15)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
void ADC1_2_IRQHandler(void)
{	
	/*1.注入转换中断*/
	if((ADC1->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
	{
		//It clear JEOC flag
		ADC1->SR = ~(u32)ADC_FLAG_JEOC;

		if (SVPWMEOCEvent())
		{
			MCL_Calc_BusVolt( );
			switch (State)
			{
				case RUN:          
					FOC_Model( );       
				break;       

				case START:        
					ENC_Start_Up( );					
				break; 

				default:
				break;
			}			
		}
	}

	/*2.模拟看门狗中断*/
	else 
	{
		if(ADC_GetITStatus(ADC1, ADC_IT_AWD) == SET)
		{	
			if(MCL_Chk_BusVolt( )==OVER_VOLT)  // 防止干扰
			MCL_SetFault(OVER_VOLTAGE);
			ADC_ClearFlag(ADC1, ADC_FLAG_AWD);
		}    
	}
}
/*******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : This function handles TIM1 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
  if(Res_f==1)//上电完成并且进入到刹车中断
  MCL_SetFault(OVER_CURRENT);
  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
}

/*******************************************************************************
* Function Name  : TIM1_UP_IRQHandler
* Description    : This function handles TIM1 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_UP_IRQHandler(void)
{
	// Clear Update Flag
	TIM_ClearFlag(TIM1, TIM_FLAG_Update); 
	SVPWMUpdateEvent( );
}
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/




/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
