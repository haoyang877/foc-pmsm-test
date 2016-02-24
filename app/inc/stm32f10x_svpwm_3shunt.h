/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_svpwm_3shunt.h
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : This file contains declaration of functions exported by 
*                      module stm32x_svpwm_3shunt.c
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SVPWM_3SHUNT_H
#define __STM32F10x_SVPWM_3SHUNT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x_MClib.h"
#include "MC_const.h"
#include "MC_Control_Param.h"

/* Exported types ------------------------------------------------------------*/

/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */

////////////////////// PWM Frequency ///////////////////////////////////

/****	 Pattern type is center aligned  ****/

#define PWM_PRSC ((u8)0)

/* Resolution: 1Hz */                            
#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
        
////////////////////////////// Deadtime Value /////////////////////////////////
#define DEADTIME  (u16)((unsigned long long)CKTIM/2*(unsigned long long)DEADTIME_NS/1000000000uL) 

///////////////////////////// Current reading parameters //////////////////////

#define PHASE_A_ADC_CHANNEL     ADC_Channel_11
#define PHASE_A_GPIO_PORT       GPIOC
#define PHASE_A_GPIO_PIN        GPIO_Pin_1

#define PHASE_B_ADC_CHANNEL     ADC_Channel_12
#define PHASE_B_GPIO_PORT       GPIOC
#define PHASE_B_GPIO_PIN        GPIO_Pin_2

#define PHASE_C_ADC_CHANNEL     ADC_Channel_13
#define PHASE_C_GPIO_PORT       GPIOC
#define PHASE_C_GPIO_PIN        GPIO_Pin_3

#define SAMPLING_TIME_NS   200  //200ns
//#define SAMPLING_TIME_NS   700  //700ns
//#define SAMPLING_TIME_NS  1200  //1.2us
//#define SAMPLING_TIME_NS  2450  //2.45us

#if (SAMPLING_TIME_NS == 200)
#define SAMPLING_TIME_CK  ADC_SampleTime_1Cycles5
#elif (SAMPLING_TIME_NS == 700)
#define SAMPLING_TIME_CK  ADC_SampleTime_7Cycles5
#elif (SAMPLING_TIME_NS == 1200)
#define SAMPLING_TIME_CK  ADC_SampleTime_13Cycles5
#elif (SAMPLING_TIME_NS == 2450)
#define SAMPLING_TIME_CK  ADC_SampleTime_28Cycles5
#else
#warning "Sampling time is not a possible value"
#endif

#define TNOISE_NS 2550
#define TRISE_NS 2550

#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS) * 72uL)/1000uL) 
#define TNOISE (u16)((((u16)(TNOISE_NS)) * 72uL)/1000uL)
#define TRISE (u16)((((u16)(TRISE_NS)) * 72uL)/1000uL)
#define TDEAD (u16)((DEADTIME_NS * 72uL)/1000uL)

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

#define TW_AFTER ((u16)(((DEADTIME_NS+MAX_TNTR_NS)*72ul)/1000ul))  //用系统时间节拍 表示ns TW_AFTER=241.2
#define TW_BEFORE (((u16)(((((u16)(SAMPLING_TIME_NS)))*72ul)/1000ul))+1)

/****  Power Stage management Conversions setting ******/

#define TEMP_FDBK_CHANNEL                 ADC_Channel_10
#define TEMP_FDBK_CHANNEL_GPIO_PORT       GPIOC
#define TEMP_FDBK_CHANNEL_GPIO_PIN        GPIO_Pin_0

#define BUS_VOLT_FDBK_CHANNEL             ADC_Channel_3
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PORT   GPIOA
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PIN    GPIO_Pin_3

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SVPWM_3ShuntInit(void);
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void);
void SVPWM_3ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input);
void SVPWM_3ShuntCurrentReadingCalibration(void);
void SVPWM_3ShuntAdvCurrentReading(FunctionalState cmd);
void SVPWMUpdateEvent(void);
u8 SVPWMEOCEvent(void);

extern u8  bSector;  

#endif /* __STM32F10x_SVPWM_3SHUNT_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
