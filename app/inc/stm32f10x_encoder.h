/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_encoder.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file contains the software implementation for the
*                      encoder position and speed reading.
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
#ifndef __STM32F10x_ENCODER_H
#define __STM32F10x_ENCODER_H



#define ENCODER_TIMER   TIM4          // Encoder unit connected to TIM4

#define ENCODER_PPR      (u16)(1000)   // number of pulses per revolution

/* Define here the absolute value of the application minimum and maximum speed 
                                                                in rpm unit*/
#define MINIMUM_MECHANICAL_SPEED_RPM  (u32)0 //rpm
#define MAXIMUM_MECHANICAL_SPEED_RPM  (u32)3400 //rpm

#define MAXIMUM_ERROR_NUMBER (u8)16
#define SPEED_BUFFER_SIZE  16   // power of 2 required to ease computations

/*************************** Alignment settings *******************************/
//Alignemnt duration
#define T_ALIGNMENT           (u16) 1000    // Alignment time in ms

#define ALIGNMENT_ANGLE       (u16) 90//Degrees [0..359] 

//设置对齐到90°的最大电流，由采样电阻和运放的倍数决定。
#define I_ALIGNMENT           (u16) 8388
//Do not be modified
#define T_ALIGNMENT_PWM_STEPS     (u32) ((T_ALIGNMENT * SAMPLING_FREQ)/1000) 
#define ALIGNMENT_ANGLE_S16       (s16)((s32)(ALIGNMENT_ANGLE) * 65536/360)
#define MINIMUM_MECHANICAL_SPEED  (u16)(MINIMUM_MECHANICAL_SPEED_RPM/6)
#define MAXIMUM_MECHANICAL_SPEED  (u16)(MAXIMUM_MECHANICAL_SPEED_RPM/6)


/* Exported functions ------------------------------------------------------- */
void ENC_Init(void);
s16 ENC_Get_Electrical_Angle(void);
s16 ENC_Get_Mechanical_Angle(void);
void ENC_Clear_Speed_Buffer(void);
void ENC_ResetEncoder(void);
s16 ENC_Get_Mechanical_Speed(void);
void ENC_Calc_Average_Speed(void);
bool ENC_ErrorOnFeedback(void);
void ENC_Start_Up(void);

#endif  /*__STM32F10x_ENCODER_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
