/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : Main program body.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 27/06/08 v2.0.1
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


void RCC_Configuration(void);
unsigned char Res_f=0;	 //上电防止出现过流

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{			
	SystemInit();
	
	/*速度反馈初始化*/	
	ENC_Init( );
	
	/*电流反馈初始化*/
	SVPWM_3ShuntInit();
	
	/*时基初始化，并启动TIM6计算速度，初始化PID*/
	TB_Init( );
	Tim6_Init( );
	PID_Init(&PID_Torque_InitStructure, &PID_Flux_InitStructure, &PID_Speed_InitStructure);	
	
	/*温度，电压数组初始化*/
	MCL_Init_Arrays();  

	/*交互界面初始化*/		
	KEYS_Init( );
	LCD_Display_init();
	/*-------------------*/
	Res_f=1; //上电完成

	//串口示波器初始化
	usart_init(115200);
	while(1)
	{ 
		
		/*UI显示，以及电源报警，用户管理*/
		Display_LCD( );
		MCL_ChkPowerStage( );    
		KEYS_process( );

		/*状态机开启运行*/
		switch (State)
		{
			case IDLE:    //通过sel按键进入INIT ，在WAIT和FAULT中进入IDEL
			break;

			case INIT:
				MCL_Init( );//初始化电机控制层
				TB_Set_StartUp_Timeout(3000);
				State = START; 
			break;

			case START:  
				//passage to state RUN is performed by startup functions; 
			break;

			case RUN:  //电机运行过程中，检测速度反馈是否存在问题      
				if(ENC_ErrorOnFeedback() == TRUE)
				{
					MCL_SetFault(SPEED_FEEDBACK);
				}

			break;  

			case STOP:  //关闭TIM1的输出，状态转为等待，停止电流检测，设置Valpha和Vbeta为0，计算三相占空比 
				TIM_CtrlPWMOutputs(TIM1, DISABLE);
				State = WAIT;								        
				SVPWM_3ShuntAdvCurrentReading(DISABLE);											
				Stat_Volt_alfa_beta.qV_Component1 = Stat_Volt_alfa_beta.qV_Component2 = 0;
				SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);
				TB_Set_Delay_500us(2000); // 1 sec delay		
			break;

			case WAIT:    // 等待速度为零时，将状态转为IDEL
				if (TB_Delay_IsElapsed( ) == TRUE) 
				{
					if(ENC_Get_Mechanical_Speed( ) ==0)             
					{              
						State = IDLE;              
					}
				}
			break;

			case FAULT: //状态变为IDEL，全局变量设为第一次启动                  
				if (MCL_ClearFault( ) == TRUE)
				{
					if(wGlobal_Flags & SPEED_CONTROL == SPEED_CONTROL)
					{
						//速度控制模式
						bMenu_index = CONTROL_MODE_MENU_1;
					}
					else
					{
						//力矩控制模式
						bMenu_index = CONTROL_MODE_MENU_6;
					} 					
					State = IDLE;
					wGlobal_Flags |= FIRST_START;
					Hall_Start_flag=0;
				}
			break;

			default:        
			break;
		}	
		usart_watcher(Speed_Iq_Id_watch);
		/********End of Usart_watch**************/
	}
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
