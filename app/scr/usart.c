
/********************************************************************************
*@brife 此文件主要是配合串口示波器使用，用于实时显示相关参数的曲线，只能输出不能输入
		1.一般讲曲线打印的相关的任务放在while（1）函数中
		
*******************************************************************************/
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "stdio.h"
#include "usart.h"


void usart_init(int baudrate)
{  
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 
	
	/*** System Clocks Configuration *****************************/
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(USARTy_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	  
  /* Enable USARTy Clock USART1 belongs to APB2 */
  RCC_APB2PeriphClockCmd(USARTy_CLK, ENABLE);  
	      
	/*USART1 use as default*/
  /* Configure USARTy Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USARTy_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);
	
  /* Configure USARTy Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USARTy_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//推挽输出
  GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);
	
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USARTy, &USART_InitStructure);
  USART_Cmd(USARTy, ENABLE);


}

void usart_watcher(Watcher_type type)
{
	switch(type)
	{
		case I_alpha_beta_watch:
			printf("%d,%d\r",Stat_Curr_alfa_beta.qI_Component1,Stat_Curr_alfa_beta.qI_Component2);
		break;
		
		case Speed_Iq_Id_watch:
			printf("  %d,%d,%d, %d,%d,%d, %d,%d,%d \r", 
			hSpeed_Reference, GET_SPEED_0_1HZ*6,0,
			hTorque_Reference,Stat_Curr_q_d.qI_Component1,0,
			hFlux_Reference,Stat_Curr_q_d.qI_Component2,0);
		break;
				
		case Electric_Angle_hallstate_section:
			printf("%d,%d,%d\r",ENC_Get_Mechanical_Angle(),ReadHallState(),bSector);	
		break;
		
		default:
		break;              
					
	}
}

/*串口打印函数原型*/
/*USARTy作为与pc机通信的端口*/
int fputc( int ch, FILE *f )
{
	  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USARTy, USART_FLAG_TC) == RESET)
  {   }
  /* Place your implementation of fputc here */
		
  /* e.g. write a character to the USART */
  USART_SendData(USARTy, (u16) ch);

  return ch;
}
