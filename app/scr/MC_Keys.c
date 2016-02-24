/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Keys.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file handles Joystick and button management
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
********************************************************************************
*@brife 本文件用于管理KEYs和LED的运行，可直接移植
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "MC_Keys.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define KEY_UP_PORT GPIOG
#define KEY_UP_BIT  GPIO_Pin_15

#define KEY_DOWN_PORT GPIOG
#define KEY_DOWN_BIT  GPIO_Pin_12

#define KEY_RIGHT_PORT GPIOG
#define KEY_RIGHT_BIT  GPIO_Pin_13

#define KEY_LEFT_PORT GPIOG	   //CRE上LEFT不用
#define KEY_LEFT_BIT  GPIO_Pin_14
						 
#define KEY_SEL_PORT GPIOG
#define KEY_SEL_BIT  GPIO_Pin_10

//#define USER_BUTTON_PORT GPIOA
//#define USER_BUTTON_BIT  GPIO_Pin_9	

//LED PF[0..3]  低电平有效
#define LED1_GPIO  		GPIOF
#define LED1_GPIO_CLK RCC_APB2Periph_GPIOF
#define LED1_GPIO_Pin GPIO_Pin_0

#define LED2_GPIO      GPIOF
#define LED2_GPIO_CLK  RCC_APB2Periph_GPIOF
#define LED2_GPIO_Pin  GPIO_Pin_1

#define LED3_GPIO  	   GPIOF
#define LED3_GPIO_CLK  RCC_APB2Periph_GPIOF
#define LED3_GPIO_Pin  GPIO_Pin_2

#define LED4_GPIO  	   GPIOF
#define LED4_GPIO_CLK  RCC_APB2Periph_GPIOF
#define LED4_GPIO_Pin  GPIO_Pin_3

#define  SEL_FLAG        (u8)0x02
#define  RIGHT_FLAG      (u8)0x04
#define  LEFT_FLAG       (u8)0x08
#define  UP_FLAG         (u8)0x10
#define  DOWN_FLAG       (u8)0x20

//Variable increment and decrement

#define SPEED_INC_DEC     (u16)10
#define KP_GAIN_INC_DEC   (u16)250
#define KI_GAIN_INC_DEC   (u16)25
#define KD_GAIN_INC_DEC   (u16)100

#define TORQUE_INC_DEC    (u16)250
#define FLUX_INC_DEC      (u16)250


/* Private macro -------------------------------------------------------------*/
u8 KEYS_Read (void);
/* Private variables ---------------------------------------------------------*/
static u8 bKey;
static u8 bPrevious_key;
static u8 bKey_Flag;
u8 bMenu_index ;

void LEDS_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//LED1 
	RCC_APB2PeriphClockCmd(LED1_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin =	LED1_GPIO_Pin|LED2_GPIO_Pin|LED3_GPIO_Pin|LED4_GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED1_GPIO, &GPIO_InitStructure);
	
	LED_OFF(ALL_LED);
}

void LED_ON(u8 led)
{
	switch(led)
	{
		case 1:
			GPIO_ResetBits(LED1_GPIO,LED1_GPIO_Pin);
		break;
		case 2:
			GPIO_ResetBits(LED2_GPIO,LED2_GPIO_Pin);
		break;
		case 3:
			GPIO_ResetBits(LED3_GPIO,LED3_GPIO_Pin);
		break;
		case 4:
			GPIO_ResetBits(LED4_GPIO,LED4_GPIO_Pin);
		break;
		
		case ALL_LED:
			GPIO_ResetBits(LED1_GPIO,LED1_GPIO_Pin|LED2_GPIO_Pin|LED3_GPIO_Pin|LED4_GPIO_Pin);
		
		default:
		break;
		
	}
}

void LED_OFF(u8 led)
{
	switch(led)
	{
		case 1:
			GPIO_SetBits(LED1_GPIO,LED1_GPIO_Pin);
		break;
		case 2:
			GPIO_SetBits(LED2_GPIO,LED2_GPIO_Pin);
		break;
		case 3:
			GPIO_SetBits(LED3_GPIO,LED3_GPIO_Pin);
		break;
		
		case 4:
			GPIO_SetBits(LED4_GPIO,LED4_GPIO_Pin);
		break;
		
		case ALL_LED:
			GPIO_SetBits(LED1_GPIO,LED1_GPIO_Pin|LED2_GPIO_Pin|LED3_GPIO_Pin|LED4_GPIO_Pin);
		
		default:
		break;
	}
}

/*******************************************************************************
* Function Name  : KEYS_Init
* Description    : Init GPIOs for joystick/button management
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void KEYS_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    
  /* Enable GPIOA, GPIOB, GPIOC, GPIOE clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |RCC_APB2Periph_GPIOG|
                         RCC_APB2Periph_GPIOE, ENABLE);
 
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Joystick GPIOs configuration*/
  
  GPIO_InitStructure.GPIO_Pin = KEY_UP_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(KEY_UP_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = KEY_DOWN_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(KEY_DOWN_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = KEY_RIGHT_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(KEY_RIGHT_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = KEY_LEFT_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(KEY_LEFT_PORT, &GPIO_InitStructure);   
  
  GPIO_InitStructure.GPIO_Pin = KEY_SEL_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(KEY_SEL_PORT, &GPIO_InitStructure);
  
  /*
  GPIO_InitStructure.GPIO_Pin = USER_BUTTON_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USER_BUTTON_PORT, &GPIO_InitStructure);	 */
}
  


/*******************************************************************************
* Function Name  : KEYS_Read
* Description    : Reads key from demoboard.
* Input          : None
* Output         : None
* Return         : Return RIGHT, LEFT, SEL, UP, DOWN, KEY_HOLD or NOKEY
*******************************************************************************/
u8 KEYS_Read ( void )
{
	/* "RIGHT" key is pressed */
	//按下按键检测到低电平，如果上一个按键相同保持不变，不同则返回被按下的按键，
	//更改上一个按键为本次按键
	if(!GPIO_ReadInputDataBit(KEY_RIGHT_PORT, KEY_RIGHT_BIT))
	{
		if (bPrevious_key == RIGHT) 
		{
			return KEY_HOLD;
		}
		else
		{
			bPrevious_key = RIGHT;
			return RIGHT;
		}
	}
	/* "LEFT" key is pressed */
	else if(!GPIO_ReadInputDataBit(KEY_LEFT_PORT, KEY_LEFT_BIT))
	{
		if (bPrevious_key == LEFT) 
		{
		return KEY_HOLD;
		}
		else
		{
		bPrevious_key = LEFT;
		return LEFT;
		}
	}	
	/* "SEL" key is pressed */
	if(!GPIO_ReadInputDataBit(KEY_SEL_PORT, KEY_SEL_BIT))
	{
		if (bPrevious_key == SEL) 
		{
			return KEY_HOLD;
		}
		else
		{//1.防抖时间没到，并且key_Flag=sel,返回NOKEY
			if ( (TB_DebounceDelay_IsElapsed( ) == FALSE) && (bKey_Flag & SEL_FLAG == SEL_FLAG) )
			{
				return NOKEY;
			}
			else
			{//2.防抖延时到并且key_Flag=0，设置sel_Flag,设置防抖时间50ms
				if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ( (bKey_Flag & SEL_FLAG) == 0) ) 
				{
					bKey_Flag |= SEL_FLAG;
					TB_Set_DebounceDelay_500us(100); // 50 ms debounce
				}
				//3.防抖时间到，并且key_Flag=sel，重置sel_Flag=0,返回sel按键
				else if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ((bKey_Flag & SEL_FLAG) == SEL_FLAG) )
				{
					bKey_Flag &= (u8)(~SEL_FLAG);
					bPrevious_key = SEL;
					return SEL;
				}
				return NOKEY;
			}
		}
	}

	/* "UP" key is pressed */
	else if(!GPIO_ReadInputDataBit(KEY_UP_PORT, KEY_UP_BIT))
	{
		if (bPrevious_key == UP) 
		{
			return KEY_HOLD;
		}
		else
		{
			bPrevious_key = UP;
			return UP;
		}
	}
	/* "DOWN" key is pressed */
	else if(!GPIO_ReadInputDataBit(KEY_DOWN_PORT, KEY_DOWN_BIT))
	{
		if (bPrevious_key == DOWN) 
		{
			return KEY_HOLD;
		}
		else
		{
			bPrevious_key = DOWN;
			return DOWN;
		}
	}

	/* No key is pressed */
	else
	{
		bPrevious_key = NOKEY;
		return NOKEY;
	}
}



/*******************************************************************************
* Function Name  : KEYS_process
* Description    : Process key 
* Input          : Key code
* Output         : None
* Return         : None
*******************************************************************************/
void KEYS_process(void)
{
	bKey = KEYS_Read( );    // read key pushed (if any...)

	switch (bMenu_index)
	{     
		
		case(CONTROL_MODE_MENU_1):
			switch(bKey)
			{
				case UP:
				case DOWN:
					wGlobal_Flags ^= SPEED_CONTROL;
					bMenu_index = CONTROL_MODE_MENU_6;
				break;

				case RIGHT:
					bMenu_index = REF_SPEED_MENU;
				break;

				case LEFT:  
					bMenu_index=POWER_STAGE_MENU;					
				break;

				case SEL:
				if (State == RUN)
				{
				State = STOP;               
				}
				else if (State== START)
				{
				State = STOP; 
				}
				else if(State == IDLE)
				{
				State = INIT;
				bMenu_index = REF_SPEED_MENU;
				}  

				break;
				default:
				break;
			}
		break;

		case(REF_SPEED_MENU):
			switch(bKey)
			{
				case UP:
				if (hSpeed_Reference <= MOTOR_MAX_SPEED_RPM)
				{
					hSpeed_Reference += SPEED_INC_DEC;
				}
				break;

				case DOWN:
				if (hSpeed_Reference >= -MOTOR_MAX_SPEED_RPM)
				{
					hSpeed_Reference -= SPEED_INC_DEC;
				}
				break;

				case RIGHT:
					bMenu_index = P_SPEED_MENU;
				break;

				case LEFT:
				if (State == IDLE)
				{
					bMenu_index = CONTROL_MODE_MENU_1;
				}
				else
				{

				}              
				break;

				case SEL:
				if (State == RUN)
				{
					State = STOP;               
				}
				else if(State == START)
				{
					State = STOP;
				}              
				else if(State == IDLE)
				{
					State = INIT;
				}   
				break;
				
				default:
				break;
			}
		break;          

		case(P_SPEED_MENU):    
			switch(bKey)
			{
				case UP:
				if (PID_Speed_InitStructure.hKp_Gain <= S16_MAX-KP_GAIN_INC_DEC)
				{
				PID_Speed_InitStructure.hKp_Gain += KP_GAIN_INC_DEC;
				}
				break;

				case DOWN:
				if (PID_Speed_InitStructure.hKp_Gain >= KP_GAIN_INC_DEC)
				{
				PID_Speed_InitStructure.hKp_Gain -= KP_GAIN_INC_DEC;
				}
				break;

				case RIGHT:
				bMenu_index = I_SPEED_MENU;
				break;

				case LEFT:
				bMenu_index = REF_SPEED_MENU;             
				break;

				case SEL:
				if (State == RUN)
				{
					State = STOP;               
				}
				else if (State== START)
				{
					State = STOP; 
				}
				else if(State == IDLE)
				{
					State = INIT;
				}   
				break;
				default:
				break;
			}
		break;

		case(I_SPEED_MENU):    
			switch(bKey)
			{
				case UP:
				if (PID_Speed_InitStructure.hKi_Gain <= S16_MAX-KI_GAIN_INC_DEC)
				{
				PID_Speed_InitStructure.hKi_Gain += KI_GAIN_INC_DEC;
				}
				break;

				case DOWN:
				if (PID_Speed_InitStructure.hKi_Gain >= KI_GAIN_INC_DEC)
				{
				PID_Speed_InitStructure.hKi_Gain -= KI_GAIN_INC_DEC;
				}
				break;

				case RIGHT: 
				bMenu_index = P_TORQUE_MENU;
				break;

				case LEFT:
				bMenu_index = P_SPEED_MENU;
				break;

				case SEL:
				if (State == RUN)
				{
				State = STOP;               
				}
				else if (State== START)
				{
				State = STOP; 
				}
				else if(State == IDLE)
				{
				State = INIT;
				}   
				break;
				default:
				break;
			}
		break;        


		case(P_TORQUE_MENU):    
			switch(bKey)
			{
				case UP:
				if (PID_Torque_InitStructure.hKp_Gain <= S16_MAX - KP_GAIN_INC_DEC)
				{
					PID_Torque_InitStructure.hKp_Gain += KP_GAIN_INC_DEC;
				}
				break;

				case DOWN:
				if (PID_Torque_InitStructure.hKp_Gain >= KP_GAIN_INC_DEC)
				{
					PID_Torque_InitStructure.hKp_Gain -= KP_GAIN_INC_DEC;
				}
				break;

				case RIGHT:
					bMenu_index = I_TORQUE_MENU;
				break;

				case LEFT:         
				if ((wGlobal_Flags & SPEED_CONTROL) == SPEED_CONTROL)
				{
					bMenu_index = I_SPEED_MENU; 
				}
				else
				{
					bMenu_index = ID_REF_MENU;
				}              
				break;

				case SEL:
				if (State == RUN)
				{
					State = STOP;               
				}
				else if (State== START)
				{
					State = STOP; 
				}
				else if(State == IDLE)
				{
					State = INIT;
				}   
				break;
				default:
				break;
			}
		break;

		case(I_TORQUE_MENU):    
			switch(bKey)
			{
			case UP:
			if (PID_Torque_InitStructure.hKi_Gain <= S16_MAX - KI_GAIN_INC_DEC)
			{
			PID_Torque_InitStructure.hKi_Gain += KI_GAIN_INC_DEC;
			}
			break;

			case DOWN:
			if (PID_Torque_InitStructure.hKi_Gain >= KI_GAIN_INC_DEC)
			{
			PID_Torque_InitStructure.hKi_Gain -= KI_GAIN_INC_DEC;
			}
			break;

			case RIGHT:
				bMenu_index = P_FLUX_MENU;
			break;

			case LEFT:
				bMenu_index = P_TORQUE_MENU;
			break;

			case SEL:
			if (State == RUN)
			{
				State = STOP;               
			}
			else if (State== START)
			{
				State = STOP; 
			}
			else if(State == IDLE)
			{
				State = INIT;
			}   
			break;
			
			default:
			break;
			}
		break;

		case(P_FLUX_MENU):    
			switch(bKey)
			{
				case UP:
					if (PID_Flux_InitStructure.hKp_Gain <= S16_MAX-KP_GAIN_INC_DEC)
					{
						PID_Flux_InitStructure.hKp_Gain += KP_GAIN_INC_DEC;
					}
				break;

				case DOWN:
					if (PID_Flux_InitStructure.hKp_Gain >= KP_GAIN_INC_DEC)
					{
						PID_Flux_InitStructure.hKp_Gain -= KP_GAIN_INC_DEC;
					}
				break;

				case RIGHT:
					bMenu_index = I_FLUX_MENU;
				break;

				case LEFT: 
					bMenu_index=I_TORQUE_MENU;					
				break;

				case SEL:
				if (State == RUN)
				{
					State = STOP;               
				}
				else if (State== START)
				{
					State = STOP; 
				}
				else if(State == IDLE)
				{
					State = INIT;
				}   
				break;
				default:
				break;
			}
		break;

		case(I_FLUX_MENU):    
			switch(bKey)
			{
				case UP:
					if (PID_Flux_InitStructure.hKi_Gain <= S16_MAX-KI_GAIN_INC_DEC)
					{
						PID_Flux_InitStructure.hKi_Gain += KI_GAIN_INC_DEC;
					}
				break;

				case DOWN:
					if (PID_Flux_InitStructure.hKi_Gain >= KI_GAIN_INC_DEC)
					{
						PID_Flux_InitStructure.hKi_Gain -= KI_GAIN_INC_DEC;
					}
				break;

				case RIGHT:
				bMenu_index = POWER_STAGE_MENU;

				break;

				case LEFT:
				bMenu_index = P_FLUX_MENU;
				break;

				case SEL:
				if (State == RUN)
				{
					State = STOP;               
				}
				else if (State== START)
				{
					State = STOP; 
				}
				else if(State == IDLE)
				{
					State = INIT;
				}   
				break;
				
				default:
				break;
			}
		break;


		case(POWER_STAGE_MENU):
			switch(bKey)
			{
				case RIGHT:  
				bMenu_index = CONTROL_MODE_MENU_1;
				break;

				case LEFT:
					bMenu_index = I_FLUX_MENU;                                      
				break;

				case SEL:
					if (State == RUN)
					{
						State = STOP;               
					}
					else if (State== START)
					{
						State = STOP; 
					}
					else if(State == IDLE)
					{
						State = INIT;
					}   
				break;
					
				default:
				break;
			}
		break; 

		case(CONTROL_MODE_MENU_6):
			switch(bKey)
			{
				case UP:
				case DOWN:
					wGlobal_Flags ^= SPEED_CONTROL;
					bMenu_index = CONTROL_MODE_MENU_1;
				break;

				case RIGHT:
					bMenu_index = IQ_REF_MENU;
				break;

				case LEFT:

					bMenu_index = POWER_STAGE_MENU;

				break;

				case SEL:
				if (State == RUN)
				{
				State = STOP;               
				}
				else if (State== START)
				{
				State = STOP; 
				}
				else if(State == IDLE)
				{
					State = INIT;
					bMenu_index = IQ_REF_MENU;
				}   
				break;
				default:
				break;
			}
		break;  

		case(IQ_REF_MENU):
			switch(bKey)
			{
				case UP:
				if (hTorque_Reference <= NOMINAL_CURRENT - TORQUE_INC_DEC)
				{
					hTorque_Reference += TORQUE_INC_DEC;
				}
				break;

				case DOWN:
					if (hTorque_Reference >= -NOMINAL_CURRENT + TORQUE_INC_DEC)
					{
						hTorque_Reference -= TORQUE_INC_DEC;
					}
				break;

				case RIGHT:
					bMenu_index = ID_REF_MENU;
				break;

				case LEFT:
					if(State == IDLE)
					{
						bMenu_index = CONTROL_MODE_MENU_6;
					}
					else
					{
						bMenu_index = POWER_STAGE_MENU;
					}
				break;

				case SEL:
					if (State == RUN)
					{
						State = STOP;               
					}
					else if (State== START)
					{
						State = STOP; 
					}
					else if(State == IDLE)
					{
						State = INIT;
					}   
				break;
					
				default:
				break;
			}
		break;    

		case(ID_REF_MENU):
			switch(bKey)
			{
				case UP:
					if (hFlux_Reference <= NOMINAL_CURRENT - FLUX_INC_DEC)
					{
						hFlux_Reference += FLUX_INC_DEC;
					}
				break;

				case DOWN:
					if (hFlux_Reference >= FLUX_INC_DEC - NOMINAL_CURRENT)
					{
						hFlux_Reference -= FLUX_INC_DEC;
					}
				break;

				case RIGHT:
					bMenu_index = P_TORQUE_MENU;
				break;

				case LEFT:
					bMenu_index = IQ_REF_MENU;
				break;

				case SEL:
				if (State == RUN)
				{
					State = STOP;               
				}
				else if (State== START)
				{
					State = STOP; 
				}
				else if(State == IDLE)
				{
					State = INIT;
				}   
				break;
				
				default:
				break;
			}
		break;

		default:
		break; 
	}
}

/*******************************************************************************
* Function Name  : KEYS_ExportbKey
* Description    : Export bKey variable
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 KEYS_ExportbKey(void)
{
  return(bKey);
}
                   
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

