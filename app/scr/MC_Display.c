/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Display.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file contains the software implementation of the
*                      display routines
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
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
/* Standard include ----------------------------------------------------------*/
#include "stm32f10x_conf.h"

/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/

#include "stm32f10x_MClib.h"
#include "MC_Display.h"
#include "MC_Globals.h"

extern u8 bMenu_index;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLINKING_TIME   5  // 5 * timebase_display_5 ms ��ʾʱ��

#define VISUALIZATION_1   (u8)1
#define VISUALIZATION_2   (u8)2
#define VISUALIZATION_3   (u8)3
#define VISUALIZATION_4   (u8)4
#define VISUALIZATION_5   (u8)5
#define VISUALIZATION_6   (u8)6
#define VISUALIZATION_7   (u8)7
#define VISUALIZATION_8   (u8)8
#define VISUALIZATION_9   (u8)9
#define VISUALIZATION_10  (u8)10

#define CHAR_0            (u8)0 //First character of the line starting from the left
#define CHAR_1            (u8)1 
#define CHAR_2            (u8)2
#define CHAR_3            (u8)3
#define CHAR_4            (u8)4
#define CHAR_5            (u8)5
#define CHAR_6            (u8)6
#define CHAR_7            (u8)7
#define CHAR_8            (u8)8
#define CHAR_9            (u8)9
#define CHAR_10           (u8)10
#define CHAR_11           (u8)11
#define CHAR_12           (u8)12
#define CHAR_13           (u8)13
#define CHAR_14           (u8)14
#define CHAR_15           (u8)15
#define CHAR_16           (u8)16
#define CHAR_17           (u8)17


/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Display_5DigitSignedNumber(u8, u8, s16);
u8 ComputeVisualization(u8 );

/* Private variables ---------------------------------------------------------*/
volatile static u16 hTimebase_Blinking;
static u8 bPrevious_Visualization = 0;
static u8 bPresent_Visualization;


/*******************************************************************************
* Function Name  : Display_Welcome_Message
* Description    : Welcome message on LCD after power-up
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Display_Welcome_Message(void)
{
  u8 *ptr = " STM32 Motor Control";
  
  LCD_DisplayStringLine(Line0, ptr);
  
  ptr = "  PMSM FOC ver 2.0  ";
  LCD_DisplayStringLine(Line1, ptr);
    
  ptr = " <> Move  ^| Change ";
  LCD_DisplayStringLine(Line9, ptr);  
  
  
  //Display_5DigitSignedNumber(Line4*24,2,12345);          
}  

/*******************************************************************************
* Function Name  : Display_LCD
* Description    : Display routine for LCD management
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Display_LCD(void)
{          
  if (TB_DisplayDelay_IsElapsed() == TRUE) 
  { 
    TB_Set_DisplayDelay_500us(500);  //  refresh LCD every 400*5 = 200 ms

    bPrevious_Visualization = bPresent_Visualization;

    bPresent_Visualization = ComputeVisualization(bMenu_index);
  
    switch(bPresent_Visualization)
    {
      u8 *ptr;
      s16 temp;
            
      case(VISUALIZATION_1):
        if (bPresent_Visualization != bPrevious_Visualization)
        { 
         
          LCD_ClearLine(Line3); 
          
          LCD_ClearLine(Line4); 
                 
          ptr = " Target     Measured";
          LCD_DisplayStringLine(Line5,ptr); 
          
          ptr = "       (rpm)        ";
          LCD_DisplayStringLine(Line7,ptr); 
          
          LCD_ClearLine(Line6);        
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        if(bMenu_index == CONTROL_MODE_MENU_1)
        {
          LCD_SetTextColor(Red);
        }        
        
        ptr = " Speed control mode";        
        LCD_DisplayStringLine(Line3,ptr);
        
        if(bMenu_index == CONTROL_MODE_MENU_1)
        {
          LCD_SetTextColor(Blue);
        }
        else //REF_SPEED_MENU
        {
          LCD_SetTextColor(Red);
        }
          
        //Compute target speed in rpm
        temp = (s16)(hSpeed_Reference);                 
        Display_5DigitSignedNumber(Line7, CHAR_0, temp);
        
        if(bMenu_index != CONTROL_MODE_MENU_1)
        {
          LCD_SetTextColor(Blue);
        }
         
        //Compute measured speed in rpm
	
		temp = (s16)(ENC_Get_Mechanical_Speed( )*6);          
        Display_5DigitSignedNumber(Line7, CHAR_13, temp); 
      
      break;
          
      case(VISUALIZATION_2):
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "       Speed        ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "    P     I     D   ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
          LCD_ClearLine(Line5);
           
          ptr = " Target        (rpm)";
          LCD_DisplayStringLine(Line6,ptr); 
          
          ptr = " Measured      (rpm)";
          LCD_DisplayStringLine(Line7,ptr);
                    
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(P_SPEED_MENU):
            LCD_SetTextColor(Red);            
            temp = PID_Speed_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            LCD_SetTextColor(Blue);
            
            temp = PID_Speed_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            
			#ifdef DIFFERENTIAL_TERM_ENABLED            
						temp = PID_Speed_InitStructure.hKd_Gain;
						Display_5DigitSignedNumber(Line4, CHAR_13, temp);
			#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {						  // 320 -
                LCD_DisplayChar(Line4*24, (u16)((16*(18-i))),'-');
              }
            }
			#endif         
 
         break;
            
          case(I_SPEED_MENU):                                 
            temp = PID_Speed_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            
            LCD_SetTextColor(Red);   
            temp = PID_Speed_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            LCD_SetTextColor(Blue);
            
			#ifdef DIFFERENTIAL_TERM_ENABLED            
						temp = PID_Speed_InitStructure.hKd_Gain;
						Display_5DigitSignedNumber(Line4, CHAR_13, temp);
			#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {					   // 320 -
                LCD_DisplayChar(Line4*24, (u16)((16*(18-i))),'-');
              }
            }
			#endif	
	      break;
          
			#ifdef DIFFERENTIAL_TERM_ENABLED
            case(D_SPEED_MENU):
              temp = PID_Speed_InitStructure.hKp_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_1, temp);
              
              temp = PID_Speed_InitStructure.hKi_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_7, temp);
              
              LCD_SetTextColor(Red);
              temp = PID_Speed_InitStructure.hKd_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_13, temp);
              LCD_SetTextColor(Blue);
          
            break;
			#endif
			
        default:
          break;
        }
        //Independently from the menu, this visualization must display current 
        //and measured speeds
        
        //Display target speed in rpm
        temp = (s16)(hSpeed_Reference);          
        Display_5DigitSignedNumber(Line6, CHAR_9, temp);
        
        //Compute measured speed in rpm
		#ifdef ENCODER
				temp = (s16)(ENC_Get_Mechanical_Speed() * 6);

		#endif
        Display_5DigitSignedNumber(Line7, CHAR_9, temp);         
      break;
      
      case(VISUALIZATION_3):
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "       Torque       ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "    P     I     D   ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
          LCD_ClearLine(Line5);
           
          ptr = " Target         (Iq)";
          LCD_DisplayStringLine(Line6,ptr); 
          
          ptr = " Measured       (Iq)";
          LCD_DisplayStringLine(Line7,ptr);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(P_TORQUE_MENU):
            LCD_SetTextColor(Red);            
            temp = PID_Torque_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            LCD_SetTextColor(Blue);
            
            temp = PID_Torque_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            
			#ifdef DIFFERENTIAL_TERM_ENABLED            
						temp = PID_Torque_InitStructure.hKd_Gain;
						Display_5DigitSignedNumber(Line4, CHAR_13, temp);
			#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {								 //320 -
                LCD_DisplayChar(Line4*24, (u16)((16*(18-i))),'-');
              }
            }
			#endif       
         break;
            
          case(I_TORQUE_MENU):                                 
            temp = PID_Torque_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            
            LCD_SetTextColor(Red);   
            temp = PID_Torque_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            LCD_SetTextColor(Blue);
            
			#ifdef DIFFERENTIAL_TERM_ENABLED             
						temp = PID_Torque_InitStructure.hKd_Gain;
						Display_5DigitSignedNumber(Line4, CHAR_13, temp);
			#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {								  //320 -
                LCD_DisplayChar(Line4*24, (u16)((16*(18-i))),'-');
              }
            }
			#endif
           break;
          
			#ifdef DIFFERENTIAL_TERM_ENABLED 
            case(D_TORQUE_MENU):
              temp = PID_Torque_InitStructure.hKp_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_1, temp);
              
              temp = PID_Torque_InitStructure.hKi_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_7, temp);
              
              LCD_SetTextColor(Red);
              temp = PID_Torque_InitStructure.hKd_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_13, temp);
              LCD_SetTextColor(Blue);
              
            break;
			#endif
        default:
          break;
        }
        //Independently from the menu, this visualization must display current 
        //and measured Iq
        
        temp = hTorque_Reference;          
        Display_5DigitSignedNumber(Line6, CHAR_9, temp);

        temp = Stat_Curr_q_d.qI_Component1;
        Display_5DigitSignedNumber(Line7, CHAR_9, temp);        
      break;
     
       case(VISUALIZATION_4):
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "        Flux        ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "    P     I     D   ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
          LCD_ClearLine(Line5);
           
          ptr = " Target         (Id)";
          LCD_DisplayStringLine(Line6,ptr); 
          
          ptr = " Measured       (Id)";
          LCD_DisplayStringLine(Line7,ptr);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(P_FLUX_MENU):
            LCD_SetTextColor(Red);            
            temp = PID_Flux_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            LCD_SetTextColor(Blue);
            
            temp = PID_Flux_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            
			#ifdef DIFFERENTIAL_TERM_ENABLED            
						temp = PID_Flux_InitStructure.hKd_Gain;
						Display_5DigitSignedNumber(Line4, CHAR_13, temp);
			#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {								 //	320 -
                LCD_DisplayChar(Line4*24, (u16)((16*(18-i))),'-');
              }
            }
			#endif       
          break;
            
          case(I_FLUX_MENU):                                 
            temp = PID_Flux_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            
            LCD_SetTextColor(Red);   
            temp = PID_Flux_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            LCD_SetTextColor(Blue);
            
			#ifdef DIFFERENTIAL_TERM_ENABLED             
						temp = PID_Flux_InitStructure.hKd_Gain;
						Display_5DigitSignedNumber(Line4, CHAR_13, temp);
			#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {									 //320 -
                LCD_DisplayChar(Line4*24, (u16)((16*(18-i))),'-');
              }
            }
			#endif
           break;
            
#ifdef DIFFERENTIAL_TERM_ENABLED 
            case(D_FLUX_MENU):
              temp = PID_Flux_InitStructure.hKp_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_1, temp);
              
              temp = PID_Flux_InitStructure.hKi_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_7, temp);
              
              LCD_SetTextColor(Red);
              temp = PID_Flux_InitStructure.hKd_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_13, temp);
              LCD_SetTextColor(Blue);
              
            break;
#endif
        default:
          break;
        }
        //Independently from the menu, this visualization must display current 
        //and measured Id
        
        temp = hFlux_Reference;          
        Display_5DigitSignedNumber(Line6, CHAR_9, temp);

        temp = Stat_Curr_q_d.qI_Component2;
        Display_5DigitSignedNumber(Line7, CHAR_9, temp);   
      break;

      
      case(VISUALIZATION_5):
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          LCD_ClearLine(Line2);
          
          ptr = " Power Stage Status ";          
          LCD_DisplayStringLine(Line3, ptr); 
          
          LCD_ClearLine(Line4);
          
          ptr = "  DC bus =     Volt ";          
          LCD_DisplayStringLine(Line5, ptr); 
          
          LCD_ClearLine(Line6);
          
          ptr = "  T =      Celsius  "; //���϶�         
          LCD_DisplayStringLine(Line7, ptr); 
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move            ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
      
        temp = MCL_Compute_BusVolt();  //320-      
        LCD_DisplayChar(Line5*24, 16*CHAR_11, (u8)(((temp%1000)/100)+0x30));
        LCD_DisplayChar(Line5*24, 16*CHAR_12, (u8)(((temp%100)/10)+0x30));
        LCD_DisplayChar(Line5*24, 16*CHAR_13, (u8)((temp%10)+0x30));
        
        temp = MCL_Compute_Temp(); 
        LCD_DisplayChar(Line7*24, 16*CHAR_6, (u8)(((temp%1000)/100)+0x30));
        LCD_DisplayChar(Line7*24, 16*CHAR_7, (u8)(((temp%100)/10)+0x30));
        LCD_DisplayChar(Line7*24, 16*CHAR_8, (u8)((temp%10)+0x30));
      
      break;    
        
      case(VISUALIZATION_6):
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
           
          LCD_ClearLine(Line3); 
          
          ptr = "     Target Measured";
          LCD_DisplayStringLine(Line4,ptr);
          
          ptr = "Iq                  ";
          LCD_DisplayStringLine(Line5,ptr); 
          
          ptr = "Id                  ";
          LCD_DisplayStringLine(Line6,ptr);
          
          ptr = "Speed (rpm)         ";
          LCD_DisplayStringLine(Line7,ptr);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(CONTROL_MODE_MENU_6):
            LCD_SetTextColor(Red);
            ptr = "Torque control mode ";        
            LCD_DisplayStringLine(Line3,ptr);  
            LCD_SetTextColor(Blue);
            
            temp = hTorque_Reference;
            Display_5DigitSignedNumber(Line5, CHAR_5, temp);
 
            temp = hFlux_Reference; 
            Display_5DigitSignedNumber(Line6, CHAR_5, temp);         
          break;   
          
          case(IQ_REF_MENU):
            ptr = "Torque control mode ";
            LCD_DisplayStringLine(Line3,ptr); 
            
            LCD_SetTextColor(Red);
            temp = hTorque_Reference;
            Display_5DigitSignedNumber(Line5, CHAR_5, temp);
            LCD_SetTextColor(Blue);
            
            temp = hFlux_Reference; 
            Display_5DigitSignedNumber(Line6, CHAR_5, temp); 
          break;
            
          case(ID_REF_MENU):
            ptr = "Torque control mode ";
            LCD_DisplayStringLine(Line3,ptr); 
            
            temp = hTorque_Reference;
            Display_5DigitSignedNumber(Line5, CHAR_5, temp);
            
            LCD_SetTextColor(Red);
            temp = hFlux_Reference; 
            Display_5DigitSignedNumber(Line6, CHAR_5, temp); 
            LCD_SetTextColor(Blue);
          break;
         default: 
          break;
        }            
        temp =Stat_Curr_q_d.qI_Component1;
        Display_5DigitSignedNumber(Line5, CHAR_13, temp);
        
        temp =Stat_Curr_q_d.qI_Component2;        
        Display_5DigitSignedNumber(Line6, CHAR_13, temp);

        //Compute measured speed in rpm
		#ifdef ENCODER
				temp = (s16)(ENC_Get_Mechanical_Speed() * 6);

		#endif 
        Display_5DigitSignedNumber(Line7, CHAR_13, temp);
      break;
        
      case(VISUALIZATION_7):
        if (bPresent_Visualization != bPrevious_Visualization)
        {  
          LCD_ClearLine(Line2);
          
          LCD_SetTextColor(Red);
          ptr = "    !!! FAULT !!!   ";
          LCD_DisplayStringLine(Line3,ptr);
          LCD_SetTextColor(Blue);
         
          if ( (wGlobal_Flags & UNDER_VOLTAGE) == UNDER_VOLTAGE)
          {           
            ptr = " Bus Under Voltage  ";
            LCD_DisplayStringLine(Line4, ptr);                                   
          }
          else if ( (wGlobal_Flags & OVER_CURRENT) ==  OVER_CURRENT)
            {
              ptr = "   Over Current    ";
              LCD_DisplayStringLine(Line4, ptr); 
            }
          else if ( (wGlobal_Flags & OVERHEAT) ==  OVERHEAT)
            {
              ptr = "   Over Heating    ";
              LCD_DisplayStringLine(Line4, ptr);                             
            }
          else if ( (wGlobal_Flags & OVER_VOLTAGE) ==  OVER_VOLTAGE)
            {
              ptr = "  Bus Over Voltage  ";
              LCD_DisplayStringLine(Line4, ptr);               
            }
          else if ( (wGlobal_Flags & START_UP_FAILURE) ==  START_UP_FAILURE)
          {
             ptr = "  Start-up failed   ";
             LCD_DisplayStringLine(Line4, ptr);    
          }      
          else if ( (wGlobal_Flags & SPEED_FEEDBACK) ==  SPEED_FEEDBACK)
          {
             ptr = "Error on speed fdbck";
             LCD_DisplayStringLine(Line4, ptr);     
          }  
          LCD_ClearLine(Line5);
          LCD_ClearLine(Line7);  
        } 

        if ((wGlobal_Flags & ( OVERHEAT | UNDER_VOLTAGE | OVER_VOLTAGE)) == 0) 
        { 
          LCD_ClearLine(Line6);
          ptr = "   Press 'Key' to   ";
          LCD_DisplayStringLine(Line8,ptr);
          
          ptr = "   return to menu   ";
          LCD_DisplayStringLine(Line9,ptr);
        }
        else
        {
          if ((wGlobal_Flags & (UNDER_VOLTAGE | OVER_VOLTAGE)) ==0)    
          {//Under or over voltage
             if (bPresent_Visualization != bPrevious_Visualization)
             { 
               LCD_ClearLine(Line6);
             }
             temp = MCL_Compute_Temp(); 
             ptr = "       T =";  
             LCD_DisplayStringLine(Line6, ptr);	//320-
             LCD_DisplayChar(Line6*24, 16*CHAR_11, (u8)(((temp%1000)/100)+0x30));
             LCD_DisplayChar(Line6*24, 16*CHAR_12, (u8)(((temp%100)/10)+0x30));
             LCD_DisplayChar(Line6*24, 16*CHAR_13, (u8)((temp%10)+0x30));
             LCD_DisplayChar(Line6*24, 16*CHAR_14, ' ');
             LCD_DisplayChar(Line6*24, 16*CHAR_15, 'C');
          }
          else 
          {           
            if (bPresent_Visualization != bPrevious_Visualization)
            { 
              LCD_ClearLine(Line6);         
            }
            ptr = "  DC bus =";            
            LCD_DisplayStringLine(Line6, ptr); 
            temp = MCL_Compute_BusVolt();   // 320-    
            LCD_DisplayChar(Line6*24, 16*CHAR_11, (u8)(((temp%1000)/100)+0x30));
            LCD_DisplayChar(Line6*24, 16*CHAR_12, (u8)(((temp%100)/10)+0x30));
            LCD_DisplayChar(Line6*24, 16*CHAR_13, (u8)((temp%10)+0x30)); 
            LCD_DisplayChar(Line6*24, 16*CHAR_14, ' ');
            LCD_DisplayChar(Line6*24, 16*CHAR_15, 'V');             
          }          
          LCD_ClearLine(Line8);
          LCD_ClearLine(Line9);
        }
      break;
     
      case(VISUALIZATION_8):  
        if (bPresent_Visualization != bPrevious_Visualization)
        {  
          LCD_ClearLine(Line2);
          
          ptr = " Motor is stopping  ";
          LCD_DisplayStringLine(Line3,ptr);
          
          ptr = "   please wait...   ";
          LCD_DisplayStringLine(Line4,ptr);
          
          LCD_ClearLine(Line5);
          LCD_ClearLine(Line6);
          LCD_ClearLine(Line7);
          LCD_ClearLine(Line8);
          LCD_ClearLine(Line9);
        } 
      break;
		
    default:
      break;      
    }
  }
}
          
/*******************************************************************************
* Function Name  : Display_5DigitSignedNumber
* Description    : It Displays a 5 digit signed number in the specified line, 
*                  starting from a specified element of LCD display matrix 
* Input          : Line, starting point in LCD dysplay matrix, 5 digit signed
*                  number 
* Output         : None
* Return         : None
*******************************************************************************/

void Display_5DigitSignedNumber(u8 Line, u8 bFirstchar, s16 number)
{ u32 i;
  u16 h_aux=1;

  if (number<0)     
  {								 //320-
    LCD_DisplayChar(Line*24,(u16)( 16*bFirstchar), '-');
    number = -number;
  }
  else 
  {								//320-
    LCD_DisplayChar(Line*24,(u16)( 16*bFirstchar), ' ');
  }
      
  for (i=0; i<4; i++)
  {										// *24 320 -
    
    LCD_DisplayChar(Line*24, (u16)((16*(bFirstchar+5-i))),
                                        (u8)(((number%(10*h_aux))/h_aux)+0x30));          
    h_aux *= 10;
  }							 //320-
  LCD_DisplayChar(Line*24,(u16)((16*(bFirstchar+1))), (u8)(((number/10000))+0x30));
}    

/*******************************************************************************
* Function Name  : ComputeVisualization
* Description    : Starting from the value of the bMenuIndex, this function 
*                  extract the information about the present menu to be 
*                  displayed on LCD
* Input          : bMenuIndex variable 
* Output         : Present visualization
* Return         : None
*******************************************************************************/

u8 ComputeVisualization(u8 bLocal_MenuIndex)
{  
  u8 bTemp;

    switch(bLocal_MenuIndex)
    {
      case(CONTROL_MODE_MENU_1):
        bTemp = VISUALIZATION_1;
      break;
      case(REF_SPEED_MENU):
        bTemp = VISUALIZATION_1;
      break;
      
      case(P_SPEED_MENU):
        bTemp = VISUALIZATION_2; 
      break;
      case(I_SPEED_MENU):
        bTemp = VISUALIZATION_2; 
      break;
	#ifdef DIFFERENTIAL_TERM_ENABLED
      case(D_SPEED_MENU):
       bTemp = VISUALIZATION_2; 
      break;
	#endif        

      case(P_TORQUE_MENU):
        bTemp = VISUALIZATION_3; 
      break; 
      case(I_TORQUE_MENU):
        bTemp = VISUALIZATION_3; 
      break; 
	 #ifdef DIFFERENTIAL_TERM_ENABLED
      case(D_TORQUE_MENU):
        bTemp = VISUALIZATION_3; 
      break; 
	#endif        
         
      case(P_FLUX_MENU):
         bTemp = VISUALIZATION_4; 
      break; 
      case(I_FLUX_MENU):
         bTemp = VISUALIZATION_4; 
      break; 
	#ifdef DIFFERENTIAL_TERM_ENABLED
      case(D_FLUX_MENU):
         bTemp = VISUALIZATION_4; 
      break; 
	#endif
           
             
      case(POWER_STAGE_MENU):
        bTemp = VISUALIZATION_5;
      break;
        
      case(CONTROL_MODE_MENU_6):
        bTemp = VISUALIZATION_6;
      break;
      case(IQ_REF_MENU):
        bTemp = VISUALIZATION_6;
      break;
      case(ID_REF_MENU):
        bTemp = VISUALIZATION_6;
      break;  
      
      case(FAULT_MENU):
        bTemp = VISUALIZATION_7;
      break;      
                 
      default:
        bTemp = VISUALIZATION_1;
      break;      
    }    
      
    if (State == WAIT)
    {
      bTemp = VISUALIZATION_8;
    }  
    
    return (bTemp);
}
void LCD_Display_init()  
{
	STM3210C_LCD_Init();
	LCD_Clear(White);
	LCD_SetTextColor(Blue);
	LCD_SetBackColor(White); 
	Display_Welcome_Message(); 
}


/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
