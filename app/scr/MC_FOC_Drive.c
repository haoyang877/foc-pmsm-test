/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_FOC_Drive.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file provides all the PMSM FOC drive functions.
* 
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
* 28/08/08 v2.0.2
* 04/09/08 v2.0.3
********************************************************************************
*@brife 此文件主要是完成FOC的运算时序，可以配合高级运算规则优化FOC运算。
		1.FOC_Model运算是完成FOC运算时序
		获取Ia、Ib电流值，Clarke变换得到Iapha、Ibeta值，Park变化得到Iq、Id值，
		和参考值比较进行PID运算得到Vq、Vd值，Vq、Vd值进行限幅运算，得到Vq*、Vd*，
		反Park变换得到Valpha、Vbeta，最后经过SVPWM计算得到各个相的占空比和ADC触发
		转换点，等待下一次更新事件到来。
		2.通过速度计算力矩参考，或者更新力矩和磁通的参考都可以在这里实现
		3.以后通过位置计算速度参考也在这里实现。
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "MC_const.h"
#include "MC_FOC_Drive.h"
#include "MC_PMSM_motor_param.h"


#define SATURATION_TO_S16(a)    if (a > S16_MAX)              \
                                {                             \
                                  a = S16_MAX;                \
                                }                             \
                                else if (a < -S16_MAX)        \
                                {                             \
                                  a = -S16_MAX;               \
                                }                             \
/* Private functions ---------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/
static volatile Curr_Components Stat_Curr_q_d_ref;
static Curr_Components Stat_Curr_q_d_ref_ref;
								

/*******************************************************************************
* Function Name : FOC_Init
* Description   : The purpose of this function is to initialize to proper values
*                 all the variables related to the field-oriented control
*                 algorithm. To be called once prior to every motor startup.
* Input         : None.
* Output        : None.
* Return        : None.
*******************************************************************************/
void FOC_Init (void)
{
  Stat_Curr_q_d_ref_ref.qI_Component1 = 0;
  Stat_Curr_q_d_ref_ref.qI_Component2 = 0;  
  
  Stat_Curr_q_d_ref.qI_Component1 = 0;
  Stat_Curr_q_d_ref.qI_Component2 = 0;
}

/*******************************************************************************
* Function Name : FOC_Model
* Description   : The purpose of this function is to perform PMSM torque and 
*                 flux regulation, implementing the FOC vector algorithm.
* Input         : None.
* Output        : None.
* Return        : None.
*******************************************************************************/
void FOC_Model(void)
{	
  Stat_Curr_a_b =SVPWM_3ShuntGetPhaseCurrentValues();
  
  Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);
  
  Stat_Curr_q_d = Park(Stat_Curr_alfa_beta,GET_ELECTRICAL_ANGLE);  

  /*loads the Torque Regulator output reference voltage Vqs*/   
  Stat_Volt_q_d.qV_Component1 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component1, 
                        Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);
  /*loads the Flux Regulator output reference voltage Vds*/
  Stat_Volt_q_d.qV_Component2 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component2, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure);  

  RevPark_Circle_Limitation( );
 
  Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);

  /*Valpha and Vbeta finally drive the power stage*/ 
  SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta); 
}

/*******************************************************************************
* Function Name   : FOC_CalcFluxTorqueRef
* Description     : This function provides current components Iqs* and Ids* to be
*                   used as reference values (by the FOC_Model function) when in
*                   speed control mode
* Input           : None.
* Output          : None.
* Return          : None.
*******************************************************************************/
void FOC_CalcFluxTorqueRef(void)
{
	//PID 速度调节单位是RPM
	Stat_Curr_q_d_ref.qI_Component1 = PID_Regulator(hSpeed_Reference,GET_SPEED_0_1HZ*6,&PID_Speed_InitStructure);
	Stat_Curr_q_d_ref.qI_Component2 = 0;

	//此处是Stat_Curr_q_d_ref_ref是经过算法处理之后的电流参考值，之后往此处添加算法
	//然而Stat_Curr_q_d_ref 是直接经过速度PID计算之后的电流参考值
	Stat_Curr_q_d_ref_ref = Stat_Curr_q_d_ref;

	hTorque_Reference = Stat_Curr_q_d_ref_ref.qI_Component1;
	hFlux_Reference = Stat_Curr_q_d_ref_ref.qI_Component2;  
}


/*******************************************************************************
* Function Name   : FOC_TorqueCntrl
* Description     : This function provides current components Iqs* and Ids* to be
*                   used as reference values (by the FOC_Model function) when in
*                   Torque control mode
* Input           : None.
* Output          : None.
* Return          : None.
*******************************************************************************/
void FOC_TorqueCtrl(void)
{
	Stat_Curr_q_d_ref_ref.qI_Component1 = hTorque_Reference;
	Stat_Curr_q_d_ref_ref.qI_Component2 = hFlux_Reference;
}




/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
