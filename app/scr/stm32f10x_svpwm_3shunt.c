/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_3shunt.c
* Author             : IMS Systems Lab
* Date First Issued  : 28/11/07
* Description        : 3 shunt resistors current reading module
********************************************************************************
* History:
* 28/11/07 v1.0
* 29/05/08 v2.0
* 03/07/08 v2.0.1
* 09/07/08 v2.0.2
* 11/07/08 v2.0.3
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

#define   THREE_SHUNT

/* Includes-------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x_svpwm_3shunt.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


#define NB_CONVERSIONS 16

#define SQRT_3		1.732051
#define T		    (PWM_PERIOD * 4)
#define T_SQRT3     (u16)(T * SQRT_3)

#define SECTOR_1	(u32)1
#define SECTOR_2	(u32)2
#define SECTOR_3	(u32)3
#define SECTOR_4	(u32)4
#define SECTOR_5	(u32)5
#define SECTOR_6	(u32)6

#define PHASE_A_ADC_CHANNEL     ADC_Channel_11
#define PHASE_B_ADC_CHANNEL     ADC_Channel_12
#define PHASE_C_ADC_CHANNEL     ADC_Channel_13

// Setting for sampling of VBUS and Temp after currents sampling
#define PHASE_A_MSK       (u32)((u32)(PHASE_A_ADC_CHANNEL) << 10)
#define PHASE_B_MSK       (u32)((u32)(PHASE_B_ADC_CHANNEL) << 10)
#define PHASE_C_MSK       (u32)((u32)(PHASE_C_ADC_CHANNEL) << 10)

// Settings for current sampling only
/*#define PHASE_A_MSK       (u32)((u32)(PHASE_A_ADC_CHANNEL) << 15)
#define PHASE_B_MSK       (u32)((u32)(PHASE_B_ADC_CHANNEL) << 15)
#define PHASE_C_MSK       (u32)((u32)(PHASE_C_ADC_CHANNEL) << 15)*/

// Setting for sampling of VBUS and Temp after currents sampling
#define TEMP_FDBK_MSK     (u32)((u32)(TEMP_FDBK_CHANNEL) <<15)
#define BUS_VOLT_FDBK_MSK (u32)((u32)(BUS_VOLT_FDBK_CHANNEL) <<15)

// Settings for current sampling only
//#define TEMP_FDBK_MSK     (u32)(0)
//#define BUS_VOLT_FDBK_MSK (u32)(0)

// Setting for sampling of VBUS and Temp after currents sampling
#define SEQUENCE_LENGHT    0x00100000

// Settings for current sampling only
//#define SEQUENCE_LENGHT    0x00000000


//ADC中断和TIM1_UP中断优先级相同，在处理ADC中断时，TIM1_UP中断不会处理
//也就是说FOC的运行周期就是力矩环的周期，运行时间：
#define ADC_PRE_EMPTION_PRIORITY 1
#define ADC_SUB_PRIORITY 0

#define BRK_PRE_EMPTION_PRIORITY 0
#define BRK_SUB_PRIORITY 0

#define TIM1_UP_PRE_EMPTION_PRIORITY 1
#define TIM1_UP_SUB_PRIORITY 0

#define LOW_SIDE_POLARITY  TIM_OCIdleState_Reset

#define PWM2_MODE 0
#define PWM1_MODE 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8  bSector;  

u16 hPhaseAOffset;
u16 hPhaseBOffset;
u16 hPhaseCOffset;

u8 PWM4Direction=PWM2_MODE;

/* Private function prototypes -----------------------------------------------*/

void SVPWM_InjectedConvConfig(void);
s32  SATURATION_TO_S16 (s32 a);
/*******************************************************************************
* Function Name  : TIM1_PWM_Init
* Description    : It initializes PWM peripherals
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_PWM_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM1_OCInitStructure;
	TIM_BDTRInitTypeDef     TIM1_BDTRInitStructure;	
	
	/* Enable TIM1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOE, ENABLE);

	/*Timer1 alternate function full remapping*/  
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);  

	GPIO_StructInit(&GPIO_InitStructure);
	/* GPIOE Configuration: Channel 1, 1N, 2, 2N, 3, 3N and 4 Output */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
								GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 

	/* Lock GPIOE Pin9 and Pin11 Pin 13 (High sides) */
	GPIO_PinLockConfig(GPIOE, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13);

	GPIO_StructInit(&GPIO_InitStructure);
	/* GPIOE Configuration: BKIN pin */   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);  
	
	/* TIM1 Peripheral Configuration -----------------------*/
	/* TIM1 Registers reset */
	TIM_DeInit(TIM1);
	TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
	/* Time Base configuration */
	TIM1_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;

	// Initial condition is REP=0 to set the UPDATE only on the underflow
	TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;
	TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);

	TIM_OCStructInit(&TIM1_OCInitStructure);
	/* Channel 1, 2,3 in PWM mode */
	TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                  
	TIM1_OCInitStructure.TIM_Pulse = 0x505; //dummy value
	TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
	TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM1_OCInitStructure.TIM_OCNIdleState = LOW_SIDE_POLARITY;          
	TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 
	TIM_OC2Init(TIM1, &TIM1_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM1_OCInitStructure);
	
	/* Channel 4 Configuration in OC */
	TIM_OCStructInit(&TIM1_OCInitStructure);
	TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
	TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
	TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD - 1; 
	TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM1_OCInitStructure.TIM_OCNPolarity =TIM_OCNPolarity_Low;         
	TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM1_OCInitStructure.TIM_OCNIdleState = LOW_SIDE_POLARITY;            
	TIM_OC4Init(TIM1, &TIM1_OCInitStructure);

	/* Enables the TIM1 Preload on CC1 Register */
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	/* Enables the TIM1 Preload on CC2 Register */
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	/* Enables the TIM1 Preload on CC3 Register */
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	/* Enables the TIM1 Preload on CC4 Register */
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
	TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;
	TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;//0：刹车输入低电平有效；1：刹车输入高电平有效。由外部引脚电平从高到底，引起。
	TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	//TIM_AutomaticOutput_Enable: 刹车后，刹车输入电平变为无效无效后，下一个脉冲周期，自动回复脉冲输出。  
	//TIM_AutomaticOutput_Disable: 刹车后，脉冲输出永久禁止，除非手动调用TIM_CtrlPWMOutputs(TIM1, ENABLE)，否则不能启动pwm输出。

	TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

	TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
	//  TIM_ITConfig(TIM1, TIM_IT_Break,ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_Break,DISABLE);  
	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);

	// Resynch to have the Update evend during Undeflow
	TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
	//UG=1：重新初始化计数器，并产生一个更新事件加上REP=1,保证了在中心对齐模式下更新事件只发生向下溢出CNT=1处

	// Clear Update Flag
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);

	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);//不允许更新中断，

	TIM_ITConfig(TIM1, TIM_IT_CC4,DISABLE);//不允许CC4捕获中断
}

/*******************************************************************************
* Function Name  : CurrentReading_ADC_Init
* Description    : It initializes ADC peripherals
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CurrentReading_ADC_Init(void)
{	
	GPIO_InitTypeDef        GPIO_InitStructure;
	ADC_InitTypeDef         ADC_InitStructure;
	
	/* ADC1, ADC2, DMA, GPIO clocks enabling -----------------------------*/

	/* ADCCLK = PCLK2/6=12M */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	/* Enable GPIOA, GPIOC, GPIOE, AFIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOC,ENABLE);
	/* Enable ADC1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Enable ADC2 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 
	 
	/* ADC1, ADC2, PWM pins configurations -------------------------------*/
	/* PC0->温度ch10，PC1->Ia ch11,PC2->Ib ch12,PC3->Ic ch13,PC4->电位器 ch14 */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	 
	/* PA3->DC_BUS ch3 */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	/* ADC1、ADC2 registers reset -----------*/
	ADC_DeInit(ADC1);
	ADC_DeInit(ADC2);

	/* Enable ADC1、ADC2 */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	/* ADC1 configuration ------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC2 Configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);  
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);

	// Start calibration of ADC1、ADC2
	ADC_StartCalibration(ADC1);
	ADC_StartCalibration(ADC2);

	// Wait for the end of ADC1、ADC2 calibration 
	while (ADC_GetCalibrationStatus(ADC1) & ADC_GetCalibrationStatus(ADC2)) {}
		
}

void TIM1_ADC_NVIC_Init(void)
{
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the ADC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM1_UP_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM1_UP_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM1 BRK Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BRK_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = BRK_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
}


void SVPWM_3ShuntInit(void)
{
	TIM1_PWM_Init( );//TIM1 初始化
	CurrentReading_ADC_Init( );//ADC初始化
	SVPWM_3ShuntCurrentReadingCalibration( );//标定偏置电压
	TIM1_ADC_NVIC_Init( );	//ADC_TIM1中断初始化
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntCurrentReadingCalibration
* Description    : Store zero current converted values for current reading 
                   network offset compensation in case of 3 shunt resistors 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_3ShuntCurrentReadingCalibration(void)
{
	static u16 bIndex;

	/* ADC1 Injected group of conversions end interrupt disabling */
	ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);

	hPhaseAOffset=0;
	hPhaseBOffset=0;
	hPhaseCOffset=0;

	/* ADC1 Injected conversions trigger is given by software and enabled */ 
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);  
	ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE); 

	/* ADC1 Injected conversions configuration */ 
	ADC_InjectedSequencerLengthConfig(ADC1,3);
	ADC_InjectedChannelConfig(ADC1, PHASE_A_ADC_CHANNEL,1,SAMPLING_TIME_CK);
	ADC_InjectedChannelConfig(ADC1, PHASE_B_ADC_CHANNEL,2,SAMPLING_TIME_CK);
	ADC_InjectedChannelConfig(ADC1, PHASE_C_ADC_CHANNEL,3,SAMPLING_TIME_CK);

	/* Clear the ADC1 JEOC pending flag */
	ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
	ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);

	/* ADC Channel used for current reading are read 
	 in order to get zero currents ADC values*/ 
	for(bIndex=0; bIndex <NB_CONVERSIONS; bIndex++)
	{
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { }

		hPhaseAOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)>>3);
		hPhaseBOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2)>>3);
		hPhaseCOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_3)>>3);    
		/* Clear the ADC1 JEOC pending flag */
		ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);    
		ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);
	}
	
	SVPWM_InjectedConvConfig( );  
}



/*******************************************************************************
* Function Name  : SVPWM_InjectedConvConfig
* Description    : This function configure ADC1 for 3 shunt current 
*                  reading and temperature and voltage feedbcak after a 
*                  calibration of the three utilized ADC Channels
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_InjectedConvConfig(void)
{
	/* ADC1 Injected conversions configuration */ 
	ADC_InjectedSequencerLengthConfig(ADC1,2);
	ADC_InjectedSequencerLengthConfig(ADC2,2);

	ADC_InjectedChannelConfig(ADC1, PHASE_B_ADC_CHANNEL,1,SAMPLING_TIME_CK);
	ADC_InjectedChannelConfig(ADC1, BUS_VOLT_FDBK_CHANNEL,2,SAMPLING_TIME_CK);

	/* ADC1 Injected conversions trigger is TIM1 TRGO */ 
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO); 

	ADC_ExternalTrigInjectedConvCmd(ADC2,ENABLE);

	/* Bus voltage protection initialization*/                            
	ADC_AnalogWatchdogCmd(ADC1,ADC_AnalogWatchdog_SingleInjecEnable);
	ADC_AnalogWatchdogSingleChannelConfig(ADC1,BUS_VOLT_FDBK_CHANNEL);
	ADC_AnalogWatchdogThresholdsConfig(ADC1, OVERVOLTAGE_THRESHOLD>>3,0x00);

	/* ADC1 Injected group of conversions end and Analog Watchdog interrupts enabling */
	ADC_ITConfig(ADC1, ADC_IT_JEOC | ADC_IT_AWD, ENABLE);
	
	/* ADC2 Injected conversions configuration */ 
	ADC_InjectedSequencerLengthConfig(ADC2,2);
	ADC_InjectedChannelConfig(ADC2, PHASE_A_ADC_CHANNEL, 1,SAMPLING_TIME_CK);
	ADC_InjectedChannelConfig(ADC2, TEMP_FDBK_CHANNEL, 2,SAMPLING_TIME_CK);	
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntGetPhaseCurrentValues
* Description    : 此函数通过不同扇区，读取注入组通道的寄存器，根据Ia+Ib+Ic=0
				   来计算得到Ia和Ib的值，是Q15格式数据
* Input          : None
* Output         : Stat_Curr_a_b
* Return         : None
* @note 		 : 根据相电流定义，流入线圈为正，流出为负，而检测电流时，全部是下桥打开才检测
				   所以相电流应该全部都是负值，都是(hPhasexOffset)-(ADC Channel value) 
*******************************************************************************/
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void)
{
	Curr_Components Local_Stator_Currents;
	s32 wAux;//因为ADC1->JDR1是32位的

	switch (bSector)
	{
		case 4:
		case 5: //Current on Phase C not accessible     
			// Ia = (hPhaseAOffset)-(ADC Channel 11 value)    
			wAux = (s32)(hPhaseAOffset)- ((ADC1->JDR1)<<1);          
		    Local_Stator_Currents.qI_Component1=SATURATION_TO_S16(wAux);
		

			// Ib = (hPhaseBOffset)-(ADC Channel 12 value)
			wAux = (s32)(hPhaseBOffset)-((ADC2->JDR1)<<1);
			Local_Stator_Currents.qI_Component2=SATURATION_TO_S16(wAux);
		break;

		case 6:
		case 1:  //Current on Phase A not accessible     
			// Ib = (hPhaseBOffset)-(ADC Channel 12 value)
			wAux = (s32)(hPhaseBOffset)-((ADC1->JDR1)<<1);
			Local_Stator_Currents.qI_Component2=SATURATION_TO_S16(wAux);
		
			// Ia = -Ic -Ib 
			wAux = ((ADC2->JDR1)<<1)-hPhaseCOffset-Local_Stator_Currents.qI_Component2;
			Local_Stator_Currents.qI_Component1=SATURATION_TO_S16(wAux);
		break;

		case 2:
		case 3:  // Current on Phase B not accessible
			// Ia = (hPhaseAOffset)-(ADC Channel 11 value)     
			wAux = (s32)(hPhaseAOffset)-((ADC1->JDR1)<<1);
			Local_Stator_Currents.qI_Component1=SATURATION_TO_S16(wAux);
		
			// Ib = -Ic-Ia;
			wAux = ((ADC2->JDR1)<<1) - hPhaseCOffset - Local_Stator_Currents.qI_Component1;
			Local_Stator_Currents.qI_Component2=SATURATION_TO_S16(wAux);
		break;

		default:
		break;
	} 
	return(Local_Stator_Currents); 
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntCalcDutyCycles
* Description    : Computes duty cycle values corresponding to the input value
		   and configures the AD converter and TIM0 for next period 
		   current reading conversion synchronization
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_3ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
	s32 wX, wY, wZ, wUAlpha, wUBeta;
	u16  hTimePhA=0, hTimePhB=0, hTimePhC=0, hTimePhD=0;
	u16  hDeltaDuty;

	wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;
	wUBeta = -(Stat_Volt_Input.qV_Component2 * T);

	wX = wUBeta;
	wY = (wUBeta + wUAlpha)/2;
	wZ = (wUBeta - wUAlpha)/2;

	// 1.Sector calculation from wX, wY, wZ
	if (wY<0)
	{
		if (wZ<0)
		{
			bSector = SECTOR_5;
		}
		else // wZ >= 0
			if (wX<=0)
			{
				bSector = SECTOR_4;
			}
			else // wX > 0
			{
				bSector = SECTOR_3;
			}
	}
	else // wY > 0
	{
		if (wZ>=0)
		{
			bSector = SECTOR_2;
		}
		else // wZ < 0
			if (wX<=0)
			{  
				bSector = SECTOR_6;
			}
			else // wX > 0
			{
				bSector = SECTOR_1;
			}
	}

	// 2.Duty cycles computation 
	PWM4Direction=PWM2_MODE;

	switch(bSector)
	{  
		case SECTOR_1:
			hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
			hTimePhB = hTimePhA + wZ/131072;
			hTimePhC = hTimePhB - wX/131072;

			// ADC Syncronization setting value  
			//相电流的采样时间点到底有多大的重要性呢，
			if ((u16)(PWM_PERIOD-hTimePhA) > TW_AFTER)
			{
				hTimePhD = PWM_PERIOD - 1;
			}
			else
			{
				hDeltaDuty = (u16)(hTimePhA - hTimePhB);

				// Definition of crossing point
				if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhA)*2) 
				{
					hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A

					if (hTimePhD >= PWM_PERIOD)
					{
						// Trigger of ADC at Falling Edge PWM4
						// OCR update

						//Set Polarity of CC4 Low
						PWM4Direction=PWM1_MODE;

						hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
					}
				}
			}
            //配置注入组通道转换的ADC通道
			ADC1->JSQR = PHASE_B_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT;                    
			ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;                                         
		break;
		
		case SECTOR_2:
			hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
			hTimePhB = hTimePhA + wZ/131072;
			hTimePhC = hTimePhA - wY/131072;

			// ADC Syncronization setting value
			if ((u16)(PWM_PERIOD-hTimePhB) > TW_AFTER)
			{
				hTimePhD = PWM_PERIOD - 1;
			}
			else
			{
				hDeltaDuty = (u16)(hTimePhB - hTimePhA);

				// Definition of crossing point
				if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhB)*2) 
				{
					hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B 
				}
				else
				{
					hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B

					if (hTimePhD >= PWM_PERIOD)
					{
						// Trigger of ADC at Falling Edge PWM4
						// OCR update

						//Set Polarity of CC4 Low
						PWM4Direction=PWM1_MODE;
						hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
					}
				}
			}       
			ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT;                                            
			ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;
		break;

		case SECTOR_3:
			hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
			hTimePhC = hTimePhA - wY/131072;
			hTimePhB = hTimePhC + wX/131072;

			// ADC Syncronization setting value
			if ((u16)(PWM_PERIOD-hTimePhB) > TW_AFTER)
			{
				hTimePhD = PWM_PERIOD - 1;
			}
			else
			{
				hDeltaDuty = (u16)(hTimePhB - hTimePhC);

				// Definition of crossing point
				if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhB)*2) 
				{
					hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B 
				}
				else
				{
					hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B

					if (hTimePhD >= PWM_PERIOD)
					{
						// Trigger of ADC at Falling Edge PWM4
						// OCR update

						//Set Polarity of CC4 Low
						PWM4Direction=PWM1_MODE;
						hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
					}
				}
			}            
			ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT;                                                        
			ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;
		break;

		case SECTOR_4:
			hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
			hTimePhB = hTimePhA + wZ/131072;
			hTimePhC = hTimePhB - wX/131072;

			// ADC Syncronization setting value
			if ((u16)(PWM_PERIOD-hTimePhC) > TW_AFTER)
			{
				hTimePhD = PWM_PERIOD - 1;
			}
			else
			{
				hDeltaDuty = (u16)(hTimePhC - hTimePhB);

				// Definition of crossing point
				if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhC)*2)
				{
					hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C 
				}
				else
				{
					hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C

					if (hTimePhD >= PWM_PERIOD)
					{
						// Trigger of ADC at Falling Edge PWM4
						// OCR update

						//Set Polarity of CC4 Low
						PWM4Direction=PWM1_MODE;
						hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
					}
				}
			} 
			ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT;                                                  
			ADC2->JSQR = PHASE_B_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;
		break;  

		case SECTOR_5:
			hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
			hTimePhB = hTimePhA + wZ/131072;
			hTimePhC = hTimePhA - wY/131072;

			// ADC Syncronization setting value
			if ((u16)(PWM_PERIOD-hTimePhC) > TW_AFTER)
			{
				hTimePhD = PWM_PERIOD - 1;
			}
			else
			{
				hDeltaDuty = (u16)(hTimePhC - hTimePhA);
				// Definition of crossing point
				if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhC)*2) 
				{
					hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C 
				}
				else
				{
					hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C

					if (hTimePhD >= PWM_PERIOD)
					{
						// Trigger of ADC at Falling Edge PWM4
						// OCR update

						//Set Polarity of CC4 Low
						PWM4Direction=PWM1_MODE;
						hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
					}
				}
			}              
			ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT;                                                    
			ADC2->JSQR = PHASE_B_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;
		break;

		case SECTOR_6:
			hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
			hTimePhC = hTimePhA - wY/131072;
			hTimePhB = hTimePhC + wX/131072;

			// ADC Syncronization setting value
			if ((u16)(PWM_PERIOD-hTimePhA) > TW_AFTER)
			{
				hTimePhD = PWM_PERIOD - 1;
			}
			else
			{
				hDeltaDuty = (u16)(hTimePhA - hTimePhC);

				// Definition of crossing point
				if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhA)*2) 
				{
					hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A

					if (hTimePhD >= PWM_PERIOD)
					{

						PWM4Direction=PWM1_MODE;
						hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
					}
				}
			}    
			ADC1->JSQR = PHASE_B_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT;                                             
			ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;
		break;
	
		default:
		break;
	}
	
	if (PWM4Direction == PWM2_MODE)
	{
		//Set Polarity of CC4 High
		TIM1->CCER &= 0xDFFF;  
	}
	else
	{
		//Set Polarity of CC4 Low
		TIM1->CCER |= 0x2000;
	}

	//  //使能4个通道配合hall启动的时候使用
	//  TIM1->CCER |= 0x1555;

	//3.Load compare registers values
	TIM1->CCR1 = hTimePhA;
	TIM1->CCR2 = hTimePhB;
	TIM1->CCR3 = hTimePhC;
	TIM1->CCR4 = hTimePhD; // To Syncronyze the ADC
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntAdvCurrentReading
* Description    :  It is used to enable or disable the advanced current reading.
			if disabled the current reading will be performed after update event
* Input          : cmd (ENABLE or DISABLE)
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_3ShuntAdvCurrentReading(FunctionalState cmd)
{
	if (cmd == ENABLE)
	{
		// Enable ADC trigger sync with CC4
		//ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);  
		ADC1->CR2 |= 0x00001000;

		// Enable UPDATE ISR
		// Clear Update Flag
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	}
	else
	{		   
		// Sync ADC trigger with Update
		//ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
		ADC1->CR2 &=0xFFFFEFFF;

		// ReEnable EXT. ADC Triggering
		ADC1->CR2 |=0x00008000;    
	}
}

/*******************************************************************************
* Function Name  : SVPWMUpdateEvent
* Description    :  Routine to be performed inside the update event ISR  it reenable 
           the ext adc.trigger in 。It must be assigned to pSVPWM_UpdateEvent pointer.	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWMUpdateEvent(void)
{
	// ReEnable EXT. ADC Triggering
	ADC1->CR2 |= 0x00008000;

	// Clear unwanted current sampling
	ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
}

/*******************************************************************************
* Function Name  : SVPWMEOCEvent
* Description    :  Routine to be performed inside the end of conversion ISR
		         It computes the bus voltage and temperature sensor sampling 
		        and disable the ext. adc triggering.	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 SVPWMEOCEvent(void)
{
	// Store the Bus Voltage and temperature sampled values
	h_ADCTemp = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);
	h_ADCBusvolt = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);

	if ((State == START) || (State == RUN))//INIT状态时，直接跳过，否则禁止ADC转化触发
	{          
		// Disable EXT. ADC Triggering
		ADC1->CR2 = ADC1->CR2 & 0xFFFF7FFF;
	}
	return ((u8)(1));
}
/*******************************************************************************
* Function Name  : SATURATION_TO_S16
* Description    :  	
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

s32 SATURATION_TO_S16 (s32 a)
{
	if(a > S16_MAX)
		a= S16_MAX; 
	else if (a < S16_MIN)
		a = S16_MIN;
	return(a);		
}
								
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/  
