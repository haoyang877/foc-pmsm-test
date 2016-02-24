#ifndef __HALL_H
#define __HALL_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */
#define HALL_TIMER TIM5 

/* Includes ------------------------------------------------------------------*/
#include "MC_hall_prm.h" 

/* Exported types ------------------------------------------------------------*/
extern u16 PWM_duty;
extern u8 Hall_Start_flag;
/* Exported constants --------------------------------------------------------*/
#define	HALL_PHASE_SHIFT (s16) 180
/* Exported macro ------------------------------------------------------------*/
void Halltimer_init(void);
u16  HALL_GetElectricalAngle(void);

void HALL_Init_Electrical_Angle(void);
void BLDC_Change_phase( s8 Dir,u16 duty);
u8 ReadHallState(void);

bool Hall_Startup(void);
#endif /* __HALL_H */
