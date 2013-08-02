#ifndef __BSP_H
#define __BSP_H
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "adc_calc.h"
#include "BaseBus_Protocol.h"
#include "Realtimedb.h"						

#define ALARM_LED     GPIOD, GPIO_Pin_9
#define RUNSTAT_LED   GPIOD, GPIO_Pin_8
#define I2C_RESET_LED GPIOB, GPIO_Pin_15



void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void); 
void ADC_Configuration(void);
void TIM2_Configuration(void);
void I2C_Configuration(void);
#ifdef WATCHDOG
void IWDG_Configuration(void);
void WDGFeeding(void);
#endif
void FREQ_MEA_Init ( void );
void SysTick_Configuration(void);
void Delay(u32 nTime);
void Time_Update(void);
void I2CHW_Reset(void);

#endif

