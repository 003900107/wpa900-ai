/**
  ******************************************************************************
  * @file    ADC/3ADCs_DMA/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "I2CRoutines.h"

void TimingDelay_Decrement(void);
void Sampling_IRQHandler(void);
void MeaFreq_IRQHandler(void);


extern I2C_InitTypeDef I2C_InitStruct;
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_3ADCs_DMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
 
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
 
/**
  * @brief  This function handles ADC1 and ADC2 global interrupts requests.
  * @param  None
  * @retval None
  */


void I2C1_EV_IRQHandler(void)
{
 
	_I2C1_EV_IRQHandler();
  
}

 void I2C1_ER_IRQHandler(void)
{
//    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF))
//    {
//        I2C_ClearFlag(I2C1, I2C_FLAG_AF);
//
//    }
//
//    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BERR))
//    {
//        I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
//
//    }
//	I2C_Cmd(I2C1, DISABLE);
//	I2C_DeInit(I2C1);
//	I2C_Init(I2C1,&I2C_InitStruct);	
//	I2C_Cmd(I2C1, ENABLE);
_I2C1_ER_IRQHandler();

}
	 
 void TIM2_IRQHandler(void)
{
  	 if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET) TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	 Sampling_IRQHandler();
}

 void TIM4_IRQHandler(void)
{
  	 if(TIM_GetITStatus(TIM4,TIM_IT_CC2)!=RESET) TIM_ClearITPendingBit(TIM4,TIM_IT_CC2 );
	 MeaFreq_IRQHandler();
}
//void DMA1_Channel1_IRQHandler(void)
//{
//  if(DMA_GetITStatus(DMA1_IT_TC1))
//  {
//  DMA_ClearITPendingBit(DMA1_IT_TC1);	
//  PeriodCycle_Index=(PeriodCycle_Index+1)&POINT_MASK; 
//  PeriodCycleTab[PeriodCycle_Index]=OverSampling(ADC12ConvertedValueTab,16);
//  }
//}
#if I2C_METHOD==DMA
void DMA1_Channel7_IRQHandler(void)
{
	_DMA1_Channel7_IRQHandler();
}
#endif
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
