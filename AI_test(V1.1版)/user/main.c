
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "bsp.h"
#include "table_fft.h"
#include "dft.h"
#include "adc_calc.h"
#include "Realtimedb.h"

#include  <string.h>

extern float MeaTab[MEANUM];
uint16_t Value;
extern int16_t PeriodCycleTab[10][SAMP_POINT_NBR];
//extern uint32_t FFT_ResultTab[7][SAMP_POINT_NBR];
//extern uint32_t lBUFMAG[SAMP_POINT_NBR];
extern float fBUFANG[7];

extern uint32_t PeriodCycle_Index;

MEA_DFT  Ua_Channel;
Mea_Para meas={1,};
__IO uint8_t RunStatturn;

/** @addtogroup STM32F10x_StdPeriph_Examples
* @{
*/

/** @addtogroup ADC_3ADCs_DMA
* @{
*/ 


/* Private functions ---------------------------------------------------------*/

/**
* @brief   Main program
* @param  None
* @retval None
*/
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32f10x_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f10x.c file
  */     
  
  /* System clocks configuration ---------------------------------------------*/
  SystemInit();
  
  RCC_Configuration();
  
  GPIO_Configuration();
  GPIO_WriteBit(ALARM_LED,  Bit_SET);
  GPIO_WriteBit(RUNSTAT_LED,  Bit_SET); 
  GPIO_WriteBit(I2C_RESET_LED,  Bit_SET);  
  
  ADC_Configuration();
  
  I2C_Configuration();
  
  SysTick_Configuration();
  
  TIM2_Configuration();
  
#ifdef WATCHDOG
  IWDG_Configuration();
#endif
  
  NVIC_Configuration();
  
  FREQ_MEA_Init();
  
  DataBase_Init();
  // DMA_Configuration();
  
  while (1)
  {
    //FastFourierTrans(0,SAMP_POINT_NBR) ;
    //  powerMag(SAMP_POINT_NBR,UA);     
    // power_Phase_Radians(SAMP_POINT_NBR,UA);  
    //for(i=0;i<6;i++)
    //{
    // MeaTab[0]=(FFT_ResultTab[0][1])>>16;
    if(63==PeriodCycle_Index)
    {
      TOTAL_MEASURE(&meas);
      
      SequenceFilter_2(&meas);
      
      SequenceFilter_0(&meas);
      
      ValueScaling(MeaTab,&meas);
      
      I2CHW_Maintain();		
      
#ifdef WATCHDOG
      WDGFeeding();
#endif
       
      //run light
      if(RunStatturn++&0x01)
      {
        GPIO_WriteBit(RUNSTAT_LED,  Bit_RESET);
      }
      else
      {
        GPIO_WriteBit(RUNSTAT_LED,  Bit_SET);
      }       
      
      //FUN_DFT_64(UA,&Ua_Channel);
    }
    //(FFT_ResultTab[0][1]);
    //MeaTab[2]=lBUFMAG[1]*VREF_VALUE/16384;
    //MeaTab[3]=(uint32_t)(fBUFANG[1]);
    //MeaTab[0]=Ua_Channel.am*1000;
    //MeaTab[1]=Ua_Channel.zero ;
    // }
    
    BusCalling_Process(); 
  }
  
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  
  
  /* Infinite loop */
  while (1)
  {  }
}
#endif

/**
* @}
*/ 

/**
* @}
*/ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
