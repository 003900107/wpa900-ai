#include "bsp.h"
#include "stdio.h"
#include "I2CRoutines.h"


__IO  uint32_t timingdelay;
__IO  uint32_t LocalTime = 0;

#if I2C_METHOD == SINGLEBYTE
extern unsigned char TxBuffer[8];
#endif
#if MEA_UPDATE_METHOD == MEMBLKCP
extern unsigned char TxBuffer[127];
#endif

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC2_DR_Address    ((uint32_t)0x4001284C)
#define ADC3_DR_Address    ((uint32_t)0x40013C4C)

#define UA_CHN  ADC_Channel_5
#define UB_CHN	ADC_Channel_6
#define UC_CHN	ADC_Channel_12
#define U0_CHN	ADC_Channel_10
#define IA_CHN	ADC_Channel_0
#define IB_CHN	ADC_Channel_3
#define IC_CHN	ADC_Channel_4
#define IAP_CHN	ADC_Channel_14
#define IBP_CHN	ADC_Channel_15
#define ICP_CHN	ADC_Channel_11


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC1_InitStructure;
ADC_InitTypeDef ADC2_InitStructure;
ADC_InitTypeDef ADC3_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
DMA_InitTypeDef  I2CDMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
I2C_InitTypeDef I2C_InitStruct;


//uint16_t InputSeq[3]={GPIO_Pin_6,GPIO_Pin_7,GPIO_Pin_9};

/* Private function prototypes -----------------------------------------------*/


//void DMA_Configuration(void) 
//{  /* DMA1 channel1 configuration ----------------------------------------------*/
//  
//  DMA_DeInit(DMA1_Channel1);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC12ConvertedValueTab;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//  DMA_InitStructure.DMA_BufferSize = 16;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel1, &DMA_InitStructure);  
//  /* Enable DMA1 channel1 */
//  DMA_Cmd(DMA1_Channel1, ENABLE);
//
////  DMA_DeInit(DMA1_Channel2);
////  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC2_DR_Address;
////  DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t)ADC12ConvertedValueTab;
////  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
////  DMA_InitStructure.DMA_BufferSize = 16;
////  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
////  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
////  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
////  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
////  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
////  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
////  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
////  DMA_Init(DMA1_Channel2, &DMA_InitStructure);  
////  /* Enable DMA1 channel1 */
////  DMA_Cmd(DMA1_Channel2, ENABLE);
//
//  /* DMA2 channel3 configuration ----------------------------------------------*/
//  DMA_DeInit(DMA1_Channel3);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC3_DR_Address;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC3ConvertedValueTab;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//  DMA_InitStructure.DMA_BufferSize = 16;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// // DMA_Init(DMA1_Channel3, &DMA_InitStructure);  
//  /* Enable DMA2 channel5 */
//  //DMA_Cmd(DMA1_Channel3, ENABLE);
//
//  DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
// // DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
// // DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);
//}

void ADC_Configuration(void)
{  /* ADC1 configuration ------------------------------------------------------*/
  
  ADC1_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC1_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC1_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC1_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC1_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC1_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC1_InitStructure);
  /* ADC1 regular channels configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC1, 3);
  ADC_InjectedChannelConfig(ADC1, UA_CHN, 1, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC1, UB_CHN, 2, ADC_SampleTime_1Cycles5); 
  ADC_InjectedChannelConfig(ADC1, IC_CHN, 3, ADC_SampleTime_1Cycles5); 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);  
  /* Enable ADC1 DMA */
  // ADC_DMACmd(ADC1, ENABLE);
  
  /* ADC2 configuration ------------------------------------------------------*/
  
  ADC2_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC2_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC2_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC2_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC2_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC2_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC2, &ADC2_InitStructure);
  /* ADC2 regular channels configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC2, 4);
  ADC_InjectedChannelConfig(ADC2, U0_CHN, 1, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC2, IAP_CHN, 2, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC2, IBP_CHN, 3, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC2, ICP_CHN, 4, ADC_SampleTime_1Cycles5);
  
  ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);
  /* Enable ADC2 EOC interupt */
  //  ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
  
  /* ADC3 configuration ------------------------------------------------------*/
  
  ADC3_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC3_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC3_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC3_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC3_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC3_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC3, &ADC3_InitStructure);
  /* ADC3 regular channel14 configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC3, 3);
  ADC_InjectedChannelConfig(ADC3, IA_CHN, 1, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC3, IB_CHN, 2, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC3, UC_CHN, 3, ADC_SampleTime_1Cycles5);
  ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_None);
  /* Enable ADC3 DMA */
  //ADC_DMACmd(ADC3, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
  
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
  
  /* Enable ADC2 reset calibaration register */   
  ADC_ResetCalibration(ADC2);
  /* Check the end of ADC2 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC2));
  
  /* Start ADC2 calibaration */
  ADC_StartCalibration(ADC2);
  /* Check the end of ADC2 calibration */
  while(ADC_GetCalibrationStatus(ADC2));
  
  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  
  /* Enable ADC3 reset calibaration register */   
  ADC_ResetCalibration(ADC3);
  /* Check the end of ADC3 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC3));
  
  /* Start ADC3 calibaration */
  ADC_StartCalibration(ADC3);
  /* Check the end of ADC3 calibration */
  while(ADC_GetCalibrationStatus(ADC3));
  
}
/**
* @brief  Configures the different system clocks.
* @param  None
* @retval None
*/
void RCC_Configuration(void)
{
  /*
  HCLK(AHB)=56M
  PCLK2(高APB)=56M
  PCLK1(低APB)=28M
  */
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4);   //14M
  
  /* Enable peripheral clocks ------------------------------------------------*/
  /* Enable DMA1 and DMA2 clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 |RCC_AHBPeriph_CRC, ENABLE);
  
  /* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3 | 
                         RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                             RCC_APB2Periph_AFIO , ENABLE);
}

/**
* @brief  Configures the different GPIO ports.
* @param  None
* @retval None
*/
void GPIO_Configuration(void)
{
  /* Configure PC.02, PC.03 and PC.04 (ADC Channel12, ADC Channel13 and 
  ADC Channel14) as analog inputs */
  //  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
  //  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  //  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_5 | GPIO_Pin_6 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5  ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);  

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
}


/**
* @brief  Configures Vector Table base location.
* @param  None
* @retval None
*/
void NVIC_Configuration(void)
{
  /* Configure and enable ADC interrupt */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	 
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#if I2C_METHOD==DMA
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  //此处优先级要比I2C_EV高，否则读取过程结束后要先打断该中断进入EV去清STOPF
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;	
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif

  //NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);
}


void I2C_Configuration(void)
{
  uint16_t timeout;
  GPIO_InitTypeDef GPIO_InitStructure; 
  DMA_InitTypeDef  I2CDMA_InitStructure;
  
  /* Enable I2C2 reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* Release I2C2 from reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  
  
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;   //复用开漏输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_ClockSpeed =100000;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  
  
  I2C_InitStruct.I2C_OwnAddress1 = 0xC0;
  I2C_Init(I2C1,&I2C_InitStruct);
  timeout=0x1ff;
  while(timeout--);
  
  I2C_DeInit(I2C1);
  I2C_Init(I2C1,&I2C_InitStruct);
  I2C_Cmd(I2C1,ENABLE);
  I2C_ITConfig(I2C1, I2C_IT_EVT|I2C_IT_ERR, ENABLE);
#if I2C_METHOD == DMA  
  {
    I2C1->CR2 |= CR2_DMAEN_Set;
  }
#endif
#if I2C_METHOD == INTERRUPT  
  {
    I2C1->CR2 |= I2C_IT_BUF;
  }
#endif

#if I2C_METHOD ==DMA		
  DMA_DeInit(DMA1_Channel6);
  I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
  I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer;   
  I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    
  I2CDMA_InitStructure.DMA_BufferSize = /*98*/102;    //tyh:20130730           
  I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &I2CDMA_InitStructure);
  
  DMA_DeInit(DMA1_Channel7);
  DMA_Init(DMA1_Channel7, &I2CDMA_InitStructure);
  DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
#endif
}

void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructure.TIM_Prescaler = 9;//128点=5
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 1749;	   //56M除1750 32K	   当HCLK = 56MHZ, 而APB1 prescaler != 1 所以timer2时钟56M
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM2,ENABLE);
  TIM_ClearITPendingBit(TIM2,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2,ENABLE);
  
}
#ifdef WATCHDOG
void IWDG_Configuration(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: 32KHz(LSI) / 32K/256 = 125Hz */
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  
  /* Set counter reload value to 349 */
  IWDG_SetReload(499);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
  
}

void WDGFeeding(void)
{
  IWDG_ReloadCounter();
} 
#endif
void Time_Update(void)
{
  LocalTime++;
}

void SysTick_Configuration()

{
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
}

void Delay(volatile uint32_t nCount)
{ 
  timingdelay = LocalTime + nCount;  
  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime);
}

void FREQ_MEA_Init ( void )
{
  /* TIM2 Input Capture Channel 4 mode Configuration */
  
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  //  NVIC_InitTypeDef NVIC_InitStructure; 
  
  //DBGMCU_Config(DBGMCU_TIM3_STOP,ENABLE); 
  //fDTS = fCK_INT 
  //CK_INT 1M
  //TIM_TimeBaseStructure.TIM_Period =;     
  //TIM_TimeBaseStructure.TIM_Prescaler = 55;      
  //TIM_TimeBaseStructure.TIM_ClockDivision = 0;   
  //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_PrescalerConfig(TIM4, 559, TIM_PSCReloadMode_Immediate);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0f;
  TIM_ICInit(TIM4, &TIM_ICInitStructure); 
  
  //PB7
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU; //GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  TIM_ClearITPendingBit(TIM4,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
  
  
  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

void I2CHW_Reset(void)
{
  __IO uint32_t Timeout;
  uint8_t i;
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  I2C_DeInit(I2C1);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
 
  //  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_8;
  //  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //  
  //  GPIO_SetBits(GPIOB,GPIO_Pin_9);
  //  GPIO_SetBits(GPIOB,GPIO_Pin_8);
  //  Timeout=0x1ff;
  //  while(Timeout--);
  
  I2C_Configuration();
  Timeout=0x1ff;
  while(Timeout--);  
}    


