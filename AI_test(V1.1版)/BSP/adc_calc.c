#include <math.h>
#include "adc_calc.h"
#include "dft.h"

uint16_t ADC11_ConvertedValueTab[16];
uint16_t ADC12_ConvertedValueTab[16];
uint16_t ADC13_ConvertedValueTab[16];

uint16_t ADC21_ConvertedValueTab[16];
uint16_t ADC22_ConvertedValueTab[16];
uint16_t ADC23_ConvertedValueTab[16];
uint16_t ADC24_ConvertedValueTab[16];

uint16_t ADC31_ConvertedValueTab[16];
uint16_t ADC32_ConvertedValueTab[16];
uint16_t ADC33_ConvertedValueTab[16];

int16_t PeriodCycleTab[13][SAMP_POINT_NBR];

uint32_t lBUFMAG[SAMP_POINT_NBR];
uint32_t FFT_ResultTab[7][SAMP_POINT_NBR];
float fBUFANG[SAMP_POINT_NBR];

int32_t FreqCounter=0x00;
int32_t temp_FreqCounter[32];
uint8_t FreqTabIndex=0;
uint8_t FreqTabIndexPlus=0; 

uint32_t PeriodCycle_Index;

__IO uint32_t Frequence=1749;

extern uint32_t MeaTab[MEANUM];



int16_t OverSampling(uint16_t *pSampleBase,uint8_t nsize)
{
  /*STM32采样过采样分辨率对应关系
  ---------------------------------
  分辨率  | 采样次数  | 每秒采样次数 
  12ADC   |    1      |      1M 
  13ADC   |    4      |      250K 
  14ADC   |    16     |      62.5K 
  15ADC   |    64     |      15.6K 
  16ADC   |    256    |      3.9K
  ---------------------------------
  
  每多一位分辨率多4倍次采样，每次中断采样过采样16次提高分辨率至14位
  */
  uint8_t Sampseq= 0;
  uint32_t result = 0;
  int16_t _result;
  
  for( Sampseq = 0;Sampseq < nsize ;Sampseq++)
    result += *(pSampleBase+Sampseq);
  
  _result=(((s32)(result>>2))*VREF_VALUE/16384)-AIN_REF; //convert to mv
  //result=(result>>2) ;
  //return (result<<16);
  return _result;    
}


void Sampling_IRQHandler(void)
{
  uint8_t i;
  if( TIM2->ARR != Frequence )
  {
    if( (Frequence<2188) &&  (Frequence > 1456) ) TIM2->ARR = Frequence-1;
  }
  
  for(i=0;i<16;i++)
  {
    ADC1->CR2 |= CR2_JEXTTRIG_JSWSTART_Set;
    ADC2->CR2 |= CR2_JEXTTRIG_JSWSTART_Set;
    ADC3->CR2 |= CR2_JEXTTRIG_JSWSTART_Set;
    
    while( (ADC1->SR & ADC_FLAG_JEOC) == 0);
    while( (ADC2->SR & ADC_FLAG_JEOC) == 0);
    while( (ADC3->SR & ADC_FLAG_JEOC) == 0);
    
    ADC11_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC1) + ADC_InjectedChannel_1 + JDR_Offset)));
    ADC12_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC1) + ADC_InjectedChannel_2 + JDR_Offset)));
    ADC13_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC1) + ADC_InjectedChannel_3 + JDR_Offset)));
    
    ADC21_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC2) + ADC_InjectedChannel_1 + JDR_Offset)));
    ADC22_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC2) + ADC_InjectedChannel_2 + JDR_Offset)));
    ADC23_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC2) + ADC_InjectedChannel_3 + JDR_Offset)));
    ADC24_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC2) + ADC_InjectedChannel_4 + JDR_Offset)));
    
    ADC31_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC3) + ADC_InjectedChannel_1 + JDR_Offset)));
    ADC32_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC3) + ADC_InjectedChannel_2 + JDR_Offset)));
    ADC33_ConvertedValueTab[i]=(u16) (*(vu32*) ((((u32)ADC3) + ADC_InjectedChannel_3 + JDR_Offset)));	
  }
  PeriodCycle_Index=(PeriodCycle_Index+1)&POINT_MASK; 
  PeriodCycleTab[UA][PeriodCycle_Index]=OverSampling(ADC11_ConvertedValueTab,16);
  //PeriodCycleTab[UA][PeriodCycle_Index]=Wave_Generator(PeriodCycle_Index);
  PeriodCycleTab[UB][PeriodCycle_Index]=OverSampling(ADC12_ConvertedValueTab,16);
  PeriodCycleTab[UC][PeriodCycle_Index]=OverSampling(ADC13_ConvertedValueTab,16);
  PeriodCycleTab[U0][PeriodCycle_Index]=OverSampling(ADC21_ConvertedValueTab,16);
  PeriodCycleTab[IA][PeriodCycle_Index]=OverSampling(ADC31_ConvertedValueTab,16);
  PeriodCycleTab[IB][PeriodCycle_Index]=OverSampling(ADC32_ConvertedValueTab,16);
  PeriodCycleTab[IC][PeriodCycle_Index]=OverSampling(ADC33_ConvertedValueTab,16);
  PeriodCycleTab[IAP][PeriodCycle_Index]=OverSampling(ADC22_ConvertedValueTab,16);
  PeriodCycleTab[IBP][PeriodCycle_Index]=OverSampling(ADC23_ConvertedValueTab,16);
  PeriodCycleTab[ICP][PeriodCycle_Index]=OverSampling(ADC24_ConvertedValueTab,16);
}


void MeaFreq_IRQHandler(void)
{	
  temp_FreqCounter[FreqTabIndex&0x1f]=TIM4->CCR2;
  
  FreqCounter=(s32)(temp_FreqCounter[FreqTabIndex&0x1f]-temp_FreqCounter[(FreqTabIndex-1)&0x1f]);
  
  if(FreqCounter<0) 
    FreqCounter=65535+FreqCounter;
  
  Frequence=7*FreqCounter/8;
  
  FreqTabIndex+=1;
}

void powerMag(long nfill,uint8_t Seq)
{
  float lX,lY;
  u32 i;
  
  for (i=0; i < nfill; i++)
  {
    lX= (FFT_ResultTab[Seq][i]<<16)>>16; /* sine_cosine --> cos */
    lY= (FFT_ResultTab[Seq][i] >> 16);   /* sine_cosine --> sin */    
    {
      // float X=  64*((float)lX)/32768;
      //float Y = 64*((float)lY)/32768;
      float Mag = sqrt(lX*lX+ lY*lY);
      //Mag=MAX(lX,lY)+MIN(lX,lY)/3;
      
      lBUFMAG[i] =(u32)(2*Mag/SAMP_POINT_NBR);
      if (0==i) lBUFMAG[0]=Mag;
    }    
  }
  
}

void power_Phase_Radians(long nfill,uint8_t Seq)
{
  int32_t lX,lY;
  uint32_t i;
  
  for (i=0; i < nfill/2; i++)
  {
    lX= (FFT_ResultTab[Seq][i]<<16)>>16; /* 取低16bit，sine_cosine --> cos */
    lY= (FFT_ResultTab[Seq][i] >> 16);   /* 取高16bit，sine_cosine --> sin */    
    {
      float phase = atan(lY/lX);
      if (lY>=0)
      {
        if (lX>=0)
          ;
        else
          phase+=PI;  
      }
      else
      {             
        if (lX>=0)
          phase+=PI2;                  
        else 
          phase+=PI;                    
      }                   
      
      fBUFANG[i] = phase*180.0/PI;
    }    
  }
}

u32  Wave_Generator(u16 i)
{ 
  float fx; 
  
  /* + AMPLITUDE *0.5* sin(PI2*i*2500.0/FreqSample) + AMPLITUDE*0.2*sin(PI2*i*2550.0/FreqSample)*/
  fx  =1400* sin(PI2*i*50.0/FreqSample);
  // return (u32)fx)<<16;
  return (s32)fx;
}



