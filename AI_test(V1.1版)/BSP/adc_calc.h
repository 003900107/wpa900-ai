#ifndef __ADCCALC_H
#define __ADCCALC_H
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32_dsp.h"
#include "bsp.h"
#include "BaseBus_Protocol.h"
#include "dft.h"

#define ScalingFactor   16384  
#define FreqSample    3200
#define DC_PERCENTAGE  0.4  
#define AMPLITUDE  (1200*ScalingFactor/VREF_VALUE)
#define PI              3.1415926    
#define PI2             6.28318530717959   


#define CR2_JEXTTRIG_JSWSTART_Set   ((uint32_t)0x00208000)
#define CR2_EXTTRIG_SWSTART_Set     ((uint32_t)0x00500000)
#define JDR_Offset                  ((uint8_t)0x28)

#if defined LXIV_POINT 
#define SAMP_POINT_NBR	 64
#define POINT_MASK	 0x3F
#define FastFourierTrans(x,y)   cr4_fft_64_stm32(FFT_ResultTab[x],PeriodCycleTab[x],y);
#elif defined CXXVIII_POINT
#define SAMP_POINT_NBR   128
#define POINT_MASK	 0x7F
#define FastFourierTrans(x,y)   cr4_fft_256_stm32(FFT_ResultTab[x],PeriodCycleTab[x],y);
#elif defined CCLVI_POINT
#define SAMP_POINT_NBR   256
#define POINT_MASK	 0xFF
#define FastFourierTrans(x,y)   cr4_fft_1024_stm32(FFT_ResultTab[x],PeriodCycleTab[x],y);
#endif

#define VREF_VALUE 3300
#define AIN_REF 1540
#define DC_OFFSET 0x1DC0

#define UA 0
#define UB 1
#define UC 2
#define IA 3
#define IB 4
#define IC 5
#define U0 6
#define UAB 7
#define UBC 8
#define UCA 9
#define IAP 10
#define IBP 11
#define ICP 12

#define MAX(x,y) (x>y?x:y)
#define MIN(x,y) (x>y?y:x)

//¿ìËÙ¸µÁ¢È~ÞD“Q

int16_t OverSampling(uint16_t *pSampleBase,uint8_t nsize);
void Sampling_IRQHandler(void);
void MeaFreq_IRQHandler(void);
void power_Phase_Radians(long nfill,uint8_t Seq);
void powerMag(long nfill,uint8_t Seq);
u32  Wave_Generator(u16 i);
#endif
