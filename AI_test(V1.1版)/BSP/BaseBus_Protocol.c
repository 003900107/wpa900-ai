/*
*     @arg I2C_EVENT_SLAVE_ADDRESS_MATCHED   : EV1
*     @arg I2C_EVENT_SLAVE_BYTE_RECEIVED     : EV2
*     @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED  : EV3
*     @arg I2C_EVENT_SLAVE_ACK_FAILURE       : EV3-2
*     @arg I2C_EVENT_MASTER_MODE_SELECT      : EV5
*     @arg I2C_EVENT_MASTER_MODE_SELECTED    : EV6
*     @arg I2C_EVENT_MASTER_BYTE_RECEIVED    : EV7
*     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED : EV8
*     @arg I2C_EVENT_MASTER_MODE_ADDRESS10   : EV9
*     @arg I2C_EVENT_SLAVE_STOP_DETECTED     : EV4

========================================

*/
#include "stm32f10x_i2c.h"
#include "bsp.h"
#include "BaseBus_Protocol.h"
#include "I2CRoutines.h"


#include <string.h>
#include <stdbool.h>

unsigned char RxBuffer[8];
#if I2C_METHOD == SINGLEBYTE
unsigned char TxBuffer[8];
#endif
#if MEA_UPDATE_METHOD == MEMBLKCP
unsigned char TxBuffer[127];
#endif
uint8_t CommingCall=0;
__IO uint8_t COEF_INTFLAG=0;
__IO uint8_t Times_of_Setcoef=0;
__IO uint8_t SlaveReceptionComplete = 0;

extern Setting SetCurrent;
extern I2C_InitTypeDef I2C_InitStruct;

float MeaTab[MEANUM];
uint32_t EV_Word;
u32 EV[256];

//uint16_t Amp_Coef[10]={1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,};

uint32_t Ang_Coef[10];

#define HEADCHECK(a,b)  ((a==0xA5&&b==0x5A)?1:0)
#define BufferSize  8

#define I2C_PollingWait 0xfff

__IO uint8_t AinQuerry_index;

uint16_t checksum16(uint8_t *pByte , uint16_t Len )
{
  uint16_t sum=0,i=0;
  while(Len--)
    sum+=*(pByte+i++); 
  return sum;
}
void AIresponse(unsigned char AiSeq)
{
#if MEA_UPDATE_METHOD == MEMBLKCP
  uint16_t CRCValue;
#endif
  //unsigned char *p;
  //p=unsigned char*(&MeaTab[AiSeq]);
  //	 if(0x00==COEF_INTFLAG)
  //    {
  TxBuffer[0]=0xA5;
  TxBuffer[1]=0x5A;
  TxBuffer[2]=AI_RES;
  
#if I2C_METHOD == SINGLEBYTE
  TxBuffer[3]=AiSeq;
  //TxBuffer[4]=*p++;
  //TxBuffer[5]=*p++;
  //TxBuffer[6]=*p++;
  //TxBuffer[7]=*p;
  memcpy((u8*)(&TxBuffer[4]),&(MeaTab[AiSeq]),4);
#endif
  
#if MEA_UPDATE_METHOD == MEMBLKCP
  memcpy((u8*)(&TxBuffer[4]),&MeaTab,/*92*/96);
  
  CRC_ResetDR();
  CRCValue =CRC_CalcBlockCRC((uint32_t *)TxBuffer, /*24*/25);        
  //CRCValue =checksum16(TxBuffer, 100);
  TxBuffer[/*96*/100]=CRCValue&0xFF;
  TxBuffer[/*97*/101]=(CRCValue>>8)&0xFF;
#endif
  //}
  //else
  //{
  //  TxBuffer[5]=++Times_of_Setcoef;
  //  COEF_INTFLAG=0x00;
  //}                
}


bool AiCoefProcess(unsigned char AiSeq)
{    
  u8 status,i;
  __IO uint8_t Timeout=0xff;
  
  if((AiSeq>0)&&(AiSeq<10))
  {
    // memcpy((u8*)(&(Amp_Coef[AiSeq-1])),&(RxBuffer[4]),2);
    memcpy((u8*)(&(SetCurrent.ChannelCoef[AiSeq-1])),&(RxBuffer[4]),2);
    //SetCurrent.ChannelCoef[AiSeq]=RxBuffer[4]+RxBuffer[5]*256;
    status=DataBase_Write(STORAGE_ROMADDR,(u32 *)(&SetCurrent),sizeof(Setting));
    
    //		while(Timeout--);
    //        if(0==memcmp(&(RxBuffer[4]),(const void*)(STORAGE_ROMADDR+(AiSeq-1)*2),2))
    //		  {  
    //			TxBuffer[0]=0xA5;
    //			TxBuffer[1]=0x5A;
    //			TxBuffer[2]=AI_COF;
    //			TxBuffer[3]=AiSeq;
    //			TxBuffer[4]=status;
    //			TxBuffer[5]=0x00;
    //			TxBuffer[6]=0x00;
    //			TxBuffer[7]=0x00;
    if(status)
      return 1;
    // }
  } 
  
  if(10==AiSeq)
  {
    for(i=0;i<9;i++) 
    {
      SetCurrent.ChannelCoef[i] =1000;
    }
    status=DataBase_Write(STORAGE_ROMADDR,(u32 *)(&SetCurrent),sizeof(Setting));	
  }  
  
  return 0;     
}

bool AiCalibration(void)
{
  u8 status,i;
  __IO uint8_t Timeout=0xff;
  
  
  SetCurrent.ChannelCoef[0] =(u16)(57.74/MeaTab[0]*1000+0.5);
  SetCurrent.ChannelCoef[1] =(u16)(57.74/MeaTab[2]*1000+0.5);
  SetCurrent.ChannelCoef[2] =(u16)(57.74/MeaTab[4]*1000+0.5);
  SetCurrent.ChannelCoef[3] =(u16)(5.000/MeaTab[6]*1000+0.5);
  SetCurrent.ChannelCoef[4] =(u16)(5.000/MeaTab[8]*1000+0.5);
  SetCurrent.ChannelCoef[5] =(u16)(5.000/MeaTab[10]*1000+0.5);
  SetCurrent.ChannelCoef[6] =(u16)(100.0/MeaTab[12]*1000+0.5);
  SetCurrent.ChannelCoef[7] =(u16)(100.0/MeaTab[13]*1000+0.5);
  SetCurrent.ChannelCoef[8] =(u16)(100.0/MeaTab[14]*1000+0.5);
  
  status=DataBase_Write(STORAGE_ROMADDR,(u32 *)(&SetCurrent),sizeof(Setting));	
  if(status)
    return 1;
  else 
    return 0;     	
}


void BusCalling_Process(void)
{
  bool result=0;
  
  if(SlaveReceptionComplete)
  {
    switch(RxBuffer[2])
    {
    case AI_COF:
      result=AiCoefProcess(RxBuffer[3]);
      //if(result) 
      //{
      //  COEF_INTFLAG=0x01;
      //}
      break;
      
    case AI_CALB:
      if(AI_CALB==RxBuffer[3])
        result=AiCalibration();
      break;
      
    default:
      break;	          
    }	
    
    SlaveReceptionComplete=0; 
    memset(RxBuffer,0,8);
  }
  
}
void Deal_Readed(void)
{
  AIresponse(AinQuerry_index++%MEANUM) ;
}

void I2CHW_Maintain(void)
{
  __IO uint32_t Timeout;
  __IO uint32_t temp = 0;
  uint16_t counter;
  
  counter = GetBusCount();
  
  temp=I2C1->SR2;
  if(temp&0x0002) //总线BUSY
  {
    counter++;
    SetBusCount(counter);
  }
  else
  {
    if(counter != 0)
    {
      counter = 0;
      SetErrorCount(0);
    }
    GPIO_WriteBit(I2C_RESET_LED,  Bit_SET); 
  }
  
  //tyh:20130730 i2c在指定次数内没有复位成功, 重启AI板
  if( GetErrorCount() > I2C_BUS_ERROR_MAX_COUNT )
  {
    GPIO_WriteBit(ALARM_LED,  Bit_RESET);
    Timeout=0x7fff;
    while(Timeout--); 
    
    MeaTab[MEANUM-1]++; //记录当前复位次数
    DataBase_Write(BACKUPS_ROMADDR, (u32*)(&MeaTab[MEANUM-1]), sizeof(float)); //将复位次数存入flash
    
    NVIC_SystemReset();
  }
  
  if(counter > 0x002b) //7s
  {
    I2C_Cmd(I2C1,DISABLE);
    I2C_Cmd(I2C1,ENABLE);
    I2CHW_Reset();
    
    GPIO_WriteBit(I2C_RESET_LED,  Bit_RESET); 
    
    //tyh:20130730 总线复位次数加"1"，
    SetErrorCount( GetErrorCount()+1 );
    
    SetBusCount(0);
  }
}

/*------------------------------End of File------------------------------------------*/
