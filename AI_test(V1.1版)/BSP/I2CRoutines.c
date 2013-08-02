/**
BASEдAI������ START -> ������ַ -> �� ACK -> (�������� -> �� ACK  (ѭ��)) -> �� STOP �� �� START ������һ�δ���
��һ�����У�����SCL�ߣ���ֻ��ACKʱ��SDA�ߣ�����ʱ������SDA�ߡ�

BASE��AIд���� START -> �ӷ���ַ -> �� ACK -> (�ӷ����� -> ��ACK (ѭ��)) -> ���������һ���ֽ�ʱ��
�� NACK -> �� STOP �� �� START 
/* Includes ------------------------------------------------------------------*/
#include "I2CRoutines.h"
#include "BaseBus_Protocol.h"
#include "bsp.h"


extern __IO uint8_t SlaveReceptionComplete;
#if I2C_METHOD == SINGLEBYTE
extern unsigned char TxBuffer[8];
#endif
#if MEA_UPDATE_METHOD == MEMBLKCP
extern unsigned char TxBuffer[127];
#endif
extern unsigned char RxBuffer[8];
__IO uint8_t Tx_Idx=0;
__IO uint8_t Rx_Idx=0;

static uint16_t BUSBusyCounter=0;

//tyh:20130730 ���߸�λ������¼����������AI��
static uint16_t BUSErrorCounter=0;

uint8_t GetBusCount()
{
  return BUSBusyCounter;
}

void SetBusCount(uint8_t count)
{
  BUSBusyCounter = count;
}

//tyh:20130730 �������߸�λ����������
uint8_t GetErrorCount()
{
  return BUSErrorCounter;
}

//tyh:20130730 ��ȡ���߸�λ����������
void SetErrorCount(uint8_t count)
{
  BUSErrorCounter = count;
}


void I2CDMA_Set(uint8_t* pBuffer,uint32_t BufferSize, uint8_t Direction)
{  
  DMA_InitTypeDef  I2CDMA_InitStructure;
  
  I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
  I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;  
  I2CDMA_InitStructure.DMA_BufferSize = BufferSize;            
  I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  
  
  if (Direction == I2C_DIRECTION_TX)
  {
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    
    DMA_Cmd(DMA1_Channel6, DISABLE);
    DMA_Init(DMA1_Channel6, &I2CDMA_InitStructure);
    DMA_Cmd(DMA1_Channel6, ENABLE);
  }
  else 
  {
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    
    DMA_Cmd(DMA1_Channel7, DISABLE);
    DMA_Init(DMA1_Channel7, &I2CDMA_InitStructure);
    DMA_Cmd(DMA1_Channel7, ENABLE);
  } 
}

#if I2C_METHOD == DMA
void _I2C1_EV_IRQHandler(void)
{
  __IO uint32_t SR1Register =0;
  __IO uint32_t SR2Register =0;
  
  SR1Register = I2C1->SR1;
  /*ע�⣬�������ȡSR1�Ĵ����󣬶�SR2�Ĵ����Ķ����������ADDRλ�����ADDR��ai�����ٿ���scl��
  ����Ҫ��SR2��ȡǰ�����ù���*/
  if ((SR1Register & 0x0002) == 0x0002)
  {
    
    if(SR1Register & 0x0080)
    {
      Deal_Readed();
      I2CDMA_Set(TxBuffer,/*98*/102,I2C_DIRECTION_TX);
    }
    else
      I2CDMA_Set(RxBuffer,8,I2C_DIRECTION_RX);
  }
  
  /* If STOPF =1: EV4 (Slave has detected a STOP condition on the bus */
  SR2Register = I2C1->SR2;
  if ( SR1Register & 0x0010)
  {
    I2C1->CR1 |= CR1_PE_Set;
    BUSBusyCounter=0; //reset busErrNum    
  }
  
//  /*bus busy*/
//  if(SR2Register & 0x0002)
//  {
//    BUSBusyCounter++;
//  }  
}

#else
void _I2C1_EV_IRQHandler(void)
{
  
  __IO uint32_t SR1Register =0;
  __IO uint32_t SR2Register =0;
  /* Read the I2C1 SR1 and SR2 status registers */
  SR1Register = I2C1->SR1;
  SR2Register = I2C1->SR2; //��һ�ν��ж�ʱ��ADDR
  
  /* If I2C1 is slave (MSL flag = 0) */
  if ((SR2Register &0x0001) != 0x0001)
  {
    /* If ADDR = 1: EV1 */
    if ((SR1Register & 0x0002) == 0x0002)
    {
      Deal_Readed();
      /* Clear SR1Register and SR2Register variables to prepare for next IT */
      SR1Register = 0;
      SR2Register = 0;
      /* Initialize the transmit/receive counters for next transmission/reception
      using Interrupt  */
      
      Tx_Idx = 0;
      Rx_Idx = 0;
    }
   
    /* If TXE = 1: EV3 */
    if ((SR1Register & 0x0080) == 0x0080)
    {
      /* Write data in data register */
      if(Tx_Idx</*98*/102)  //tyh:20130730
        I2C1->DR = TxBuffer[Tx_Idx++];	 //дDR��TXE
      SR1Register = 0;
      SR2Register = 0;
    }
   
    /* If RXNE = 1: EV2 */
    if ((SR1Register & 0x0040) == 0x0040)
    {
      /* Read data from data register */
      RxBuffer[Rx_Idx++] = I2C1->DR;
      SR1Register = 0;
      SR2Register = 0;
      if(8==Rx_Idx) 
        SlaveReceptionComplete = 1;
      
    }
    
    /* If STOPF =1: EV4 (Slave has detected a STOP condition on the bus */
    if (( SR1Register & 0x0010) == 0x0010)
    {
      I2C1->CR1 |= CR1_PE_Set;
      SR1Register = 0;
      SR2Register = 0;
    }
  } /* End slave mode */
  
  //����쳣���� tyh 20130731
  if( (SR1Register != 0)&&(SR2Register != 0) )
  {
    //��λI2C
    I2CHW_Reset();
  }  
}
#endif

void _I2C1_ER_IRQHandler(void)
{
  
  __IO uint32_t SR1Register =0;
  __IO uint32_t SR2Register =0;  
  
  /* Read the I2C1 status register */
  SR1Register = I2C1->SR1;
  SR2Register = I2C1->SR2; //��һ�ν��ж�ʱ��ADDR
  
  /* If AF = 1 */
  if ((SR1Register & 0x0400) == 0x0400)
  {
    I2C1->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  
  /* If ARLO = 1 */
  if ((SR1Register & 0x0200) == 0x0200)
  {
    I2C1->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  
  /* If BERR = 1 */
  if ((SR1Register & 0x0100) == 0x0100)
  {
    I2C1->SR1 &= 0xFEFF;
    SR1Register = 0;
  }
  
  /* If OVR = 1 */ 
  if ((SR1Register & 0x0800) == 0x0800)
  {
    I2C1->SR1 &= 0xF7FF;
    SR1Register = 0;
  }
  
  //����쳣���� tyh 20130731
  if(SR1Register != 0)
  {
    //��λI2C
    I2CHW_Reset();
  }  
  
//  /*bus busy*/
//  if(SR2Register & 0x0002)
//  {
//    BUSBusyCounter++;
//  }  
}

void _DMA1_Channel7_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC7))
  {
    DMA_ClearITPendingBit(DMA1_IT_GL7);
    DMA_Cmd(DMA1_Channel7, DISABLE);
    SlaveReceptionComplete = 1;
  }
}

/**
* @}
*/


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
