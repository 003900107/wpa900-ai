#ifndef BASEBUSPROTCOL
#define BASEBUSPROTCOL

#include <stdbool.h>
#define AI_QRY 0xC1
#define AI_RES 0XC2
#define AI_SET 0XC4
#define AI_COF 0XC3
#define AI_CALB 0XC6

#define DMA	1
#define POLLING	2
#define INTERRUPT 3
#define I2C_METHOD DMA

#define MEMBLKCP  1
#define SINGLEBYTE  2
#define MEA_UPDATE_METHOD MEMBLKCP

#define CR1_PE_Set              ((uint16_t)0x0001)
#define CR1_PE_Reset            ((uint16_t)0xFFFE)
#define MEANUM      24
#define PTR_F2I(x) unsigned char*(x)

void _I2C1_EV_IRQHandler(void);
void AIresponse(unsigned char AiSeq);
void i2c_buffer_read(unsigned char *pBuffer, unsigned char SlaveAddr);
void i2c_buffer_write(unsigned char *pBuffer, unsigned char SlaveAddr);
void BusCalling_Process(void);
void Deal_Readed(void);
bool AiCoefProcess(unsigned char AiSeq);
bool slave_write(void);
bool slave_read(void);
void I2CHW_Maintain(void);
#endif
