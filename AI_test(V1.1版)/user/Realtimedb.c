#ifndef _REALTIMEDB_H_
#define _REALTIMEDB_H_
#include "stm32f10x_flash.h"
#include "Realtimedb.h"
#include "BaseBus_Protocol.h"

#define   FLASH_PAGE_SIZE         0x400         // 1K per page 
#define   FLASH_WAIT_TIMEOUT      100000 
#define   MEMCHECK(a,b,c,d)  ((a!=0xFF&&b!=0xFF&&c!=0xFF&&d!=0xFF)?1:0)

//extern uint16_t Amp_Coef[10];
extern float MeaTab[MEANUM];

Setting SetCurrent={1000,1000,1000,1000,1000,1000,1000,1000,1000};

u8 DataBase_Write(u32 FLASH_Offset, u32 *Writebuff, u16 Transfer_Length) 
{ 
  u16 i;
  u8 status=0x01;
  
  if(FLASH_WaitForLastOperation(FLASH_WAIT_TIMEOUT)!=FLASH_TIMEOUT)
  { 
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR); 
  } 
  
  FLASH_ErasePage(FLASH_Offset); 
  
  for(i=0;i<Transfer_Length;i+=4)
  { 
    if(FLASH_WaitForLastOperation(FLASH_WAIT_TIMEOUT)!=FLASH_TIMEOUT)
    { 
      FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR); 
    } 
    if(FLASH_COMPLETE==FLASH_ProgramWord(FLASH_Offset + i , Writebuff[i>>2])) 
    {
      status&=0x01;
    } 
    else
    {
      status&=0x00;
    }  
  }
  
  return status;    
}   

u8 DataBase_read(u32 FLASH_Offset, u32 *Readbuff, u16 Receive_Length)
{
  u16 i;
  for(i=0;i<Receive_Length;i+=4)
  { 
    Readbuff[i>>2] = ((vu32*)(FLASH_Offset))[i>>2]; 	//定义volatile意义何在？
  } 
  
  return 0;
}

void DataBase_Init(void)
{
  char p[4];
  
  FLASH_Unlock();
  
  if(MEMCHECK(*((u8*)STORAGE_ROMADDR),*((u8*)STORAGE_ROMADDR+1),*((u8*)STORAGE_ROMADDR+2),*((u8*)STORAGE_ROMADDR+3)))
  {
    DataBase_read(STORAGE_ROMADDR,(u32*)(&SetCurrent),sizeof(Setting));
    //	memcpy(Amp_Coef,&SetCurrent,24);
    
    //tyh:20130730 读取AI复位总数
    DataBase_read(BACKUPS_ROMADDR, (u32*)(&MeaTab[MEANUM-1]), sizeof(float));
    memcpy(p, &MeaTab[MEANUM-1], sizeof(float));
    if((p[0] == 0xff)&&(p[1] == 0xff)&&(p[2] == 0xff)&&(p[3] == 0xff))
      MeaTab[MEANUM-1] = 0;
  }
}
#endif