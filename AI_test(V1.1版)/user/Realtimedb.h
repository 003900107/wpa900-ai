#ifndef _REALTIMEDB_
#define _REALTIMEDB_

#define STORAGE_ROMADDR 0x803D000
#define BACKUPS_ROMADDR 0x803E000

typedef struct tag_SETTING
{
	    u16 ChannelCoef[9];
        //¡­¡­¡­¡­¡­
}Setting;

u8 DataBase_Write(u32 FLASH_Offset, u32 *Writebuff, u16 Transfer_Length) ;
u8 DataBase_read(u32 FLASH_Offset, u32 *Readbuff, u16 Receive_Length);
void DataBase_Init(void);
#endif