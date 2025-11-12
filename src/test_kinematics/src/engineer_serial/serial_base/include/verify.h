//
// Created by ljy on 19-7-8.
//

#ifndef INFANTRY_LJY_V1_VERIFY_H
#define INFANTRY_LJY_V1_VERIFY_H
#include "common.h"

u32 Verify_CRC16_Check_Sum(u8* pchMessage, u32 dwLength);
// void Append_CRC16_Check_Sum(u8* pchMessage, u32 dwLength);
u16 Get_CRC16_Check_Sum(u8* pchMessage, u32 dwLength, u16 wCRC);
#endif // INFANTRY_LJY_V1_VERIFY_H
