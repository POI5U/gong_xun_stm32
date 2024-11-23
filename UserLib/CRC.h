#ifndef __CRC_H__
#define __CRC_H__

#include "stdint.h"

uint16_t CRC16(uint8_t *puchMsg,uint8_t usDataLen);
void CRC16_ModbusAdd(uint8_t *puchMsg,uint8_t ALL_Long);
int8_t CRC16_modbuscheck(uint8_t *puchMsg, uint8_t ALL_Long);

uint8_t CRC8(uint8_t *p, uint8_t len);

#endif
