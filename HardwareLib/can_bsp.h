#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"
#include "string.h"


typedef struct Can_Frame{
    uint16_t frame_id;
    uint8_t frame_data[8];
    uint8_t data_len;
    
}Can_Frame;


extern Can_Frame Can1_Rx;
extern uint8_t Can1_RxSinge;



void can_bsp_init(void);

void can_filter_init(void);

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);

void fdcan1_rx_callback(void);

void fdcanx_send_frame(FDCAN_HandleTypeDef *hfdcan, Can_Frame* send_frame);

int8_t fdcanx_receive_frame(FDCAN_HandleTypeDef *hfdcan, Can_Frame* frame_loc);

void can_delay_us(uint32_t us);


#endif /* __CAN_BSP_H_ */


