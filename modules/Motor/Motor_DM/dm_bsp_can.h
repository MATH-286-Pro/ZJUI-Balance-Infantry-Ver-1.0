#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "main.h"
#include "can.h"


typedef CAN_HandleTypeDef hcan_t;

void bsp_can_init(hcan_t* hcan);
void can_filter_init(hcan_t* hcan);
uint8_t canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive(hcan_t *hcan, uint16_t *recid, uint8_t *buf);
void can1_rx_callback(void);
void can2_rx_callback(void);

#endif /* __CAN_BSP_H_ */

