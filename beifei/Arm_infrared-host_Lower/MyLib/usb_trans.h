#ifndef __USB_TRANS_H__
#define __USB_TRANS_H__


#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#define USB_CDC_RECV_BUFFER_SIZE	512

typedef struct CDC_SendReq_t CDC_SendReq_t;
typedef struct CDC_RecvReq_t CDC_RecvReq_t;


typedef void(*Recv_finished_cb_t)(uint8_t *src,uint16_t size);                    //数据包接收完成回调
typedef void(*RecvBufferOverflow_cb_t)(void* user_data);                                     //接收缓冲区溢出回调


extern QueueHandle_t kUsbRecvQueue;
extern QueueHandle_t kUsbSendsemphr;
extern QueueHandle_t kUsbSendReqQueue;

void USB_CDC_Init(Recv_finished_cb_t recv_cb,RecvBufferOverflow_cb_t recv_overflow_cb,void* user_data);
void CDC_RecvCplt_Handler(uint8_t* Buf, uint32_t *Len);


#endif
