#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

QueueHandle_t kUsbRecvQueue;
QueueHandle_t kUsbRecvSemphr;

TaskHandle_t kUsbRecvTaskHandle;

static Recv_finished_cb_t Recv_Finished_Cb;
static RecvBufferOverflow_cb_t Recv_Overflow_cb;
static void* kUserData;


int32_t buffer_index = 0;
uint32_t current_cdc_pack_size=0;

static void USB_RecvTask(void *param)
{
    while (1)
    {
        buffer_index=0;
				USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
        do{
						USBD_CDC_ReceivePacket(&hUsbDeviceFS);
						xQueueReceive(kUsbRecvQueue, &current_cdc_pack_size, portMAX_DELAY);
            buffer_index=buffer_index+current_cdc_pack_size;
            if(buffer_index+64>USB_CDC_RECV_BUFFER_SIZE)        //如果下一次接收可能溢出，那么通知应用层
            {
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS+buffer_index-current_cdc_pack_size);
                buffer_index=-1;
                if(Recv_Overflow_cb)
                    Recv_Overflow_cb(kUserData);
            }
            else
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS+buffer_index);     //防止缓冲区溢出，重设接收缓冲区地址 
        }while(current_cdc_pack_size==64);
        if(buffer_index>0)
           Recv_Finished_Cb(UserRxBufferFS, buffer_index);
    }
}

// 初始化USB CDC应用层
void USB_CDC_Init(Recv_finished_cb_t recv_cb,RecvBufferOverflow_cb_t recv_overflow_cb,void* user_data)
{
    Recv_Finished_Cb = recv_cb;
    Recv_Overflow_cb = recv_overflow_cb;
    kUserData = user_data;

    kUsbRecvQueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(USB_RecvTask, "usb_cdc_recv", 128, NULL, 6, &kUsbRecvTaskHandle);
}

void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len)
{
    BaseType_t pxHigherPriorityTaskWoken;
    xQueueSendFromISR(kUsbRecvQueue, Len, &pxHigherPriorityTaskWoken);
		//current_cdc_pack_size=*Len;
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
