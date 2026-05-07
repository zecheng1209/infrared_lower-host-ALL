#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "RobStride2.h"
#include "motorEx.h"

#pragma pack(1)

typedef struct{
    float rad;
    float omega;
    float torque;
}Motor_t;

typedef struct{
    int pack_type;
    Motor_t joints[6];
}Arm_t;

#pragma pack()

typedef struct{
    RobStride_t   Rs_motor;
    Motor2006Ex_t RM_motor;
    float pos_offset;
    int8_t inv_motor;

    float exp_rad;
    float exp_omega;
    float exp_torque;

    PID pos_pid;
    PID vel_pid;
}Joint_t;


void Motor_Drive(void *param);
void MotorSendTask(void *param);// 将电机的数据发送到PC上
void MotorRecTask(void *param);// 从PC接收电机的期望值

#endif
