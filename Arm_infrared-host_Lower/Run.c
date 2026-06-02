#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "PID.h"
#include "math.h"
#include "Task_Init.h"
#include "infrared_host.h"
extern uint8_t ready;

Joint_t Joint[5];
int16_t can_buf[4] = {0};

TaskHandle_t Motor_Drive_Handle;

RobStride_t rs02={.hcan=&hcan2,.motor_id=0x03,.type= RobStride_02, };//注意02为03id

RobStride_t rs03={.hcan=&hcan2,.motor_id=0x02,.type= RobStride_03, };
//float rs03_torque =-0.0f,rs03_rad=-0.2f;

float M0 = 7.7f; //rs03水平位置前馈
float rs02_torque=0.0f,rs02_rad=1.65f;//rs02目标角

float rs03_torque=5.0f,rs03_rad=4.15f;//rs03目标角//4.15

///float angle = 0.0f;//目标角度，角度制
//   rs03_rad = angle / 57.3f+rs03_rad;

//float rs03_kp=170.0f,rs03_kd=34.0f,rs03_omega=0.2f;   //MotionControl
PID rs02_vel_pid;  // 速度环PID
PID rs02_pos_pid;  // 位置环PID
PID rs03_vel_pid;  // 速度环PID
PID rs03_pos_pid;  // 位置环PID

typedef enum {
    init,         // 初始化
	
    moving1,      // 行进
	
	  moving2,      //抓杆
    moving22,
	
	  moving3,      //对接
	  moving32,
	
//    stop,         // 停止
//    error         // 错误状态
} STATE;
STATE task_state = init;       // 02位置有要求，以近似初始化位置进行初始化，且初始状态需为该项，
                                          // 02会根据不同的初始位置定义不同的角度！！！
//STATE task_state = moving1;
//STATE task_state = moving2;
//STATE task_state = moving3;


void rs02_PID_Init() {
  
	rs02_vel_pid.Kp = 3.0f;
	rs02_vel_pid.Ki = 0.0f;
	rs02_vel_pid.Kd = 1.0f;
	rs02_vel_pid.limit = 4.0f;
	rs02_vel_pid.output_limit = 25.0f;
	
	rs02_pos_pid.Kp = 11.5f;
	rs02_pos_pid.Ki = 0.0f;
	rs02_pos_pid.Kd = 1.0f;
	rs02_pos_pid.limit = 5.0f;
	rs02_pos_pid.output_limit = 2.3f;
}


void rs03_PID_Init() {
  
	rs03_vel_pid.Kp = 6.0f;
	rs03_vel_pid.Ki = 0.0f;
	rs03_vel_pid.Kd = 3.0f;
	rs03_vel_pid.limit = 20.0f;
	rs03_vel_pid.output_limit = 35.0f;
	
	rs03_pos_pid.Kp = 100.0f;
	rs03_pos_pid.Ki = 0.0f;
	rs03_pos_pid.Kd = 5.0f;
	rs03_pos_pid.limit = 15.0f;
	rs03_pos_pid.output_limit = 5.0f;
}



void RampToTarget(float* var, float target, float step);

void Motor_Drive(void *param)
{

	TickType_t Last_wake_time = xTaskGetTickCount();
	rs02_PID_Init();
	rs03_PID_Init();
  ///////////////RobStrideResetAngle(&rs03);//重置角度

	for(;;)
	{   
		  float curr_rad02 = rs02.state.rad;
			float curr_omega02 = rs02.state.omega;
			
			float curr_rad03 = rs03.state.rad;
			float curr_omega03 = rs03.state.omega;
		  switch(task_state) {
            case init:
                RampToTarget(&rs02_rad, 0.3f, 0.001f); // 使用RampToTarget平滑过渡到目标位置
                RampToTarget(&rs03_rad, 4.17f, 0.001f);
                break;
            case moving1:                            //行进
                RampToTarget(&rs02_rad, -1.7f, 0.01f);
                RampToTarget(&rs03_rad, 4.6f, 0.01f);
                break;
            case moving2:                            //抓杆
                RampToTarget(&rs02_rad, 1.47f, 0.01f);
                RampToTarget(&rs03_rad, 4.25f, 0.01f);
						    break;
						case moving22:                            //抓杆2
                RampToTarget(&rs02_rad, 1.50f, 0.01f);
                RampToTarget(&rs03_rad, 4.30f, 0.01f);
                break;
            case moving3:                            //对接
                RampToTarget(&rs02_rad, -0.25f, 0.01f);
                RampToTarget(&rs03_rad, 4.7f, 0.01f);
                break;
						case moving32:                            //对接2
                RampToTarget(&rs02_rad, -0.25f, 0.01f);
                RampToTarget(&rs03_rad, 4.64f, 0.01f);
                break;
            default:
                break;
            }
  float M1 = M0 * cos(rs03.state.rad- 4.12f);//rs03初始水平的角度
	
	PID_Control(curr_rad02, rs02_rad, &rs02_pos_pid);
  float target_omega02 = rs02_pos_pid.pid_out;
	PID_Control(curr_omega02,target_omega02 , &rs02_vel_pid);	//target_omega02
  
	PID_Control(curr_rad03, rs03_rad, &rs03_pos_pid);
 	float target_omega03 = rs03_pos_pid.pid_out;
	PID_Control(curr_omega03,target_omega03 , &rs03_vel_pid);	//target_omega03	
	
//rs02_torque = rs02_vel_pid.pid_out;
	rs03_torque = M1 + rs03_vel_pid.pid_out;
	rs02_torque = rs02_vel_pid.pid_out;
  RobStrideTorqueControl(&rs02,rs02_torque);
  RobStrideTorqueControl(&rs03,rs03_torque);
      
	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
	}
}

TaskHandle_t Motor_RM_Handle;
Motor3508Ex_t RM_3508;
Motor2006Ex_t RM_2006;
float Expect_3508 = 0.0f;
float Expect_2006 = 0.0f;
int16_t Can2_TxData[4] = {0};
int16_t Can1_TxData[4] = {0};

void Motor_RM(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	
	RM_3508.ID = 0x201;
	RM_3508.hcan = &hcan1;
	
	RM_3508.pos_pid.Kp = 0.4f;
	RM_3508.pos_pid.Ki = 0.0f;
	RM_3508.pos_pid.Kd = 0.005f;
	RM_3508.pos_pid.limit = 1000.0f;
	RM_3508.pos_pid.output_limit = 5000.0f;
	
	RM_3508.vel_pid.Kp =18.0f;
	RM_3508.vel_pid.Ki = 0.0f;
	RM_3508.vel_pid.Kd = 0.1f;
	RM_3508.vel_pid.output_limit = 16384.0f;
	
	
	RM_2006.ID = 0x202;
	RM_2006.hcan = &hcan1;
	
	RM_2006.pos_pid.Kp = 1.0f; 
	RM_2006.pos_pid.Ki = 0.0f;
	RM_2006.pos_pid.Kd = 0.0f;
	RM_2006.pos_pid.limit = 0.0f;
	RM_2006.pos_pid.output_limit = 2000.0f;
	
	RM_2006.vel_pid.Kp =10.0f;
	RM_2006.vel_pid.Ki = 0.01f;
	RM_2006.vel_pid.Kd = 0.0f;
	RM_2006.vel_pid.output_limit = 16384.0f;
	
	for(;;)
	{
				  switch(task_state) {
            case init:
                // 在每个状态中使用 RampToTarget 进行平滑过渡
                RampToTarget(&Expect_3508, 0.0f, 1.0f);
                RampToTarget(&Expect_2006, 0.0f, 1.0f);
                break;
                                         //前几个状态无需3508和2006改变位置
            case moving1:                             //行进
                RampToTarget(&Expect_3508, 0.0f, 1.0f);
                RampToTarget(&Expect_2006, 0.0f, 1.0f);
                break;
            case moving2:                             //抓杆
                RampToTarget(&Expect_3508, 0.0f, 1.0f);
                RampToTarget(&Expect_2006, 0.0f, 1.0f);
                break;
						case moving22:                             //抓杆2
                RampToTarget(&Expect_3508, 0.0f, 1.0f);
                RampToTarget(&Expect_2006, 0.0f, 1.0f);
                break;
            case moving3:                             //对接
                RampToTarget(&Expect_3508, -39000.0f, 80.0f);
                RampToTarget(&Expect_2006, 0.0f, 1.0f);
                break;
						case moving32:                             //对接
                RampToTarget(&Expect_3508, -39000.0f, 80.0f);
                RampToTarget(&Expect_2006, 0.0f, 1.0f);
                break;
            }
		PID_Control(RM_3508.actual_pos, Expect_3508, &RM_3508.pos_pid);
		PID_Control(RM_3508.motor.Speed, RM_3508.pos_pid.pid_out , &RM_3508.vel_pid);//RM_3508.pos_pid.pid_out
		
		PID_Control(RM_2006.actual_pos, Expect_2006, &RM_2006.pos_pid);
		PID_Control(RM_2006.motor.Speed, RM_2006.pos_pid.pid_out , &RM_2006.vel_pid);//RM_2006.pos_pid.pid_out
		Can1_TxData[0] = RM_3508.vel_pid.pid_out;//RM_3508.vel_pid.pid_out
		Can1_TxData[1] = RM_2006.vel_pid.pid_out;//RM_2006.vel_pid.pid_out
		//MotorSend(&hcan2 ,0x200, Can2_TxData);
		MotorSend(&hcan1 ,0x200, Can1_TxData);
		
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
	}
}
Arm_t arm_t;
SemaphoreHandle_t cdc_recv_semphr;
Arm_t arm_Rec_t;
uint16_t cur_recv_size;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		uint8_t rx_data[8];
		CAN_RxHeaderTypeDef rx_header;
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
			IR_OnCanRx(&rx_header, rx_data);
		}
		uint8_t buf[8];
		uint16_t ID = CAN_Receive_DataFrame(&hcan1, buf);
		Motor2006Recv(&RM_2006, &hcan1, ID, buf);
		Motor3508Recv(&RM_3508, &hcan1, ID, buf);
	}
	else if (hcan->Instance == CAN2)
	{
		uint8_t buf[8];
		uint16_t ID = CAN_Receive_DataFrame(&hcan2, buf);
		Motor2006Recv(&RM_2006, &hcan2, ID, buf);
		Motor3508Recv(&RM_3508, &hcan2, ID, buf);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		uint8_t buf[8];
		uint32_t ID = CAN_Receive_DataFrame(hcan, buf);	
	 	RobStrideRecv_Handle(&rs03, &hcan2, ID, buf);
		RobStrideRecv_Handle(&rs02, &hcan2, ID, buf);
		//Motor3508Recv(&RM_3508, &hcan2, ID, buf);
	 // Motor2006Recv(&RM_2006, &hcan2, ID, buf);
}
