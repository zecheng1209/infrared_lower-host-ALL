#include "RobStride2.h"

static inline uint16_t float_to_uint16(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    if (offset < 0)
        offset = 0;
    if (offset > span)
        offset = span;
    return (uint16_t)((offset * 65535.0f) / span);
}

void RobStrideInit(RobStride_t *device, CAN_HandleTypeDef *hcan, uint32_t id, RobStrideType type)
{
    device->hcan = hcan;
    device->motor_id = id;
    device->host_id = HOST_ID;
    device->type = type;
}

uint32_t RobStrideEnable(RobStride_t *device)
{
    uint8_t buf[8] = {0};
    return RobStrideSend(device, (3 << 24) | (device->host_id << 8) | (device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideDisable(RobStride_t *device, uint8_t clear_error)
{
    uint8_t buf[8] = {clear_error};
    return RobStrideSend(device, (4 << 24) | (device->host_id << 8) | (device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideResetAngle(RobStride_t *device)
{
    uint8_t buf[8] = {1};
    return RobStrideSend(device, (6 << 24) | (device->host_id << 8) | (device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideGet(RobStride_t *device, uint16_t cmd)
{
    uint8_t buf[8] = {0};
    *((uint16_t *)buf) = cmd;
    uint32_t Extid = (17 << 24) | (device->host_id << 8) | (device->motor_id);
    return RobStrideSend(device, Extid, buf); // 发送使能命令
}

uint32_t RobStrideSetMode(RobStride_t *device, RobStrideMode mode)
{
    uint8_t buf[8] = {0};
    buf[0] = (uint8_t)PARAM_RUN_MODE;
    buf[1] = (uint8_t)(PARAM_RUN_MODE >> 8);
    buf[4] = (uint8_t)mode;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf); // 发送使能命令
}

uint32_t RobStrideMotionControl(RobStride_t *device, uint8_t motor_id, float torque, float angle, float omega, float kp, float kd)
{
    if (device == NULL)
    {
        return 1; // 无效设备，返回错误码
    }

    uint8_t buf[8] = {0};
    float tq_min, tq_max;       // 扭矩范围
    float omega_min, omega_max; // 角速度范围
    float kp_min, kp_max;       // Kp范围
    float kd_min, kd_max;       // Kd范围

    // 根据电机型号设置参数量程
    switch (device->type)
    {
    case RobStride_01:
        tq_min = -17.0f;
        tq_max = 17.0f;
        omega_min = -44.0f;
        omega_max = 44.0f;
        kp_min = 0.0f;
        kp_max = 500.0f;
        kd_min = 0.0f;
        kd_max = 5.0f;
        break;
    case RobStride_02:
        tq_min = -60.0f;
        tq_max = 60.0f;
        omega_min = -44.0f;
        omega_max = 44.0f;
        kp_min = 0.0f;
        kp_max = 5000.0f;
        kd_min = 0.0f;
        kd_max = 100.0f;
        break;
    case RobStride_03:
        tq_min = -60.0f;
        tq_max = 60.0f;
        omega_min = -20.0f;
        omega_max = 20.0f;
        kp_min = 0.0f;
        kp_max = 5000.0f;
        kd_min = 0.0f;
        kd_max = 100.0f;
        break;
    case RobStride_04:
        tq_min = -120.0f;
        tq_max = 120.0f;
        omega_min = -15.0f;
        omega_max = 15.0f;
        kp_min = 0.0f;
        kp_max = 5000.0f;
        kd_min = 0.0f;
        kd_max = 100.0f;
        break;
    default:
        return 2; // 无效型号，返回错误码
    }

    // 量化参数（角度范围固定为-4π~4π）
    uint16_t pos_u = float_to_uint16(angle, -4.0f * M_PI, 4.0f * M_PI);
    uint16_t vel_u = float_to_uint16(omega, omega_min, omega_max);
    uint16_t kp_u = float_to_uint16(kp, kp_min, kp_max);
    uint16_t kd_u = float_to_uint16(kd, kd_min, kd_max);
    uint16_t tq_u = float_to_uint16(torque, tq_min, tq_max);

    // 填充数据区（高字节在前，大端序）
    buf[0] = (pos_u >> 8) & 0xFF;
    buf[1] = pos_u & 0xFF;
    buf[2] = (vel_u >> 8) & 0xFF;
    buf[3] = vel_u & 0xFF;
    buf[4] = (kp_u >> 8) & 0xFF;
    buf[5] = kp_u & 0xFF;
    buf[6] = (kd_u >> 8) & 0xFF;
    buf[7] = kd_u & 0xFF;

    // 构造29位扩展ID：Bit28~24=0x1，Bit23~8=扭矩量化值，Bit7~0=电机ID
    uint32_t ExtID = (0x1 << 24) | ((uint32_t)tq_u << 8) | (motor_id & 0xFF);

    // 发送CAN帧（假设RobStrideSend返回0为成功，其他为错误）
    return RobStrideSend(device, ExtID, buf);
}

uint32_t RobStrideTorqueControl(RobStride_t *device, float req)
{
    uint8_t buf[8] = {0};
    *((uint16_t *)&buf[0]) = PARAM_IQ_REF;
    *((float *)&buf[4]) = req;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
}

uint32_t RobStrideSpeedControl(RobStride_t *device, float vel)
{
    uint8_t buf[8] = {0};
    *((uint16_t *)&buf[0]) = PARAM_SPD_REF;
    *((float *)&buf[4]) = vel;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
}

uint32_t RobStridePositionControl(RobStride_t *device, float pos)
{
    uint8_t buf[8] = {0};
    *((uint16_t *)&buf[0]) = PARAM_LOC_REF;
    *((float *)&buf[4]) = pos;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
}

uint32_t RobStrideSetVelPID(RobStride_t *device, float kp, float ki)
{
    uint8_t buf[8];
    uint32_t ret;
    *((uint32_t *)(&buf[0])) = PARAM_SPD_KP;
    *((float *)&buf[4]) = kp;
    ret = RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);

    *((uint32_t *)(&buf[0])) = PARAM_SPD_KI;
    *((float *)&buf[4]) = ret | RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
    return 0;
}

uint32_t RobStrideSetLocPID(RobStride_t *device, float kp)
{
    uint8_t buf[8];
    *((uint32_t *)(&buf[0])) = PARAM_LOC_KP;
    *((float *)&buf[4]) = kp;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
}

uint32_t RobStrideSetCurPID(RobStride_t *device, float kp, float ki)
{
    // TODO:设置电流环PID（一般不需要，所以就不写了）
    return 0;
}

uint32_t RobStrideSetVelLimit(RobStride_t *device, float vel)
{
    uint8_t buf[8] = {0};
    *((uint16_t *)&buf[0]) = PARAM_LIMIT_SPD;
    *((float *)&buf[4]) = vel;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
}

uint32_t RobStrideSetCurLimit(RobStride_t *device, float cur)
{
    uint8_t buf[8] = {0};
    *((uint16_t *)&buf[0]) = PARAM_LIMIT_CUR;
    *((float *)&buf[4]) = cur;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
}

uint32_t RobStrideSetTorqueLimit(RobStride_t *device, float torque)
{
    uint8_t buf[8] = {0};
    *((uint16_t *)&buf[0]) = PARAM_LIMIT_TORQUE;
    *((float *)&buf[4]) = torque;
    return RobStrideSend(device, (18 << 24) | (device->host_id << 8) | (device->motor_id), buf);
}

uint32_t RobStrideRecv_Handle(RobStride_t *device, CAN_HandleTypeDef *hcan, uint32_t ID, uint8_t *buf)
{
    if (hcan->Instance != device->hcan->Instance)
        return 0;
    if (((ID >> 8) & 0x00FF) != device->motor_id)
        return 0;
    uint32_t type = ID >> 24;
    if (type == 0)
    {
        // TODO:尝试获取设备信息以检查通信
    }
    else if (type == 2) // 存储反馈的数据
    {
        float omega_scale, torque_scale;

        // 提取公共部分：反馈ID和角度（所有型号角度范围一致）
        device->state.feedback = ID >> 16;
        device->state.rad = (float)(((DEPACK_U8_TO_U16_FLIP(buf[0], buf[1])) - 32767.0f) * 4.0f * M_PI / 32767.0f);

        // 根据设备类型设置角速度和扭矩的缩放系数
        switch (device->type)
        {
        case RobStride_01:
            omega_scale = 44.0f;  // 角速度范围 ±44
            torque_scale = 17.0f; // 扭矩范围 ±17
            break;
        case RobStride_02:
            omega_scale = 44.0f;  // 角速度范围 ±44
            torque_scale = 60.0f; // 扭矩范围 ±60
            break;
        case RobStride_03:
            omega_scale = 20.0f;  // 角速度范围 ±20
            torque_scale = 60.0f; // 扭矩范围 ±60
            break;
        case RobStride_04:
            omega_scale = 15.0f;   // 角速度范围 ±15
            torque_scale = 120.0f; // 扭矩范围 ±120
            break;
        default:
            // 处理未知型号（可选：设置默认值或报错）
            omega_scale = 0.0f;
            torque_scale = 0.0f;
            return 1; // 返回错误码
        }

        // 统一计算角速度、扭矩和温度（所有型号温度计算方式一致）
        device->state.omega = (float)(((DEPACK_U8_TO_U16_FLIP(buf[2], buf[3])) - 32767) * omega_scale / 32767);
        device->state.torque = (float)(((DEPACK_U8_TO_U16_FLIP(buf[4], buf[5])) - 32767) * torque_scale / 32767);
        device->state.temperature = DEPACK_U8_TO_U16_FLIP(buf[6], buf[7]) * 0.1f;
    }
    else if (type == 21) // 复制错误信息到电机状态结构体
    {
        device->state.error = *((uint64_t *)buf);
    }
    else if (type == 17) // 请求读取单个参数时的处理
    {
        uint16_t cmd = DEPACK_U8_TO_U16(buf[0], buf[1]);
        switch (cmd)
        {
        case PARAM_RUN_MODE: // 当前控制模式
            device->param.run_mode = buf[4];
            break;
        case PARAM_LIMIT_TORQUE: // 力矩限制
            device->param.torque_limit = *((float *)&buf[4]);
            break;
        case PARAM_CUR_KP:
            device->param.cur_kp = *((float *)&buf[4]);
            break;
        case PARAM_CUR_KI:
            device->param.cur_ki = *((float *)&buf[4]);
            break;
        case PARAM_CUR_FILT_GAIN:
            device->param.cur_filt_gain = *((float *)&buf[4]);
            break;
        case PARAM_LIMIT_SPD:
            device->param.limit_spd = *((float *)&buf[4]);
            break;
        case PARAM_LIMIT_CUR:
            device->param.limit_cur = *((float *)&buf[4]);
            break;
        case PARAM_LOC_KP:
            device->param.loc_kp = *((float *)&buf[4]);
            break;
        case PARAM_SPD_KP:
            device->param.spd_kp = *((float *)&buf[4]);
            break;
        case PARAM_SPD_KI:
            device->param.spd_ki = *((float *)&buf[4]);
            break;
        case PARAM_MECH_POS:
            device->state.rad = *((float *)&buf[4]);
            break;
        case PARAM_MECH_VEL:
            device->state.omega = *((float *)&buf[4]);
            break;
        default:
            break;
        }
    }
    return 1;
}

uint32_t RobStrideSend(RobStride_t *device, uint32_t ExtID, uint8_t *buf)
{
    CAN_TxHeaderTypeDef head;
    uint32_t mailbox;
    head.StdId = 0;
    head.ExtId = ExtID;
    head.IDE = CAN_ID_EXT;
    head.RTR = CAN_RTR_DATA;
    head.DLC = 8;
    head.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(device->hcan, &head, buf, &mailbox);
}
