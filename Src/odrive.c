/**
  ******************************************************************************
  * @file    odrive.c
  * @brief   ODrive 电机驱动：CAN2 收发 + 转速滤波
  ******************************************************************************
  * 【管哪两个电机】
  *   AXIS0 (NODE 0) → 动量轮（平衡用）
  *   AXIS1 (NODE 1) → 后轮  （驱动用）
  *   对应全局变量： v_momentum / v_rear_wheel（单位 RPM，供 main.c 上报）
  *
  * 【怎么控制】
  *   odrive_speed_ctrl(num, speed)  发 CAN 帧 MSG_SET_INPUT_VEL 设转速
  *   odrive_vel_callback(num)       发【远程帧】请求编码器估计值
  *   接收在 HAL_CAN_RxFifo0MsgPendingCallback() 里，只处理 CAN2。
  *
  * 【转速为什么是三取平均】
  *   CAN 回传的瞬时转速抖动大，用 fliter_speed[3] 环形缓冲取平均，
  *   相当于一个简单的滑动滤波 —— 便宜且有效。
  *
  * ⚠️【踩过的坑】CAN2 的过滤器 Bank 必须从 14 开始
  *    STM32F4 的 CAN1/CAN2 共用过滤器组，CAN2 属于"从机"，
  *    SlaveStartFilterBank 必须显式指定（见 odrive_canFilter_init）。
  ******************************************************************************
  */
#include "odrive.h"
#include "can.h"
#include <string.h>

OdirveTypeDef odrive;
int rx_packet_count = 0; 

// 【新增】定义全局变量，供 main.c 读取用于显示
// 单位：RPM (转/分钟)
volatile float v_momentum = 0.0f;    // 动量轮速度 (对应 AXIS0)
volatile float v_rear_wheel = 0.0f;  // 后轮速度   (对应 AXIS1)

void odrive_init(void)
{
    memset(&odrive, 0, sizeof(OdirveTypeDef));
    odrive_canFilter_init();
}

void odrive_canFilter_init(void)
{
    CAN_FilterTypeDef filter;
    
    // 【关键修复】确保 CAN1 和 CAN2 的时钟同时开启
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    filter.FilterActivation = ENABLE;
    
    // 强制使用 Bank 14 
    filter.FilterBank = 14; 
    
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    
    // 【全通模式】
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    
    // 【核心修复】明确告诉硬件：Bank 14 及之后的过滤器属于从机（CAN2）
    filter.SlaveStartFilterBank = 14; 

    HAL_CAN_ConfigFilter(&hcan2, &filter);
    
    // 启动并激活通知
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void odrive_speed_ctrl(unsigned char num, float speed)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 8;
    header.StdId = ((NODE_ID(num) << 5) | MSG_SET_INPUT_VEL);
    header.ExtId = 0;
    header.TransmitGlobalTime = DISABLE;

    memcpy(&data[0], &speed, 4);
    memset(&data[4], 0, 4);

    uint32_t ret;
    HAL_CAN_AddTxMessage(&hcan2, &header, data, &ret);
}

void odrive_vel_callback(unsigned char num)
{
    CAN_TxHeaderTypeDef header;
    uint8_t dummy_data[8] = {0};
    
    header.RTR = CAN_RTR_REMOTE; 
    header.IDE = CAN_ID_STD;
    header.DLC = 0; 
    header.StdId = ((NODE_ID(num) << 5) | MSG_GET_ENCODER_ESTIMATES);
    header.ExtId = 0;
    header.TransmitGlobalTime = DISABLE;

    uint32_t ret;
    HAL_CAN_AddTxMessage(&hcan2, &header, dummy_data, &ret);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t buf[8];
    
    // 明确判断实例是否为 CAN2
    if(hcan->Instance == CAN2)
    {
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, buf) == HAL_OK)
        {
            rx_packet_count++; 
            
            uint8_t cmd_id = header.StdId & 0x1F;
            uint8_t node_id = header.StdId >> 5;

            if (cmd_id == MSG_GET_ENCODER_ESTIMATES)
            {
                float temp_vel = 0;
                memcpy(&temp_vel, &buf[4], 4); 

                if(node_id == AXIS0_CAN_NODE_ID)
                {
                    // 动量轮数据处理
                    odrive.speed0_i = (odrive.speed0_i + 1) % 3;
                    odrive.fliter_speed0[odrive.speed0_i] = temp_vel;
                    odrive.now_speed0 = (odrive.fliter_speed0[0] + odrive.fliter_speed0[1] + odrive.fliter_speed0[2]) / 3.0f;
                    
                    // 【新增】更新全局变量，单位转为 RPM (原单位 turns/s)
                    v_momentum = odrive.now_speed0 * 60.0f;
                }
                else if(node_id == AXIS1_CAN_NODE_ID)
                {
                    // 后轮数据处理
                    odrive.speed1_i = (odrive.speed1_i + 1) % 3;
                    odrive.fliter_speed1[odrive.speed1_i] = temp_vel;
                    odrive.now_speed1 = (odrive.fliter_speed1[0] + odrive.fliter_speed1[1] + odrive.fliter_speed1[2]) / 3.0f;     
                    
                    // 【新增】更新全局变量，单位转为 RPM
                    v_rear_wheel = odrive.now_speed1 * 60.0f;
                }
            }
        }
    }
}