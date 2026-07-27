/**
 * @file    dm_motor.hpp
 * @brief   达妙(DM)电机驱动器通信协议封装
 *
 * @details 本文件定义了达妙系列电机的 CAN 总线通信接口类。
 *          通信采用 CAN 标准帧(11位ID), 每条 CAN 总线上最多挂载 255 台电机。
 *          协议特点:
 *          - 发送帧 ID = can_id(电机基地址) + mode_id(控制模式偏移), 其中 mode_id 高 4 bit 编码控制模式
 *          - 接收帧 ID = 0xFF - can_id, 即 mst_id, 由电机反馈数据时使用
 *          - 支持四种控制模式: MIT模式、位置-速度模式、速度模式、位置-速度-电流模式
 *          - 参数读写通过特殊帧(0x7FF)完成, data[2] 区分操作类型: 0x33读 / 0x55写 / 0xCC反馈请求
 *          - 控制指令数据区采用定点数编码, 将浮点参数线性映射到特定位数的整数
 * @note    #define 中的 KP/KD 范围需根据实际电机型号调整
 * @version 0.1
 */
#ifndef DM_MOTOR_HPP
#define DM_MOTOR_HPP

#include <cstdint>
#include "fdcan.h"

/* 需要根据实际电机型号更改 */
#define DM_KP_MIN   (0.0f)   /*!< MIT模式Kp最小值 */
#define DM_KP_MAX   (500.0f) /*!< MIT模式Kp最大值 */
#define DM_KD_MIN   (0.0f)   /*!< MIT模式Kd最小值 */
#define DM_KD_MAX   (5.0f)   /*!< MIT模式Kd最大值 */

/** 达妙电机控制模式枚举, 对应 CAN 发送帧 ID 的高 4 bit 偏移量
 *  - kMIT:          0x000 — MIT模式, 位置+速度+扭矩+Kp+Kd 全部通过 CAN 数据区编码传输
 *  - kPosVel:       0x100 — 位置-速度模式, 位置和速度以 float 格式发送
 *  - kVel:          0x200 — 速度模式, 仅发送速度参考值(4字节 float)
 *  - kPosVelCur:    0x300 — 位置-速度-电流模式, 位置(4B)+速度(2B)+电流(2B)组合编码
 */
enum class DMMotorModeID : uint16_t
{
    kMIT = 0x000,
    kPosVel = 0x100,
    kVel = 0x200,
    kPosVelCur = 0x300,
};

/** 达妙电机内部寄存器地址枚举, 用于 ReadReg() / WriteReg() 读写电机参数
 *  - 0~34:   系统保护参数与运行参数(过压/过流/过温保护, PID增益, 映射范围等)
 *  - 35~36:  CAN 配置参数(波特率代码, 子版本号)
 *  - 50~55:  标定参数(U相/V相偏置, 补偿因子, 角度偏移, 方向)
 *  - 80~81:  实时状态(电机位置, 输出轴位置)
 */
enum class DMMotorReg : uint8_t
{
    RID_UV_VALUE = 0,    // 低压保护值
    RID_KT_VALUE = 1,    // 扭矩系数
    RID_OT_VALUE = 2,    // 过温保护值
    RID_OC_VALUE = 3,    // 过流保护值
    RID_ACC		 = 4,    // 加速度
    RID_DEC		 = 5,    // 减速度
    RID_MAX_SPD	 = 6,    // 最大速度
    RID_MST_ID	 = 7,    // 反馈ID
    RID_ESC_ID	 = 8,    // 接收ID
    RID_TIMEOUT	 = 9,    // 超时警报时间
    RID_CMODE	 = 10,   // 控制模式
    RID_DAMP	 = 11,   // 电机粘滞系数
    RID_INERTIA  = 12,   // 电机转动惯量
    RID_HW_VER	 = 13,   // 保留
    RID_SW_VER	 = 14,   // 软件版本号
    RID_SN		 = 15,   // 保留
    RID_NPP		 = 16,   // 电机极对数
    RID_RS		 = 17,   // 电阻
    RID_LS		 = 18,   // 电感
    RID_FLUX	 = 19,   // 磁链
    RID_GR		 = 20,   // 齿轮减速比
    RID_PMAX	 = 21,   // 位置映射范围
    RID_VMAX	 = 22,   // 速度映射范围
    RID_TMAX	 = 23,   // 扭矩映射范围
    RID_I_BW	 = 24,   // 电流环控制带宽
    RID_KP_ASR	 = 25,   // 速度环Kp
    RID_KI_ASR	 = 26,   // 速度环Ki
    RID_KP_APR	 = 27,   // 位置环Kp
    RID_KI_APR	 = 28,   // 位置环Ki
    RID_OV_VALUE = 29,   // 过压保护值
    RID_GREF	 = 30,   // 齿轮力矩效率
    RID_DETA	 = 31,   // 速度环阻尼系数
    RID_V_BW	 = 32,   // 速度环滤波带宽
    RID_IQ_CL	 = 33,   // 电流环增强系数
    RID_VL_CL	 = 34,   // 速度环增强系数
    RID_CAN_BR	 = 35,   // CAN波特率代码
    RID_SUB_VER	 = 36,   // 子版本号
    RID_U_OFF	 = 50,   // u相偏置
    RID_V_OFF	 = 51,   // v相偏置
    RID_K1		 = 52,   // 补偿因子1
    RID_K2		 = 53,   // 补偿因子2
    RID_M_OFF	 = 54,   // 角度偏移
    RID_DIR		 = 55,   // 方向
    RID_P_M		 = 80,   // 电机位置
    RID_X_OUT	 = 81    // 输出轴位置
};

/** 达妙电机参数结构体, 存储从电机读取的全部配置信息
 *  - 浮点类型参数通过 ReadReg() 读取时直接按 float 解析
 *  - 整型参数通过 ReadReg() 读取时按 uint32_t 解析
 *  - PMAX/VMAX/TMAX 三个映射范围用于控制指令中定点数与浮点数的互相转换, 决定了分辨率
 */
typedef struct DMInf
{
	DMMotorReg read_reg_;           /*!< 当前正在读取的寄存器编号, 用于 ReadReg() 响应匹配 */

    float UV_Value_;                /*!< 低压保护阈值 */
    float KT_Value_;                /*!< 扭矩系数(Nm/A), 用于电流到扭矩的换算 */
    float OT_Value_;                /*!< 过温保护阈值(摄氏度) */
    float OC_Value_;                /*!< 过流保护阈值(A) */
    float ACC_;                     /*!< 加速度限制 */
    float DEC_;                     /*!< 减速度限制 */
    float MAX_SPD_;                 /*!< 最大速度限制 */
    uint32_t MST_ID_;               /*!< 电机反馈CAN ID, 由电机固件决定, 通常为 0xFF-can_id */
    uint32_t ESC_ID_;               /*!< 电机接收CAN ID, 即主机发送给电机的 ID 基址 */
    uint32_t TIMEOUT_;              /*!< CAN通信超时警报时间(ms), 超时后电机会自动停止 */
    uint32_t cmode_;                /*!< 当前控制模式代码, 对应 DMMotorModeID 枚举 */
    float    Damp_;                 /*!< 电机粘滞阻尼系数 */
    float    Inertia_;              /*!< 电机转子转动惯量(kg*m^2) */
    uint32_t hw_ver_;               /*!< 硬件版本号(保留字段) */
    uint32_t sw_ver_;               /*!< 固件软件版本号 */
    uint32_t SN_;                   /*!< 电机序列号(保留字段) */
    uint32_t NPP_;                  /*!< 电机极对数 */
    float    Rs_;                   /*!< 相电阻(ohm) */
    float    Ls_;                   /*!< 相电感(H) */
    float    Flux_;                 /*!< 永磁体磁链(Wb) */
    float    Gr_;                   /*!< 齿轮减速比 */
    float    PMAX_;                 /*!< 位置映射范围(rad), 控制指令中 16bit 定点数映射到的物理范围, 默认12.5 */
    float    VMAX_;                 /*!< 速度映射范围(rad/s), 控制指令中 12bit 定点数映射到的物理范围, 默认45.0 */
    float    TMAX_;                 /*!< 扭矩映射范围(Nm), 控制指令中 12bit 定点数映射到的物理范围, 默认12.0 */
    float    I_BW_;                 /*!< 电流环控制带宽(Hz) */
    float    KP_ASR_;               /*!< 速度环比例增益 Kp */
    float    KI_ASR_;               /*!< 速度环积分增益 Ki */
    float    KP_APR_;               /*!< 位置环比例增益 Kp */
    float    KI_APR_;               /*!< 位置环积分增益 Ki */
    float    OV_Value_;             /*!< 过压保护阈值(V) */
    float    GREF_;                 /*!< 齿轮力矩传递效率 */
    float    Deta_;                 /*!< 速度环阻尼系数 */
    float    V_BW_;                 /*!< 速度环滤波带宽(Hz) */
    float    IQ_cl_;                /*!< 电流环增强系数 */
    float    VL_cl_;                /*!< 速度环增强系数 */
    uint32_t can_br_;               /*!< CAN波特率代码 */
    uint32_t sub_ver_;              /*!< 固件子版本号 */
	float    u_off_;                /*!< U相电流偏置标定值 */
	float    v_off_;                /*!< V相电流偏置标定值 */
	float    k1_;                   /*!< 角度标定补偿因子1 */
	float    k2_;                   /*!< 角度标定补偿因子2 */
	float    m_off_;                /*!< 编码器角度偏移标定值 */
	float    dir_;                  /*!< 电机旋转方向(正反转) */
	float    p_m_;                  /*!< 电机端机械位置 */
	float    x_out_;                /*!< 减速器输出轴机械位置 */
} DMInf;

/** 达妙电机反馈数据结构体, 由 CanRxCallBack() 解析填充
 *  - 当 CAN 帧的 data[2]==0x33 时, 该帧为寄存器读取应答, 数值存入 DMInf
 *  - 其他情况为常规控制应答帧, pos/vel/tor 由定点数解算为浮点数
 */
typedef struct DMFeedback
{
    uint8_t flag_;          /*!< 反馈更新标志, 1 表示收到新数据 */
    int id_;                /*!< 反馈帧中携带的电机 ID */
    int state_;             /*!< 电机状态字 */
    float pos_rad_;         /*!< 当前机械位置(rad), 由 16bit 定点数解算 */
    float vel_radps_;       /*!< 当前机械速度(rad/s), 由 12bit 定点数解算 */
    float tor_output_Nm_;          /*!< 当前扭矩(Nm), 由 12bit 定点数解算 */
    float Kp_;              /*!< MIT模式当前使用的 Kp */
    float Kd_;              /*!< MIT模式当前使用的 Kd */
    float Tmos_;            /*!< MOS管温度(摄氏度), 直接映射 data[6] */
    float Tcoil_;           /*!< 线圈温度(摄氏度), 直接映射 data[7] */
} DMFeedback;

/** 达妙电机控制参数结构体, 调用各控制函数前需设置相应的目标值
 *  - 不同控制模式使用不同字段: MIT模式使用全部字段, PosVel模式仅用 pos/vel, Vel模式仅用 vel
 */
typedef struct
{
    DMMotorModeID mode_id_;         /*!< 当前控制模式 */
    float pos_set_rad_;             /*!< 目标位置(rad), 映射范围由 PMAX_ 决定 */
    float vel_set_radps_;           /*!< 目标速度(rad/s), 映射范围由 VMAX_ 决定 */
    float tor_set_Nm_;              /*!< MIT模式: 目标前馈扭矩(Nm) */
	float cur_set_A_;               /*!< PosVelCur模式: 目标电流(A) */
    float kp_set_;                  /*!< MIT模式: 位置环比例增益 */
    float kd_set_;                  /*!< MIT模式: 速度环微分增益 */
} DMCtrlParam;

/**
 * @class   DMMotor
 * @brief   达妙系列电机驱动器类
 *
 * @details 提供达妙电机在 CAN 总线上的完整控制接口, 包括:
 *          - 电机使能/失能/清零/设零等基础操作
 *          - 四种控制模式(MIT/PosVel/Vel/PosVelCur)的指令封装
 *          - 电机内部寄存器的读写(保护阈值, PID增益, 标定参数等)
 *          - CAN 反馈数据的解析
 *
 *          通信协议:
 *          - 发送帧 ID = can_id_ + mode_id_(偏移量)
 *          - 接收帧 ID = mst_id_ = 0xFF - can_id_ (遵循达妙协议, 互补关系)
 *          - 寄存器操作统一使用 ID=0x7FF, data[2] 区分操作类型
 *
 * @note    使用前需确保 BSP CAN 已初始化, 且在 CAN 接收回调中调用 CallDMCanRxCallBack()
 */
class DMMotor
{
public:
    DMMotor(FDCAN_HandleTypeDef &hfdcan, uint16_t can_id);
    ~DMMotor() = default;

    void EnableMotor(void);             /** 电机使能, 发送 0xFF..FC 使能指令 */
    void DisableMotor(void);            /** 电机失能, 发送 0xFF..FD 失能指令 */
    void ClearError(void);              /** 清除错误, 发送 0xFF..FB 错误清除指令 */
    void SetMecPosZero(void);           /** 将当前电机位置设为机械零点, 发送 0xFF..FE */
    void SetMotorMode(void);            /** 将 ctrl_param_.mode_id_ 通过写寄存器方式同步到电机 */
    void MitControl(void);              /** MIT模式控制, 将 pos/vel/tor/kp/kd 编码为定点数发送 */
    void PosVelControl(void);           /** 位置-速度模式控制, 直接发送 float 格式的 pos 和 vel */
    void VelControl(void);              /** 速度模式控制, 仅发送 4 字节 float 速度值 */
    void PosVelCurControl(void);        /** 位置-速度-电流模式, 编码 pos(4B)+vel(2B)+cur(2B) */
    void ClearCtrlParam(void);          /** 清零所有控制参数(不改变模式) */
    void ReadReg(DMMotorReg reg);       /** 通过 0x7FF 帧读取电机内部寄存器, 结果由 CanRxCallBack 解析 */
    void WriteReg(DMMotorReg reg, uint8_t value[4]); /** 通过 0x7FF 帧写入电机内部寄存器(4字节) */
    void ReadFeedback(void);            /** 发送 0x7FF+0xCC 请求电机上报反馈帧 */
    void SaveToFlash(void);             /** 将当前参数保存到电机 Flash, 掉电不丢失 */
    void CanRxCallBack(uint32_t can_id, const uint8_t *can_rxdata); /** CAN接收回调, 解析反馈数据 */

    uint16_t can_id_;                   /*!< 电机 CAN ID 基址(也作为发送帧的基址) */
    uint16_t mst_id_;                   /*!< 电机反馈帧 CAN ID(0xFF - can_id_), 用于接收匹配 */
    DMInf inf_;                         /*!< 电机参数信息, 通过 ReadReg 填充 */
    DMFeedback feedback_;               /*!< 电机实时反馈数据 */
    DMCtrlParam ctrl_param_;            /*!< 电机控制参数(目标值) */

private:
    void SendData(uint32_t can_std_id, uint8_t *data, uint32_t data_size);

    FDCAN_HandleTypeDef &hfdcan_;        /*!< 该电机固定使用的 FDCAN 外设 */
};

extern "C" {
void CallDMCanRxCallBack(DMMotor *ptr_dm, uint32_t can_ext_id, uint8_t *rx_data);
}

#endif // DM_MOTOR_HPP
