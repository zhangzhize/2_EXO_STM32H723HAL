#ifndef DM_MOTOR_HPP
#define DM_MOTOR_HPP

#include <cstdint>

/** 需要根据实际电机型号更改 */
#define DM_KP_MIN   (0.0f)
#define DM_KP_MAX   (500.0f)
#define DM_KD_MIN   (0.0f)
#define DM_KD_MAX   (5.0f)

enum class DMMotorModeID : uint16_t
{
    kMIT = 0x000,
    kPosVel = 0x100,
    kVel = 0x200,
    kPosVelCur = 0x300,
};

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

// 电机参数
typedef struct DMInf
{
	DMMotorReg read_reg_;

    float UV_Value_;		// 低压保护值
    float KT_Value_;		// 扭矩系数
    float OT_Value_;		// 过温保护值
    float OC_Value_;		// 过流保护值
    float ACC_;			    // 加速度
    float DEC_;			    // 减速度
    float MAX_SPD_;		    // 最大速度
    uint32_t MST_ID_;	    // 反馈ID
    uint32_t ESC_ID_;	    // 接收ID
    uint32_t TIMEOUT_;	    // 超时警报时间
    uint32_t cmode_;		// 控制模式
    float  	 Damp_;		    // 电机粘滞系数
    float    Inertia_;	    // 电机转动惯量
    uint32_t hw_ver_;	    // 保留
    uint32_t sw_ver_;	    // 软件版本号
    uint32_t SN_;		    // 保留
    uint32_t NPP_;		    // 电机极对数
    float    Rs_;		    // 电阻
    float    Ls_;		    // 电感
    float    Flux_;		    // 磁链
    float    Gr_;		    // 齿轮减速比
    float    PMAX_;		    // 位置映射范围
    float    VMAX_;		    // 速度映射范围
    float    TMAX_;		    // 扭矩映射范围
    float    I_BW_;		    // 电流环控制带宽
    float    KP_ASR_;	    // 速度环Kp
    float    KI_ASR_;	    // 速度环Ki
    float    KP_APR_;	    // 位置环Kp
    float    KI_APR_;	    // 位置环Ki
    float    OV_Value_;	    // 过压保护值
    float    GREF_;		    // 齿轮力矩效率
    float    Deta_;		    // 速度环阻尼系数
    float 	 V_BW_;		    // 速度环滤波带宽
    float 	 IQ_cl_;		// 电流环增强系数
    float    VL_cl_;		// 速度环增强系数
    uint32_t can_br_;	    // CAN波特率代码
    uint32_t sub_ver_;	    // 子版本号
	float 	 u_off_;		// u相偏置
	float	 v_off_;		// v相偏置
	float	 k1_;		    // 补偿因子1
	float 	 k2_;		    // 补偿因子2
	float 	 m_off_;		// 角度偏移
	float  	 dir_;		    // 方向
	float	 p_m_;		    // 电机位置
	float	 x_out_;		// 输出轴位置
} DMInf;

typedef struct DMFeedback
{
    uint8_t flag_;
    int id_;
    int state_;
    float pos_rad_;
    float vel_radps_;
    float tor_Nm_;
    float Kp_;
    float Kd_;
    float Tmos_;
    float Tcoil_;
} DMFeedback;

typedef struct
{
    DMMotorModeID mode_id_;
    float pos_set_rad_;
    float vel_set_radps_;
    float tor_set_Nm_;
	float cur_set_A_;
    float kp_set_;
    float kd_set_;
} DMCtrlParam;

class DMMotor
{
public:
    DMMotor(uint16_t can_id);
    ~DMMotor() = default;

    void EnableMotor(void);
    void DisableMotor(void);
    void ClearError(void);
    void SetMecPosZero(void);
    void SetMotorMode(void);
    void MitControl(void);
    void PosVelControl(void);
    void VelControl(void);
    void PosVelCurControl(void);
    void ClearCtrlParam(void);
    void ReadReg(DMMotorReg reg);
    void WriteReg(DMMotorReg reg, uint8_t value[4]);
    void ReadFeedback(void);
    void SaveToFlash(void);
    void CanRxCallBack(uint32_t can_id, uint8_t *can_rxdata);

    uint16_t can_id_;
    uint16_t mst_id_;
    DMInf inf_;
    DMFeedback feedback_;
    DMCtrlParam ctrl_param_;
};

extern "C" {
void CallDMCanRxCallBack(DMMotor *ptr_dm, uint32_t can_ext_id, uint8_t *rx_data);
}

#endif // DM_MOTOR_HPP