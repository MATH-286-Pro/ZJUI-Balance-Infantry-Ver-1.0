#ifndef __DM_MOTOR_DRV_H__
#define __DM_MOTOR_DRV_H__
#include "main.h"
#include "can.h"
#include "dm_bsp_can.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

typedef enum
{
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
	Motor7,
	Motor8,
	Motor9,
	Motor10,
    num
} motor_num;

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

// 电机参数
typedef struct
{
	uint8_t read_flag;
	uint8_t write_flag;
	uint8_t save_flag;
	
    float UV_Value;		// 低压保护值
    float KT_Value;		// 扭矩系数
    float OT_Value;		// 过温保护值
    float OC_Value;		// 过流保护值
    float ACC;			// 加速度
    float DEC;			// 减速度
    float MAX_SPD;		// 最大速度
    uint32_t MST_ID;	// 反馈ID
    uint32_t ESC_ID;	// 接收ID
    uint32_t TIMEOUT;	// 超时警报时间
    uint32_t cmode;		// 控制模式
    float  	 Damp;		// 电机粘滞系数
    float    Inertia;	// 电机转动惯量
    uint32_t hw_ver;	// 保留
    uint32_t sw_ver;	// 软件版本号
    uint32_t SN;		// 保留
    uint32_t NPP;		// 电机极对数
    float    Rs;		// 电阻
    float    Ls;		// 电感
    float    Flux;		// 磁链
    float    Gr;		// 齿轮减速比
    float    PMAX;		// 位置映射范围
    float    VMAX;		// 速度映射范围
    float    TMAX;		// 扭矩映射范围
    float    I_BW;		// 电流环控制带宽
    float    KP_ASR;	// 速度环Kp
    float    KI_ASR;	// 速度环Ki
    float    KP_APR;	// 位置环Kp
    float    KI_APR;	// 位置环Ki
    float    OV_Value;	// 过压保护值
    float    GREF;		// 齿轮力矩效率
    float    Deta;		// 速度环阻尼系数
    float 	 V_BW;		// 速度环滤波带宽
    float 	 IQ_cl;		// 电流环增强系数
    float    VL_cl;		// 速度环增强系数
    uint32_t can_br;	// CAN波特率代码
    uint32_t sub_ver;	// 子版本号
	float 	 u_off;		// u相偏置
	float	 v_off;		// v相偏置
	float	 k1;		// 补偿因子1
	float 	 k2;		// 补偿因子2
	float 	 m_off;		// 角度偏移
	float  	 dir;		// 方向
	float	 p_m;		// 电机位置
	float	 x_out;		// 输出轴位置
} esc_inf_t;

// 电机回传信息结构体
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

// 电机参数设置结构体
typedef struct
{
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
	float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

typedef struct
{
    uint16_t id;
	uint16_t mst_id;
    motor_fbpara_t para;
    motor_ctrl_t ctrl;
	esc_inf_t tmp;
} motor_t;



float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm_motor_ctrl_send(hcan_t* hcan, motor_t *motor);
void dm_motor_enable(hcan_t* hcan, motor_t *motor);
void dm_motor_disable(hcan_t* hcan, motor_t *motor);
void dm_motor_clear_para(motor_t *motor);
void dm_motor_clear_err(hcan_t* hcan, motor_t *motor);
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hcan_t* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor);
void pos_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur);
	
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);

#endif /* __DM_MOTOR_DRV_H__ */

