/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "OLED.h"
#include "Debug_Tool.h"
#include "ins_task.h"
#include "ZJUI_balance.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include "rc.h"
#include "bsp_buzzer.h"
#include "MI_motor_drive.h"
#include "A1_motor_drive.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "joint.h"
#include "wheel.h"
#include "Balance.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern LinkNPodParam l_side, r_side;    
extern ChassisParam chassis;

extern MI_Motor_s MI_Motor_ID1;               // 定义小米电机结构体1
extern MI_Motor_s MI_Motor_ID2;               // 定义小米电机结构体2

extern RC_Type rc;        // 遥控器数据

extern INS_t *INS_DATA;
extern pid_type_def PID_Balance;
extern pid_type_def PID_VEL_UP;

extern float Vel_print;
extern float Vel_L;
extern float Vel_R;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId insTaskHandle;   // 手动添加
osThreadId robotTaskHandle; // 手动添加
osThreadId Joint_MotorHandle;  // 手动添加


/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartINSTASK(void const *argument);      // 手动添加
void StartROBOTTASK(void const *argument);    // 手动添加
void Joint_Motor_Task(void const * argument); // 手动添加


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512); // 原先为 128
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(instask, StartINSTASK, osPriorityNormal, 0, 1024); // 手动添加
  insTaskHandle = osThreadCreate(osThread(instask), NULL);

  osThreadDef(robottask, StartROBOTTASK, osPriorityAboveNormal, 0, 2048); // 手动添加
  robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

  osThreadDef(Joint_Motor, Joint_Motor_Task, osPriorityIdle, 0, 128);
  Joint_MotorHandle = osThreadCreate(osThread(Joint_Motor), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  extern float Vel_Diff;
  /* Infinite loop */
  for(;;)
  {
    // USB_printf("Velocity.Dist.motionN:%d,%d,%d,%d,%d,%d,%d,%d,%d\n",(int)(INS_DATA->Yaw),(int)(INS_DATA->Pitch),(int)(INS_DATA->Roll),
    //                                     (int)(INS_DATA->MotionAccel_b[0]*100),(int)(INS_DATA->MotionAccel_b[1]*100),(int)(INS_DATA->MotionAccel_b[2]*100),
    //                                     (int)(INS_DATA->Vel[0]*100),(int)(INS_DATA->Vel[1]*100),(int)(INS_DATA->Vel[2]*100));
    // USB_printf("Velocity.Dist.motionN:%d,%d\n",(int)(chassis.dist*100),(int)(chassis.vel_m*100));

    // 湖南大学 Pitch Roll 需要互换 
    // USB_printf("Yaw.Roll.Pitch:%d,%d,%d,%d,%d\n",(int)(INS_DATA->Roll*10),(int)(Vel_print*10*10),(int)(rc.RY*4*10*10),
    //                                           (int)(PID_VEL_UP.out*100)),(int)(PID_Balance.out*100);
    // osDelay(10);
    // usart1_printf
    USB_printf("Vel_Diff.Vel_L.Vel_R:%d,%d,%d\n", 
                              (int)(Vel_Diff*100),
                              (int)(Vel_L*100),
                              (int)(Vel_R*100));
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartINSTASK(void const *argument)
{
  while (1)
  {
    // 1kHz
    INS_Task(); 
    osDelay(1);
  }
}


/*------------------------------------------------------髋关节控制---------------------------------------------------------*/
void Joint_Control_function(float left_angle, float right_angle, float KP, float KD){

  // 基本参数
  float32_t MAX_ANGLE     = 0.0*PI/180;    // 向上最大角度
  float32_t MIN_ANGLE     = -60.0*PI/180;  // 向下最大角度
  float32_t retract_angle = 50.0 * PI/180; // 50度的收回角度

  // 限幅
  if (left_angle > MAX_ANGLE) left_angle = MAX_ANGLE;
  if (left_angle < MIN_ANGLE) left_angle = MIN_ANGLE;
  if (right_angle > MAX_ANGLE) right_angle = MAX_ANGLE;
  if (right_angle < MIN_ANGLE) right_angle = MIN_ANGLE;

  // 电机控制
    // 左腿（Motor1, Motor2）                这里 retrct_angle 和 left/right angle 符号是一样的，都是向上为正
    mit_ctrl(&hcan2, &motor[Motor1], motor[Motor1].id, -retract_angle - left_angle, 0, KP, KD, 0);     osDelay(2);
    mit_ctrl(&hcan2, &motor[Motor2], motor[Motor2].id, +retract_angle + left_angle, 0, KP, KD, 0);     osDelay(2);

    // 右腿（Motor3, Motor4）
    mit_ctrl(&hcan2, &motor[Motor3], motor[Motor3].id, +retract_angle + right_angle, 0, KP, KD, 0);     osDelay(2);
    mit_ctrl(&hcan2, &motor[Motor4], motor[Motor4].id, -retract_angle - right_angle, 0, KP, KD, 0);     osDelay(2);

}

void Joint_Motor_Task(void const * argument)
{
  Joint_Control_function(0, 0, 1, 0); // 初始位置，关节角度为0

  while(1)  // 如果没有 while 那么该函数只会执行一次，如果该函数结束就会触发 prvTaskExitError() 报错函数
  {
    // 位置模式 (零点模式)
    if (rc.sw1 == SW_UP) {
      Joint_Control_function(0, 0, 1, 0); // 位置模式下，关节角度为0
    }
    // 位置模式
    if (rc.sw1 == SW_MID) {
      Joint_Control_function(rc.LY*PI/3.0 - rc.LX*PI/3.0, rc.LY*PI/3.0 + rc.LX*PI/3.0, 80.0, 2.0); // 位置模式下，关节角度为 LY 的值
    }
  }   
}

void StartROBOTTASK(void const *argument)
{
  // static float robot_dt, robot_start;
  MI_motor_Init(&MI_Motor_ID1,&MI_CAN_1,1); // 将MI_CAN_1，ID=1传入小米结构体 
  MI_motor_Init(&MI_Motor_ID2,&MI_CAN_1,2); // 将MI_CAN_1，ID=2传入小米结构体 
  MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
  MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能
  osDelay(100);
  for(;;)
  {
    // robot_start = DWT_GetTimeline_ms();
    // BalanceTask();
    // robot_dt = DWT_GetTimeline_ms() - robot_start;
    // osDelay(1);
    if (rc.sw2 == SW_UP)   //倒地-遥控模式
    {
      Wheel_Speed_Control(rc.RY*20 + rc.RX*20, rc.RY*20 - rc.RX*20);
      osDelay(2);
    }
    if (rc.sw2 == SW_MID)  //平衡-遥控模式
    {
      stand_task_start(INS_DATA, rc.RY, rc.RX);
      osDelay(2);
    }
    if (rc.sw2 == SW_DOWN) //平衡-跟踪模式
    {
      stand_task_start(INS_DATA, rc.RY, rc.RX);
      osDelay(2);
    }
  }
}
/* USER CODE END Application */


