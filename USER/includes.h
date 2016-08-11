#ifndef __INCLUDES_H__
#define __INCLUDES_H__



//#define CORDIC_TEST
//#define SPP_UART_TEST
//#define MPU6050_ATTITUDE_TEST
//#define PWM_TEST
//#define ADC_TEST
//#define ENCODER_TEST
#define BALANCE_RUN

/******电机控制相关宏定义******/
#define TIM_ARR (1000)
#define Balance_Period (100) //100Hz
#define PwmPeriodCount_Per_AdjustPeriod (SystemCoreClock/Balance_Period/TIM_ARR)


#define MOTOR_OUT_DEAD_VAL       0	   //死区值
#define MOTOR_OUT_MAX          (TIM_ARR)   //占空比正最大值
#define MOTOR_OUT_MIN          (-TIM_ARR)  //占空比负最大值		   

#define AIN2_HIGH()   GPIOB->BSRR = GPIO_Pin_12;//GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define AIN2_LOW()    GPIOB->BRR = GPIO_Pin_12;//GPIO_ResetBits(GPIOB,GPIO_Pin_12)

#define AIN1_HIGH()   GPIOB->BSRR = GPIO_Pin_13;//GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define AIN1_LOW()    GPIOB->BRR = GPIO_Pin_13;//GPIO_ResetBits(GPIOB,GPIO_Pin_13)

#define BIN1_HIGH()   GPIOB->BSRR = GPIO_Pin_14;//GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define BIN1_LOW()    GPIOB->BRR = GPIO_Pin_14;//GPIO_ResetBits(GPIOB,GPIO_Pin_14)

#define BIN2_HIGH()   GPIOB->BSRR = GPIO_Pin_15;//GPIO_SetBits(GPIOB,GPIO_Pin_15)
#define BIN2_LOW()    GPIOB->BRR = GPIO_Pin_15;//GPIO_ResetBits(GPIOB,GPIO_Pin_15)

//keil库头文件
#include <math.h>    
#include <stdio.h>  

//驱动头文件
#include "sys.h"
#include "delay.h"
#include "encoder.h"
#include "usart.h"
#include "adc.h"
#include "timer.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

#include "cordic.h"
//程序头文件
#include "carstand.h"

#endif


