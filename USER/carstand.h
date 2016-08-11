#ifndef _CARSTAND_H_
#define _CARSTAND_H_

/******角度控制相关宏定义******/
#define	CAR_ANGLE_SET	      0				//角度预设值
#define CAR_ANGULARSPEED_SET  0					//角速度预设值
#define GRAVITY_OFFSET   (0)//(2000)      			//加速度零点偏移值 
#define GYRO_OFFSET      0      				//陀螺仪零点偏移值 
#define	GYROSCOPE_ANGLE_RATIO 131.0		        //陀螺仪比例因子
/******速度控制相关宏定义******/
#define CAR_POSITION_SET  0						//路程预设值
#define CAR_SPEED_SET     g_iCarSpeedSet	    //速度期望值
#define MOTOR_LEFT_SPEED_POSITIVE  ((int)g_fLeftMotorOut >0)   //左轮速度方向判断
#define MOTOR_RIGHT_SPEED_POSITIVE ((int)g_fRightMotorOut>0)   //右轮速度方向判断
#define SPEED_CONTROL_OUT_MAX	(MOTOR_OUT_MAX )
#define SPEED_CONTROL_OUT_MIN	(MOTOR_OUT_MIN )


extern unsigned char  g_ucIRFlag;
extern unsigned int  g_uiStartCount;
extern unsigned char  g_ucLEDCount;
extern int g_iLeftMotorPulse;
extern int g_iRightMotorPulse;
extern float g_fCarAngle;
extern float g_fGravityAngle;
extern float g_fGyroAngleSpeed ;

extern float g_fcAngle_P ;  	 //角度环P参数
extern float g_fcAngle_D ;   	 //角度环D参数
extern float g_fcSpeed_P ; 	 //速度环P参数	15
extern float g_fcSpeed_I ;   	 //速度环I参数	1.4

extern int g_iAccelInputVoltage_X_Axis;
extern int g_iAccelInputVoltage_Y_Axis ;	
extern int g_iAccelInputVoltage_Z_Axis ;	
extern int g_iGyroInputVoltage_X_Axis ;
extern int g_iGyroInputVoltage_Y_Axis ;
extern int g_iGyroInputVoltage_Z_Axis ;

extern short aacx,aacy,aacz;		//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据

extern volatile int g_ucSpeedControlCount ;
extern volatile int g_ucPrintCount;
extern float g_fSpeedControlOut;
extern float g_fCarSpeed;
extern volatile float g_fCarPosition;
extern float g_fGyroscopeAngleIntegral;
extern int g_iGyroOffset;;
extern float g_fAngleControlOut;
extern float g_fLeftMotorOut;
extern float g_fPower;
extern volatile unsigned char g_ucRxd3;
extern volatile unsigned char g_ucUart3Flag;
extern unsigned char g_ucUltraDis;
extern unsigned char g_ucUltraDis;
extern unsigned char g_ucUltraDisLast;

extern float Angle_Filtered,Omega_Filtered;

void DirectionControl(void);
void EliminateDirectionDeviation(void);
void DriversInit(void);
void SampleInputVoltage(void);
void CarStandInit(void);
void SpeedControl(void);
void AngleControl(void);
void BluetoothControl(void);	
void GetMotorPulse(void);
void GetGyroRevise(void);
void MotorOutput(void);
void BatteryChecker(void);
void UltraControl(void);
int DataSynthesis(unsigned char REG_Address);
void TB6612Init(void);
#endif

