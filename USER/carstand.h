#ifndef _CARSTAND_H_
#define _CARSTAND_H_

/******�Ƕȿ�����غ궨��******/
#define	CAR_ANGLE_SET	      0				//�Ƕ�Ԥ��ֵ
#define CAR_ANGULARSPEED_SET  0					//���ٶ�Ԥ��ֵ
#define GRAVITY_OFFSET   (0)//(2000)      			//���ٶ����ƫ��ֵ 
#define GYRO_OFFSET      0      				//���������ƫ��ֵ 
#define	GYROSCOPE_ANGLE_RATIO 131.0		        //�����Ǳ�������
/******�ٶȿ�����غ궨��******/
#define CAR_POSITION_SET  0						//·��Ԥ��ֵ
#define CAR_SPEED_SET     g_iCarSpeedSet	    //�ٶ�����ֵ
#define MOTOR_LEFT_SPEED_POSITIVE  ((int)g_fLeftMotorOut >0)   //�����ٶȷ����ж�
#define MOTOR_RIGHT_SPEED_POSITIVE ((int)g_fRightMotorOut>0)   //�����ٶȷ����ж�
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

extern float g_fcAngle_P ;  	 //�ǶȻ�P����
extern float g_fcAngle_D ;   	 //�ǶȻ�D����
extern float g_fcSpeed_P ; 	 //�ٶȻ�P����	15
extern float g_fcSpeed_I ;   	 //�ٶȻ�I����	1.4

extern int g_iAccelInputVoltage_X_Axis;
extern int g_iAccelInputVoltage_Y_Axis ;	
extern int g_iAccelInputVoltage_Z_Axis ;	
extern int g_iGyroInputVoltage_X_Axis ;
extern int g_iGyroInputVoltage_Y_Axis ;
extern int g_iGyroInputVoltage_Z_Axis ;

extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;	//������ԭʼ����

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

