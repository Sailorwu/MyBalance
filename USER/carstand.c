#include "includes.h"

#define	SlaveAddress 0xD0   //IIC

unsigned int  g_uiStartCount;
unsigned char g_ucLEDCount;
unsigned char g_ucIRFlag = 0;
/******������Ʋ���******/
int   g_iCarSpeedSet;
float g_fSpeedControlOut;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;
/******�Ƕȿ��Ʋ���******/
int   g_iAccelInputVoltage_X_Axis ;	//���ٶ�X������
int   g_iAccelInputVoltage_Y_Axis ;	//���ٶ�Y������
int   g_iAccelInputVoltage_Z_Axis ;	//���ٶ�Z������
int   g_iGyroInputVoltage_X_Axis  ;	//������X������
int   g_iGyroInputVoltage_Y_Axis  ;	//������Y������
int   g_iGyroInputVoltage_Z_Axis  ;	//������Z������

short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����


float g_fCarAngle;         			//��ģ���
float g_fGyroAngleSpeed;			//���ٶ�      			
float g_fGyroscopeAngleIntegral;	//���ٶȻ���ֵ
float g_fGravityAngle;				//���ٶȳ�������õ������
int g_iGyroOffset;
/******�ٶȿ��Ʋ���******/
int   g_iLeftMotorPulse;
int   g_iRightMotorPulse;
float g_fDeltaOld;
float g_fCarSpeed;
float g_fCarSpeedOld;
volatile float g_fCarPosition;
int g_ucSpeedControlPeriod ;
volatile int g_ucSpeedControlCount ;
volatile int g_ucPrintCount ;
/*-----�ǶȻ����ٶȻ�PID���Ʋ���-----*/
float g_fcAngle_P = 90.0;  	 //�ǶȻ�P����
float g_fcAngle_D = 3.0;   	 //�ǶȻ�D����
float g_fcSpeed_P = 15.0 ; 	 //�ٶȻ�P����	15
float g_fcSpeed_I = 1.4;   	 //�ٶȻ�I����	1.4
//float code g_fcSpeed_D = 0.0;		 //�ٶȻ�D����
float g_fcDirection_P = 300.0;	 //����P����
float g_fcEliminate_P= 0.0;	 //�̾����������P����  
/******�������Ʋ���******/
volatile float g_fBluetoothSpeed;
volatile float g_fBluetoothDirection;

volatile float g_fDirectionDeviation;
float g_fDirectionControlOut;

float g_fPower;

volatile unsigned char g_ucRxd3;
volatile unsigned char g_ucUart3Flag;

unsigned char g_ucUltraDis;
unsigned char g_ucUltraDisLast;
float g_fUltraSpeed;

static unsigned char backFlag=0;
unsigned char UltraControlMode=0;


void TB6612Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��GPIOA Bʱ��
		
	 //Motor1 AIN2 AIN1   PB.12 PB.13
	 //Motor1 BIN1 BIN2   PB.14 PB.15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
   //���ø�����Ϊ�����������,���TIM_CH1 TIM_CH4��PWM���岨��	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11; //TIM_CH1 TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: DriversInit
** ��������: �ײ�������ʼ��            
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/

void DriversInit(void)
{
  MPU_Init();					//��ʼ��MPU6050
	Encoder_GPIO_init();
  Adc_Init();
	TB6612Init();
	TIM1_PWM_Init(TIM_ARR,0,PwmPeriodCount_Per_AdjustPeriod);
	CP2102_uart_init(500000);		//��ʼ�����ڲ�����Ϊ500000
  SPP_uart_init(9600);		//��ʼ�����ڲ�����Ϊ9600

}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: CarStandInit
** ��������: ֱ��������ʼ��            
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void CarStandInit()
{
	aacx = gyroy = 0;
	g_iLeftMotorPulse = g_iRightMotorPulse = 0;
	g_iLeftMotorPulseSigma = g_iRightMotorPulseSigma = 0;
	g_iCarSpeedSet = 0;

	g_iCarSpeedSet=0;
	g_fCarSpeed    = 0;
	g_fCarSpeedOld = 0;
	g_fCarPosition = 0;
	g_fCarAngle    = 0;
	g_fGyroAngleSpeed = 0;
	g_fGravityAngle   = 0;
	g_fGyroscopeAngleIntegral = 0;

	g_fAngleControlOut = g_fSpeedControlOut = 0;

	g_fLeftMotorOut    = g_fRightMotorOut   = 0;
	g_fBluetoothSpeed  = g_fBluetoothDirection = 0;

  g_ucLEDCount = 0;				
	g_uiStartCount= 0;

	g_fDirectionDeviation = g_fDirectionControlOut = 0;

	g_fPower = 0;

	g_ucRxd3 = g_ucUart3Flag = 0;
	g_fUltraSpeed = 0;
	g_ucUltraDisLast = 0;

	USART3SendStr("AT+NAMEMWBalanced\r\n");//���������豸����Ϊ MWBalanced
}
/***************************************************************
** ��������: Single_ReadI2C
** ��������: ��I2C�豸��ȡһ���ֽ�����
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����
** ��  ����  Http://miaowlabs.taobao.com
** �ա���:   2014��08��01��
***************************************************************/
unsigned char Single_ReadI2C(unsigned char REG_Address)
{
	unsigned char REG_data;
	IIC_Start();                   //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress);    //�����豸��ַ+д�ź�
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Start();                   //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress+1);  //�����豸��ַ+���ź�
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	REG_data=IIC_Read_Byte(0);       //�����Ĵ�������
	//IIC_Send_ACK(1);                //����Ӧ���ź�
	IIC_Stop();                    //ֹͣ�ź�
	return REG_data;
}

int16_t Single_ReadI2C_TwoBytes(unsigned char REG_Address)
{
	u16 buffer;
	IIC_Start();                   //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress);    //�����豸��ַ+д�ź�
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Start();                   //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress+1);  //�����豸��ַ+���ź�
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	
	buffer =(IIC_Read_Byte(1))<<8;		//������,����ACK  
	buffer |= IIC_Read_Byte(0);//������,����nACK 
	
	IIC_Stop();                    //ֹͣ�ź�
	return buffer;
}
/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: DataSynthesis
** ��������: ���ݺϳɺ���            
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
int DataSynthesis(unsigned char REG_Address)	
{
//	char uiHighByte; /*�߰�λ*/
//	char ucLowByte; /*�Ͱ�λ*/

//	uiHighByte = Single_ReadI2C(REG_Address)  ;
//	ucLowByte  = Single_ReadI2C(REG_Address+1);

//	return ((uiHighByte << 8) + ucLowByte);   /*���غϳ�����*/
	
	return Single_ReadI2C_TwoBytes(REG_Address);
}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: SampleInputVoltage
** ��������: MPU6050��������            
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void SampleInputVoltage(void)
{	
//	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
	
//g_iGyroInputVoltage_Y_Axis   = DataSynthesis(GYRO_YOUT_H) ; //������Y��
//g_iAccelInputVoltage_X_Axis  = DataSynthesis(ACCEL_XOUT_H); //���ٶ�X��	
  g_iAccelInputVoltage_Y_Axis  = DataSynthesis(ACCEL_YOUT_H); //���ٶ�Y��		
  g_iAccelInputVoltage_Z_Axis  = DataSynthesis(ACCEL_ZOUT_H); //���ٶ�Z��		
	g_iGyroInputVoltage_X_Axis   = DataSynthesis(GYRO_XOUT_H) ; //������X��	

}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: GyroRevise
** ��������: ������У������            
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void GetGyroRevise()
{
//	long int tempsum;
//	int temp;
//	for(temp=0;temp<500;temp++)
//	{
//		tempsum += DataSynthesis(GYRO_YOUT_H) ;
//	}
//	g_iGyroOffset = (int)(tempsum/500);
//	tempsum=0;
}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: SetMotorVoltageAndDirection
** ��������: ������ú���            
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
	int iLeftMotorValue;
	int iRighttMotorValue;	
void SetMotorVoltageAndDirection(int iLeftVoltage,int iRightVoltage)
{

	if(iLeftVoltage<0)
    {	
      AIN1_HIGH();				      //�ҵ��ǰ��	�Ƕ�Ϊ��  �ٶ�Ϊ��
      AIN2_LOW();
      iLeftMotorValue = (-iLeftVoltage);
    }
    else 
    {	
      AIN1_LOW();				      //�ҵ������  �Ƕ�Ϊ��  �ٶ�Ϊ��
      AIN2_HIGH(); 
      iLeftMotorValue = iLeftVoltage;
    }

	if(iRightVoltage<0)
    {	
      BIN1_HIGH();				    //����ǰ��  �Ƕ�Ϊ��  �ٶ�Ϊ��
      BIN2_LOW();
      iRighttMotorValue = (-iRightVoltage);
    }
    else
    {	
      BIN1_LOW();				      //��������  �Ƕ�Ϊ��  �ٶ�Ϊ��
      BIN2_HIGH(); 
      iRighttMotorValue = iRightVoltage;
    }

//	iLeftMotorValue   = (1000 - iLeftMotorValue)  ;	   
//	iRighttMotorValue = (1000 - iRighttMotorValue);
 
  TIM_SetCompare1(TIM1,iRighttMotorValue);	
  TIM_SetCompare4(TIM1,iLeftMotorValue);		
		
		
#if 1//IF_CAR_FALL		 /*�жϳ����Ƿ������������*/

	if((int)g_fCarAngle > 25 || (int)g_fCarAngle < (-25))
	{
		AIN1_LOW();				      //�ҵ��ǰ��	�Ƕ�Ϊ��  �ٶ�Ϊ��
    AIN2_LOW(); 
		BIN1_LOW();				      //�ҵ��ǰ��	�Ƕ�Ϊ��  �ٶ�Ϊ��
    BIN2_LOW();   
	}

#endif

}
/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: MotorOutput
** ��������: ����������            
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: ��ֱ�����ơ��ٶȿ��ơ�������Ƶ���������е���,����
			 �����������������������������
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void MotorOutput(void)
{

	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection ;// + g_fDirectionControlOut;
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirection ;//- g_fDirectionControlOut;			
	
	/*������������*/
//	if(g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
//	else if(g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
//	if(g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
//	else if(g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*������ʹ����Ǳ�֤��������ᳬ��PWM�������̷�Χ��*/	
	if(g_fLeftMotorOut  >= MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if(g_fLeftMotorOut  <= MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if(g_fRightMotorOut >= MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if(g_fRightMotorOut <= MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;

  SetMotorVoltageAndDirection(g_fLeftMotorOut,g_fRightMotorOut);
}


/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: SpeedControl
** ��������: �ٶȻ����ƺ���           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void SpeedControl(void)
{  
	g_fCarSpeed = (float)((g_iLeftMotorPulseSigma  + g_iRightMotorPulseSigma)) ;
  g_iLeftMotorPulseSigma = g_iRightMotorPulseSigma = 0;	  //ȫ�ֱ��� ע�⼰ʱ����
  
	/*��ͨ�˲�*/
  g_fCarSpeed = (float)(g_fCarSpeedOld * 0.85f + g_fCarSpeed * 0.15f) ;  //һ�׻����˲�
	g_fCarSpeedOld = g_fCarSpeed;

	g_fCarPosition += g_fCarSpeed; 
	g_fCarPosition += g_fBluetoothSpeed;
	if(g_fCarPosition > SPEED_CONTROL_OUT_MAX) g_fCarPosition = SPEED_CONTROL_OUT_MAX;//�����ֱ��� 
	if(g_fCarPosition < -SPEED_CONTROL_OUT_MIN) g_fCarPosition = -SPEED_CONTROL_OUT_MIN;
	//printf("Pos=%d\r\n",(int)(g_fCarPosition));
  g_fSpeedControlOut = g_fCarSpeed* g_fcSpeed_P + g_fCarPosition*g_fcSpeed_I;
	
	
//	fI = fDelta * g_fcSpeed_I;
//	g_fCarPosition += fI;
//	g_fCarPosition += g_fBluetoothSpeed;


//	/*������������*/			  
//	if((int)g_fCarPosition > SPEED_CONTROL_OUT_MAX)    g_fCarPosition = SPEED_CONTROL_OUT_MAX;
//	if((int)g_fCarPosition < SPEED_CONTROL_OUT_MIN)    g_fCarPosition = SPEED_CONTROL_OUT_MIN;	
//	g_fSpeedControlOut = fP + g_fCarPosition;


	
//	Speed = Sum_Right + Sum_Left - 0; 
//  Speeds_filter *=0.7; //һ�׻����˲�
//  Speeds_filter += Speed*0.3;
//  Distance += Speeds_filter - Run_Speed;
//  Distance = constrain(Distance, -5000, 5000);
//  Output += Speeds_filter *  kps + Distance * kps/200 ;

}

float Angle_Raw, Angle_Filtered, Omega_Raw,Omega_Filtered, dt =(1.0/Balance_Period);

//һ�׻����˲�
float K1 =0.05f; // �Լ��ٶȵ�Ȩ��
void Yijielvbo(float angle_m, float gyro_m)//�ɼ������ĽǶȺͼ��ٶ�
{
     Angle_Filtered = K1 * angle_m + (1-K1) * (Angle_Filtered + gyro_m * dt);
     Omega_Filtered = Omega_Raw;
}

//���׻����˲�
float K2 =0.2; // �Լ��ٶȵ�Ȩ��
float Angle_Delta, Angle_Recursive, Angle_Confidence;

void Erjielvbo(float Angle_Raw,float Omega_Raw)//�ɼ������ĽǶȺͼ��ٶ�
{  
  Angle_Delta = (Angle_Raw - Angle_Filtered) * (1-K2)*(1-K2);
  Angle_Recursive = Angle_Delta * dt + Angle_Recursive;
  Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 2*(1-K2) + Omega_Raw;
  Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
  Omega_Filtered = Omega_Raw;
}

//�������˲������ͺ���
float P[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; //�Ƕ��������Ŷȣ��Ǽ��ٶ��������Ŷ�
float R_angle=0.5 ,C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;


//�������˲�
void Kalman_Filter(float angle_m, float gyro_m)//angleAx �� gyroGy
{
Angle_Filtered+=(gyro_m-q_bias) * dt;
angle_err = angle_m - Angle_Filtered;
Pdot[0]=Q_angle - P[0][1] - P[1][0];
Pdot[1]= -P[1][1];
Pdot[2]= -P[1][1];
Pdot[3]=Q_gyro;
P[0][0] += Pdot[0] * dt;
P[0][1] += Pdot[1] * dt;
P[1][0] += Pdot[2] * dt;
P[1][1] += Pdot[3] * dt;
PCt_0 = C_0 * P[0][0];
PCt_1 = C_0 * P[1][0];
E = R_angle + C_0 * PCt_0;
K_0 = PCt_0 / E;
K_1 = PCt_1 / E;
t_0 = PCt_0;
t_1 = C_0 * P[0][1];
P[0][0] -= K_0 * t_0;
P[0][1] -= K_0 * t_1;
P[1][0] -= K_1 * t_0;
P[1][1] -= K_1 * t_1;
Angle_Filtered += K_0 * angle_err; //���ŽǶ�
q_bias += K_1 * angle_err;
Omega_Filtered = gyro_m-q_bias;//���Ž��ٶ�
}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 2015��11��29��
** ��������: AngleControl
** ��������: �ǶȻ����ƺ���           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/

void AngleControl(void)	 
{  
	//ȥ�����ƫ��,����õ����ٶȴ������ĽǶȣ����ȣ�
//	g_fGravityAngle = (float)(g_iAccelInputVoltage_Y_Axis - GRAVITY_OFFSET) / 16384.0f;
//	// 57.2957795=180/3.1415926535898 ����ת��Ϊ��
//	g_fGravityAngle = g_fGravityAngle * 57.2957795f ;

//	g_fGyroAngleSpeed = (g_iGyroInputVoltage_X_Axis - GYRO_OFFSET) / GYROSCOPE_ANGLE_RATIO *(-1.0);
	
//	atan2CORDIC(g_iAccelInputVoltage_Y_Axis, g_iAccelInputVoltage_Z_Axis);
	
	
	Angle_Raw = (((int16_t)(atan2CORDIC(g_iAccelInputVoltage_Y_Axis, g_iAccelInputVoltage_Z_Axis)))/8192.0f) * 57.2957795f;
	
  Omega_Raw = (g_iGyroInputVoltage_X_Axis - GYRO_OFFSET) / GYROSCOPE_ANGLE_RATIO;

  #if 1
		//�����˲�
  Yijielvbo(Angle_Raw,Omega_Raw);
  #endif
  
  #if 0
		//�����˲�
  Erjielvbo(Angle_Raw,Omega_Raw);
  #endif
  
  #if 0
		//�������˲�
  Kalman_Filter(Angle_Raw,Omega_Raw);
  #endif  
	
	

	g_fCarAngle = Angle_Filtered;
	
	
	//�ǶȻ�PID����
	g_fAngleControlOut =  Angle_Filtered* g_fcAngle_P + Omega_Filtered * g_fcAngle_D ;	 

}

/***************************************************************
** ��������: BluetoothControl
** ��������: �������ƺ���
             �ֻ���������
			 ǰ����0x01    ���ˣ�0x02
             ��  0x03    �ң�  0x04
             ֹͣ��0x10
             ���ܼ������Ա���λ��������չ����
             ��������0x07
			 ��������0x08
			 �����ǰ����0x09  ������ǰ����0x0A
			 �����غ��ˣ�0x0B  �����غ��ˣ�0x0C   Ѳ��������0x0D
** �䡡��:   
** �䡡��:   
** ȫ�ֱ���: 
** ������:   ����ʵ����
** ��  ����  Http://miaowlabs.taobao.com
** �ա���:   2014��08��01��
***************************************************************/
void BluetoothControl(void)	 
{
	unsigned char ucBluetoothValue = 0;

	ucBluetoothValue = g_ucRxd3;		
		
	switch(ucBluetoothValue)
	{

	  case 0x02 : g_fBluetoothSpeed = 500 ;  break;//����
	  case 0x01 : g_fBluetoothSpeed =  (-500) ;  break;//ǰ��	  
	  case 0x03 : g_fBluetoothDirection = (-500)   ;  break;//��ת
	  case 0x04 : g_fBluetoothDirection = 500;  break;//��ת
	  case 0x05 : g_iCarSpeedSet =   20 ; break ;
	  case 0x06 : g_iCarSpeedSet = (-20); break ;
	  case 0x07 : g_fBluetoothDirection =   800 ;  break;
	  case 0x08 : g_fBluetoothDirection = (-800);  break;
	  case 0x0D : g_iCarSpeedSet = 8;  break;	   //Ѳ������
	  default : g_fBluetoothSpeed = 0; g_fBluetoothDirection = 0;g_iCarSpeedSet=0;break;
	}
}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 
** ��������: EliminateDirectionDeviation
** ��������: �̾���ֱ�߾������ƺ���           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/

void EliminateDirectionDeviation(void)
{
	int Delta = 0;

	Delta = g_iLeftMotorPulseSigma - g_iRightMotorPulseSigma;

	g_fDirectionDeviation = Delta * g_fcEliminate_P * (-1);


}


/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 20160408
** ��������: DirectionControl
** ��������: ����ѭ��������ƺ���           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void DirectionControl(void)
{ 
//  int iLeft,iRight;
//  
//  iLeft = LeftIR;
//  iRight = RightIR;

//  if(iLeft==0&&iRight==1)
//  {
//  	g_fDirectionControlOut = g_fcDirection_P;	
//  }
//  else if(iLeft==1&&iRight==0)
//  {
//  	g_fDirectionControlOut = (-1) * g_fcDirection_P;	
//  }
//  else if(iLeft==0&&iRight==0)
//  {
//  	g_fDirectionControlOut = 0;	
//  }
//  else if(iLeft==1&&iRight==1)
//  {
//  	g_fDirectionControlOut = 0;	
//  }
}

/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 20160415
** ��������: BatteryChecker
** ��������: ������⣨���������㣬�������ƣ�           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void BatteryChecker()
{

	g_fPower = Get_Adc_Average(Channel_VOLTAGE,10);//GetADCResult();	 				//max8.4*510/(1000+510)/3.3*256=220
	g_fPower = g_fPower / 220* 8400;	 		 
	if((int)g_fPower <= 7499)						//low7.4*510/(1000+510)=2.499v
	{
	}

}


/***************************************************************
** ����  ��: Songyimiao
** ��    ����http://www.miaowlabs.com
** ��    ����http://miaowlabs.taobao.com
** �ա�  ��: 20160415
** ��������: BatteryChecker
** ��������: ���������棨���0.5m��Χ���������2.5m��           
** �䡡  ��:   
** �䡡  ��:   
** ��    ע: 
********************����ʵ���Ұ�Ȩ����**************************
***************************************************************/
void UltraFollow(void)
{
	//g_ucUltraDis = g_ucUltraDis * 0.3 + g_ucUltraDisLast * 0.7;
	if(25>g_ucUltraDis>0)
	{
		if(g_ucUltraDisLast>g_ucUltraDis)	
		{
		 	g_iCarSpeedSet=(-8);
			//g_fUltraSpeed=20;
		}
		else if(g_ucUltraDisLast<g_ucUltraDis)	
		{
		 	g_iCarSpeedSet=8;
			//g_fUltraSpeed=(-20);
		}
		else
		{
			g_iCarSpeedSet=0;
			//g_fUltraSpeed=0;
		}
	}
	else 
	{
		g_iCarSpeedSet=0;
		g_fUltraSpeed=0;	
	}
	g_ucUltraDisLast = g_ucUltraDis;
}

void UltraControl()
{
	if(UltraControlMode==0){		
	if((g_ucUltraDis<=20)&&(g_fBluetoothSpeed>=20))
	{
		g_fBluetoothSpeed = 0;
	}
	
	else if((g_ucUltraDis<=15)&&(g_fBluetoothSpeed>0))
	{
		g_fBluetoothSpeed = 0;
	}
	else if(g_ucUltraDis<=10)
	{
		backFlag=1;
		g_fBluetoothSpeed = -30;
	}
	else if(backFlag==1)
	{
		backFlag=0;
		g_fBluetoothSpeed=0;
	}
	}
	if(UltraControlMode==1)
	{
	
	}
}

