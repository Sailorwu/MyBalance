#include "includes.h"

#define	SlaveAddress 0xD0   //IIC

unsigned int  g_uiStartCount;
unsigned char g_ucLEDCount;
unsigned char g_ucIRFlag = 0;
/******电机控制参数******/
int   g_iCarSpeedSet;
float g_fSpeedControlOut;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;
/******角度控制参数******/
int   g_iAccelInputVoltage_X_Axis ;	//加速度X轴数据
int   g_iAccelInputVoltage_Y_Axis ;	//加速度Y轴数据
int   g_iAccelInputVoltage_Z_Axis ;	//加速度Z轴数据
int   g_iGyroInputVoltage_X_Axis  ;	//陀螺仪X轴数据
int   g_iGyroInputVoltage_Y_Axis  ;	//陀螺仪Y轴数据
int   g_iGyroInputVoltage_Z_Axis  ;	//陀螺仪Z轴数据

short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据


float g_fCarAngle;         			//车模倾角
float g_fGyroAngleSpeed;			//角速度      			
float g_fGyroscopeAngleIntegral;	//角速度积分值
float g_fGravityAngle;				//加速度初步计算得到的倾角
int g_iGyroOffset;
/******速度控制参数******/
int   g_iLeftMotorPulse;
int   g_iRightMotorPulse;
float g_fDeltaOld;
float g_fCarSpeed;
float g_fCarSpeedOld;
volatile float g_fCarPosition;
int g_ucSpeedControlPeriod ;
volatile int g_ucSpeedControlCount ;
volatile int g_ucPrintCount ;
/*-----角度环和速度环PID控制参数-----*/
float g_fcAngle_P = 90.0;  	 //角度环P参数
float g_fcAngle_D = 3.0;   	 //角度环D参数
float g_fcSpeed_P = 15.0 ; 	 //速度环P参数	15
float g_fcSpeed_I = 1.4;   	 //速度环I参数	1.4
//float code g_fcSpeed_D = 0.0;		 //速度环D参数
float g_fcDirection_P = 300.0;	 //方向环P参数
float g_fcEliminate_P= 0.0;	 //短距离纠正方向环P参数  
/******蓝牙控制参数******/
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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	//使能GPIOA B时钟
		
	 //Motor1 AIN2 AIN1   PB.12 PB.13
	 //Motor1 BIN1 BIN2   PB.14 PB.15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
   //设置该引脚为复用输出功能,输出TIM_CH1 TIM_CH4的PWM脉冲波形	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11; //TIM_CH1 TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: DriversInit
** 功能描述: 底层驱动初始化            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/

void DriversInit(void)
{
  MPU_Init();					//初始化MPU6050
	Encoder_GPIO_init();
  Adc_Init();
	TB6612Init();
	TIM1_PWM_Init(TIM_ARR,0,PwmPeriodCount_Per_AdjustPeriod);
	CP2102_uart_init(500000);		//初始化串口波特率为500000
  SPP_uart_init(9600);		//初始化串口波特率为9600

}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: CarStandInit
** 功能描述: 直立参数初始化            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
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

	USART3SendStr("AT+NAMEMWBalanced\r\n");//设置蓝牙设备名称为 MWBalanced
}
/***************************************************************
** 函数名称: Single_ReadI2C
** 功能描述: 从I2C设备读取一个字节数据
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
unsigned char Single_ReadI2C(unsigned char REG_Address)
{
	unsigned char REG_data;
	IIC_Start();                   //起始信号
	IIC_Send_Byte(SlaveAddress);    //发送设备地址+写信号
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(REG_Address);     //发送存储单元地址，从0开始
	IIC_Wait_Ack();		//等待应答
	IIC_Start();                   //起始信号
	IIC_Send_Byte(SlaveAddress+1);  //发送设备地址+读信号
	IIC_Wait_Ack();		//等待应答
	REG_data=IIC_Read_Byte(0);       //读出寄存器数据
	//IIC_Send_ACK(1);                //接收应答信号
	IIC_Stop();                    //停止信号
	return REG_data;
}

int16_t Single_ReadI2C_TwoBytes(unsigned char REG_Address)
{
	u16 buffer;
	IIC_Start();                   //起始信号
	IIC_Send_Byte(SlaveAddress);    //发送设备地址+写信号
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(REG_Address);     //发送存储单元地址，从0开始
	IIC_Wait_Ack();		//等待应答
	IIC_Start();                   //起始信号
	IIC_Send_Byte(SlaveAddress+1);  //发送设备地址+读信号
	IIC_Wait_Ack();		//等待应答
	
	buffer =(IIC_Read_Byte(1))<<8;		//读数据,发送ACK  
	buffer |= IIC_Read_Byte(0);//读数据,发送nACK 
	
	IIC_Stop();                    //停止信号
	return buffer;
}
/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: DataSynthesis
** 功能描述: 数据合成函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
int DataSynthesis(unsigned char REG_Address)	
{
//	char uiHighByte; /*高八位*/
//	char ucLowByte; /*低八位*/

//	uiHighByte = Single_ReadI2C(REG_Address)  ;
//	ucLowByte  = Single_ReadI2C(REG_Address+1);

//	return ((uiHighByte << 8) + ucLowByte);   /*返回合成数据*/
	
	return Single_ReadI2C_TwoBytes(REG_Address);
}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: SampleInputVoltage
** 功能描述: MPU6050采样函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
void SampleInputVoltage(void)
{	
//	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
	
//g_iGyroInputVoltage_Y_Axis   = DataSynthesis(GYRO_YOUT_H) ; //陀螺仪Y轴
//g_iAccelInputVoltage_X_Axis  = DataSynthesis(ACCEL_XOUT_H); //加速度X轴	
  g_iAccelInputVoltage_Y_Axis  = DataSynthesis(ACCEL_YOUT_H); //加速度Y轴		
  g_iAccelInputVoltage_Z_Axis  = DataSynthesis(ACCEL_ZOUT_H); //加速度Z轴		
	g_iGyroInputVoltage_X_Axis   = DataSynthesis(GYRO_XOUT_H) ; //陀螺仪X轴	

}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: GyroRevise
** 功能描述: 陀螺仪校正函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
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
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机设置函数            
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
	int iLeftMotorValue;
	int iRighttMotorValue;	
void SetMotorVoltageAndDirection(int iLeftVoltage,int iRightVoltage)
{

	if(iLeftVoltage<0)
    {	
      AIN1_HIGH();				      //右电机前进	角度为负  速度为正
      AIN2_LOW();
      iLeftMotorValue = (-iLeftVoltage);
    }
    else 
    {	
      AIN1_LOW();				      //右电机后退  角度为正  速度为负
      AIN2_HIGH(); 
      iLeftMotorValue = iLeftVoltage;
    }

	if(iRightVoltage<0)
    {	
      BIN1_HIGH();				    //左电机前进  角度为负  速度为正
      BIN2_LOW();
      iRighttMotorValue = (-iRightVoltage);
    }
    else
    {	
      BIN1_LOW();				      //左电机后退  角度为正  速度为负
      BIN2_HIGH(); 
      iRighttMotorValue = iRightVoltage;
    }

//	iLeftMotorValue   = (1000 - iLeftMotorValue)  ;	   
//	iRighttMotorValue = (1000 - iRighttMotorValue);
 
  TIM_SetCompare1(TIM1,iRighttMotorValue);	
  TIM_SetCompare4(TIM1,iLeftMotorValue);		
		
		
#if 1//IF_CAR_FALL		 /*判断车辆是否跌倒，调试用*/

	if((int)g_fCarAngle > 25 || (int)g_fCarAngle < (-25))
	{
		AIN1_LOW();				      //右电机前进	角度为负  速度为正
    AIN2_LOW(); 
		BIN1_LOW();				      //右电机前进	角度为负  速度为正
    BIN2_LOW();   
	}

#endif

}
/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: MotorOutput
** 功能描述: 电机输出函数            
** 输　  入:   
** 输　  出:   
** 备    注: 将直立控制、速度控制、方向控制的输出量进行叠加,并加
			 入死区常量，对输出饱和作出处理。
********************喵呜实验室版权所有**************************
***************************************************************/
void MotorOutput(void)
{

	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection ;// + g_fDirectionControlOut;
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirection ;//- g_fDirectionControlOut;			
	
	/*增加死区常数*/
//	if(g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
//	else if(g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
//	if(g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
//	else if(g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*输出饱和处理是保证输出量不会超出PWM的满量程范围。*/	
	if(g_fLeftMotorOut  >= MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if(g_fLeftMotorOut  <= MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if(g_fRightMotorOut >= MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if(g_fRightMotorOut <= MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;

  SetMotorVoltageAndDirection(g_fLeftMotorOut,g_fRightMotorOut);
}


/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/
void SpeedControl(void)
{  
	g_fCarSpeed = (float)((g_iLeftMotorPulseSigma  + g_iRightMotorPulseSigma)) ;
  g_iLeftMotorPulseSigma = g_iRightMotorPulseSigma = 0;	  //全局变量 注意及时清零
  
	/*低通滤波*/
  g_fCarSpeed = (float)(g_fCarSpeedOld * 0.85f + g_fCarSpeed * 0.15f) ;  //一阶互补滤波
	g_fCarSpeedOld = g_fCarSpeed;

	g_fCarPosition += g_fCarSpeed; 
	g_fCarPosition += g_fBluetoothSpeed;
	if(g_fCarPosition > SPEED_CONTROL_OUT_MAX) g_fCarPosition = SPEED_CONTROL_OUT_MAX;//抗积分饱和 
	if(g_fCarPosition < -SPEED_CONTROL_OUT_MIN) g_fCarPosition = -SPEED_CONTROL_OUT_MIN;
	//printf("Pos=%d\r\n",(int)(g_fCarPosition));
  g_fSpeedControlOut = g_fCarSpeed* g_fcSpeed_P + g_fCarPosition*g_fcSpeed_I;
	
	
//	fI = fDelta * g_fcSpeed_I;
//	g_fCarPosition += fI;
//	g_fCarPosition += g_fBluetoothSpeed;


//	/*积分上限设限*/			  
//	if((int)g_fCarPosition > SPEED_CONTROL_OUT_MAX)    g_fCarPosition = SPEED_CONTROL_OUT_MAX;
//	if((int)g_fCarPosition < SPEED_CONTROL_OUT_MIN)    g_fCarPosition = SPEED_CONTROL_OUT_MIN;	
//	g_fSpeedControlOut = fP + g_fCarPosition;


	
//	Speed = Sum_Right + Sum_Left - 0; 
//  Speeds_filter *=0.7; //一阶互补滤波
//  Speeds_filter += Speed*0.3;
//  Distance += Speeds_filter - Run_Speed;
//  Distance = constrain(Distance, -5000, 5000);
//  Output += Speeds_filter *  kps + Distance * kps/200 ;

}

float Angle_Raw, Angle_Filtered, Omega_Raw,Omega_Filtered, dt =(1.0/Balance_Period);

//一阶互补滤波
float K1 =0.05f; // 对加速度的权重
void Yijielvbo(float angle_m, float gyro_m)//采集后计算的角度和加速度
{
     Angle_Filtered = K1 * angle_m + (1-K1) * (Angle_Filtered + gyro_m * dt);
     Omega_Filtered = Omega_Raw;
}

//二阶互补滤波
float K2 =0.2; // 对加速度的权重
float Angle_Delta, Angle_Recursive, Angle_Confidence;

void Erjielvbo(float Angle_Raw,float Omega_Raw)//采集后计算的角度和加速度
{  
  Angle_Delta = (Angle_Raw - Angle_Filtered) * (1-K2)*(1-K2);
  Angle_Recursive = Angle_Delta * dt + Angle_Recursive;
  Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 2*(1-K2) + Omega_Raw;
  Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
  Omega_Filtered = Omega_Raw;
}

//卡尔曼滤波参数和函数
float P[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度，角加速度数据置信度
float R_angle=0.5 ,C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;


//卡尔曼滤波
void Kalman_Filter(float angle_m, float gyro_m)//angleAx 和 gyroGy
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
Angle_Filtered += K_0 * angle_err; //最优角度
q_bias += K_1 * angle_err;
Omega_Filtered = gyro_m-q_bias;//最优角速度
}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 2015年11月29日
** 函数名称: AngleControl
** 功能描述: 角度环控制函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/

void AngleControl(void)	 
{  
	//去除零点偏移,计算得到加速度传感器的角度（弧度）
//	g_fGravityAngle = (float)(g_iAccelInputVoltage_Y_Axis - GRAVITY_OFFSET) / 16384.0f;
//	// 57.2957795=180/3.1415926535898 弧度转换为度
//	g_fGravityAngle = g_fGravityAngle * 57.2957795f ;

//	g_fGyroAngleSpeed = (g_iGyroInputVoltage_X_Axis - GYRO_OFFSET) / GYROSCOPE_ANGLE_RATIO *(-1.0);
	
//	atan2CORDIC(g_iAccelInputVoltage_Y_Axis, g_iAccelInputVoltage_Z_Axis);
	
	
	Angle_Raw = (((int16_t)(atan2CORDIC(g_iAccelInputVoltage_Y_Axis, g_iAccelInputVoltage_Z_Axis)))/8192.0f) * 57.2957795f;
	
  Omega_Raw = (g_iGyroInputVoltage_X_Axis - GYRO_OFFSET) / GYROSCOPE_ANGLE_RATIO;

  #if 1
		//互补滤波
  Yijielvbo(Angle_Raw,Omega_Raw);
  #endif
  
  #if 0
		//互补滤波
  Erjielvbo(Angle_Raw,Omega_Raw);
  #endif
  
  #if 0
		//卡尔曼滤波
  Kalman_Filter(Angle_Raw,Omega_Raw);
  #endif  
	
	

	g_fCarAngle = Angle_Filtered;
	
	
	//角度环PID控制
	g_fAngleControlOut =  Angle_Filtered* g_fcAngle_P + Omega_Filtered * g_fcAngle_D ;	 

}

/***************************************************************
** 函数名称: BluetoothControl
** 功能描述: 蓝牙控制函数
             手机发送内容
			 前进：0x01    后退：0x02
             左：  0x03    右：  0x04
             停止：0x10
             功能键（可自编下位机程序扩展）：
             左自旋：0x07
			 右自旋：0x08
			 更快地前进：0x09  更慢地前进：0x0A
			 更慢地后退：0x0B  更慢地后退：0x0C   巡线启动：0x0D
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室
** 淘  宝：  Http://miaowlabs.taobao.com
** 日　期:   2014年08月01日
***************************************************************/
void BluetoothControl(void)	 
{
	unsigned char ucBluetoothValue = 0;

	ucBluetoothValue = g_ucRxd3;		
		
	switch(ucBluetoothValue)
	{

	  case 0x02 : g_fBluetoothSpeed = 500 ;  break;//后退
	  case 0x01 : g_fBluetoothSpeed =  (-500) ;  break;//前进	  
	  case 0x03 : g_fBluetoothDirection = (-500)   ;  break;//左转
	  case 0x04 : g_fBluetoothDirection = 500;  break;//右转
	  case 0x05 : g_iCarSpeedSet =   20 ; break ;
	  case 0x06 : g_iCarSpeedSet = (-20); break ;
	  case 0x07 : g_fBluetoothDirection =   800 ;  break;
	  case 0x08 : g_fBluetoothDirection = (-800);  break;
	  case 0x0D : g_iCarSpeedSet = 8;  break;	   //巡线启动
	  default : g_fBluetoothSpeed = 0; g_fBluetoothDirection = 0;g_iCarSpeedSet=0;break;
	}
}

/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 
** 函数名称: EliminateDirectionDeviation
** 功能描述: 短距离直线纠正控制函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
***************************************************************/

void EliminateDirectionDeviation(void)
{
	int Delta = 0;

	Delta = g_iLeftMotorPulseSigma - g_iRightMotorPulseSigma;

	g_fDirectionDeviation = Delta * g_fcEliminate_P * (-1);


}


/***************************************************************
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 20160408
** 函数名称: DirectionControl
** 功能描述: 红外循迹方向控制函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
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
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 20160415
** 函数名称: BatteryChecker
** 功能描述: 电量检测（若电量不足，将亮起红灯）           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
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
** 作　  者: Songyimiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：http://miaowlabs.taobao.com
** 日　  期: 20160415
** 函数名称: BatteryChecker
** 功能描述: 超声波跟随（设成0.5m范围，最大可设成2.5m）           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室版权所有**************************
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

