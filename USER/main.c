#include "includes.h"




#ifdef MPU6050_ATTITUDE_TEST	

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)(send_buf[i]);	//发送数据到串口1 
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
} 
#endif

#ifdef ENCODER_TEST
	
	u32 TIM1_Updata_Count=0;
//定时器1中断服务程序
void TIM1_UP_IRQHandler(void)   //TIM1中断 8ms
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_0);
			
		  if(TIM1_Updata_Count>= PwmPeriodCount_Per_AdjustPeriod)
			{
				TIM1_Updata_Count = 0;
					
				g_ucSpeedControlCount++;
				if(g_ucSpeedControlCount>=10) 
					{	
						g_fCarSpeed = (float)(g_iLeftMotorPulseSigma  + g_iRightMotorPulseSigma);
						g_iLeftMotorPulseSigma = g_iRightMotorPulseSigma = 0;	  //全局变量 注意及时清零

						g_ucSpeedControlCount=0;
					}
			}
			else
				TIM1_Updata_Count ++;
			
			GPIO_ResetBits(GPIOB,GPIO_Pin_0);
				
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源			
		}
			

}

#endif	
	
#ifdef BALANCE_RUN
//定时器1中断服务程序
void TIM1_UP_IRQHandler(void)   //TIM1中断 
{	
	float fcSpeed_I_Temp;
	GPIO_SetBits(GPIOB,GPIO_Pin_0);

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{

				SampleInputVoltage();		//采集数据函数
				
				g_fcAngle_P=Get_Adc_Average(Channel_Kap,8)>3;
				g_fcAngle_D=Get_Adc_Average(Channel_Kad,8)>>4;
				g_fcSpeed_P=Get_Adc_Average(Channel_Ksp,8)>>4;
			  //fcSpeed_I_Temp=Get_Adc_Average(Channel_Ksi,8)>>4;
		  	//g_fcSpeed_I=fcSpeed_I_Temp/200;
				g_fcSpeed_I=g_fcSpeed_P/200;
					

					
				AngleControl();				//直立控制函数
			
			
			
//					GPIO_SetBits(GPIOB,GPIO_Pin_1);
////        SpeedControl();					  //速度控制函数
//				if(g_ucSpeedControlCount>=10) 
//					{	 	  

//						SpeedControl();					  //速度控制函数
//						g_ucSpeedControlCount=0;

//					}
//				else 
//					g_ucSpeedControlCount++;
//			    GPIO_ResetBits(GPIOB,GPIO_Pin_1);
				
				
				g_ucPrintCount++;
				if(g_ucPrintCount>=25) 
					{	
						printf("ap=%d   ", (int)(g_fcAngle_P));
						printf("ad=%d   ", (int)(g_fcAngle_D));
						printf("sp=%d   ",(int)(g_fcSpeed_P));
						printf("si=%d   ",(int)(fcSpeed_I_Temp));
						printf("SpeedO=%d   ",(int)(g_fSpeedControlOut));
						printf("AngleO=%d   ",(int)(g_fAngleControlOut));
						printf("Angle=%d   ",(int)(Angle_Filtered));
						printf("W=%d   ",(int)(Omega_Filtered));
						printf("BLE=%d \r\n ",(int)(g_ucRxd3));
						g_ucPrintCount=0;
					}
					
				MotorOutput();							  //电机输出函数

			
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源
			GPIO_ResetBits(GPIOB,GPIO_Pin_0);		
	
		}
			
}

#endif

int main(void)
{ 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init();  //初始化延时函数

#ifdef BALANCE_RUN
	
  DriversInit();
	
	
//	AIN1_LOW();				      //右电机前进	角度为负  速度为正
//	AIN2_HIGH();
//	BIN1_LOW();				    //左电机前进  角度为负  速度为正
//	BIN2_HIGH();
//  TIM_SetCompare1(TIM1,TIM_ARR-10);		
//  TIM_SetCompare4(TIM1,TIM_ARR-10);	
//	
//	while(1)
//	{
//		
//	}
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //使能指定的TIM1中断,允许更新中断
	
	while(1)
	{  
		
		//BatteryChecker();	//电池电量检测
		
		while(g_ucUart3Flag)
		{

			BluetoothControl();	
			g_ucUart3Flag = 0;

		}
			
	}
#endif	
	

//#ifdef ENCODER_TEST
//	
//	Encoder_GPIO_init();
//	TB6612Init();
//	TIM1_PWM_Init(TIM_ARR,0,PwmPeriodCount_Per_AdjustPeriod);
//  TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE ); //使能指定的TIM1中断,允许更新中断
//	
//	AIN1_HIGH();				      //右电机前进	角度为负  速度为正
//	AIN2_LOW();
//	BIN1_HIGH();				    //左电机前进  角度为负  速度为正
//	BIN2_LOW();
//  TIM_SetCompare1(TIM1,TIM_ARR-10);		
//  TIM_SetCompare4(TIM1,TIM_ARR-10);		
//	
//	delay_ms(50000);
//  TIM_SetCompare1(TIM1,0);		
//  TIM_SetCompare4(TIM1,0);		
//	
//	delay_ms(5000);
//	
//	AIN1_LOW();				      //右电机前进	角度为负  速度为正
//	AIN2_HIGH();
//	BIN1_LOW();				    //左电机前进  角度为负  速度为正
//	BIN2_HIGH();
//  TIM_SetCompare1(TIM1,TIM_ARR-10);		
//  TIM_SetCompare4(TIM1,TIM_ARR-10);	
//	
//	while(1)
//	{
//		
//	}
//#endif
//	
//#ifdef CORDIC_TEST
//	//+30000 0x7530  
//	//+20000 0x4E20
//	//-30000 0x8AD0
//	//-20000 0xB1E0
//	//-2000 F830
//	//3000  0BB8
//	r1 = (int16_t)(CORDIC_ATAN((long)(-30000) ,(long)(-20000) )); 

//#endif	
//	
//#ifdef SPP_UART_TEST
//  SPP_uart_init(9600);		//初始化串口波特率为9600
//	while(1)
//	{
//		
//  }
//#endif	

//	
//#ifdef PWM_TEST
//	TIM1_PWM_Init(899,0);	

//  TIM_SetCompare1(TIM1,200);		
//  TIM_SetCompare4(TIM1,200);		
//	
//	while(1)
//	{
//  }
//#endif
//	
//	
//#ifdef ADC_TEST
//	
//u16 adc_Ksd,adc_Ksp,adc_Kai,adc_Kap;	
//	
// 	Adc_Init();		  		//ADC3初始化
//	while(1)
//	{
//	adc_Ksd=Get_Adc_Average(Channel_Ksd,10);
//	adc_Ksp=Get_Adc_Average(Channel_Ksp,10);
//	adc_Kai=Get_Adc_Average(Channel_Kai,10);
//	adc_Kap=Get_Adc_Average(Channel_Kap,10);
//	}
//#endif	
//	
//#ifdef MPU6050_ATTITUDE_TEST	

////	MPU_Init();					//初始化MPU6050
////	while(1)
////	{
////		
////  MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
////  MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
////	}

//	
//	CP2102_uart_init(500000);		//初始化串口波特率为500000

//	MPU_Init();					//初始化MPU6050

//	while(mpu_dmp_init())
//	{
//	}

// 	while(1)
//	{

//		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
//		{ 
//			temp=MPU_Get_Temperature();	//得到温度值
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
//			mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
//			usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
//		}
//	}
//#endif	

}	

