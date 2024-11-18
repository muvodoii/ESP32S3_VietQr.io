#include "cptiic.h"
#include "mcu_touch_magic.h"

//控制I2C速度的延时
void CT_Delay(uint32_t us)
{
	uint32_t time=100*us/7;    
    while(--time); 
} 

//电容触摸芯片IIC接口初始化
CPTIIC::CPTIIC(int8_t cptscl, int8_t cptsda)
{					     
//#ifdef __AVR__
//	cptsclPort = portOutputRegister(digitalPinToPort(cptscl));
//	cptsdaPort = portOutputRegister(digitalPinToPort(cptsda));

//	cptsclPinSet = digitalPinToBitMask(cptscl);
//	cptsdaPinSet = digitalPinToBitMask(cptsda);
	
//	pinMode(cptscl, OUTPUT);	  // Enable outputs
//	pinMode(cptsda, OUTPUT);
//#endif
	digitalWrite(cptscl, HIGH);
	digitalWrite(cptsda, HIGH);
	pinMode(cptscl, OUTPUT);
	pinMode(cptsda, OUTPUT);
	gcptsda=cptsda;
	gcptscl=cptscl;
}

//产生IIC起始信号
void CPTIIC::CT_IIC_Start(void)
{
	pinMode(gcptsda, OUTPUT);     //sda线输出
	//CT_IIC_SDA_HIGH;
	digitalWrite(gcptsda, HIGH);
	//CT_IIC_SCL_HIGH;
	digitalWrite(gcptscl, HIGH);
	delayMicroseconds(10);
 	//CT_IIC_SDA_LOW;//START:when CLK is high,DATA change form high to low 
	digitalWrite(gcptsda, LOW);
	delayMicroseconds(10);
	//CT_IIC_SCL_LOW;//钳住I2C总线，准备发送或接收数据 
	digitalWrite(gcptscl, LOW);
}

//产生IIC停止信号
void CPTIIC::CT_IIC_Stop(void)
{ 
	pinMode(gcptsda, OUTPUT);     //sda线输出
	//CT_IIC_SCL_LOW;
	digitalWrite(gcptscl, LOW);
	//CT_IIC_SDA_LOW;
	digitalWrite(gcptsda, LOW);
	delayMicroseconds(10);
	//CT_IIC_SCL_HIGH;
	digitalWrite(gcptscl, HIGH);
	delayMicroseconds(10);
	//CT_IIC_SDA_HIGH;//STOP:when CLK is high DATA change form low to high 
	digitalWrite(gcptsda, HIGH);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t CPTIIC::CT_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	pinMode(gcptsda, INPUT);      //SDA设置为输入  
	//CT_IIC_SDA_HIGH;
	digitalWrite(gcptsda, HIGH);
	delayMicroseconds(1);	   
	//CT_IIC_SCL_HIGH;
	digitalWrite(gcptscl, HIGH);
	delayMicroseconds(1);	 
	//while(CT_IIC_SDA_STATE)
	while(digitalRead(gcptsda))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			CT_IIC_Stop();
			return 1;
		} 
	}
	//CT_IIC_SCL_LOW;//时钟输出0 	   
	digitalWrite(gcptscl, LOW);
	return 0;  
} 

//产生ACK应答
void CPTIIC::CT_IIC_Ack(void)
{
	//CT_IIC_SCL_LOW;
	digitalWrite(gcptscl, LOW);
	pinMode(gcptsda, OUTPUT);     //sda线输出
	//CT_IIC_SDA_LOW;
	digitalWrite(gcptsda, LOW);
	delayMicroseconds(10);
	//CT_IIC_SCL_HIGH;
	digitalWrite(gcptscl, HIGH);
	delayMicroseconds(10);
	//CT_IIC_SCL_LOW;
	digitalWrite(gcptscl, LOW);
}
//不产生ACK应答		    
void CPTIIC::CT_IIC_NAck(void)
{
	//CT_IIC_SCL_LOW;
	digitalWrite(gcptscl, LOW);
	pinMode(gcptsda, OUTPUT);     //sda线输出
	//CT_IIC_SDA_HIGH;
	digitalWrite(gcptsda, HIGH);
	delayMicroseconds(10);
	//CT_IIC_SCL_HIGH;
	digitalWrite(gcptscl, HIGH);
	delayMicroseconds(10);
	//CT_IIC_SCL_HIGH;
	digitalWrite(gcptscl, HIGH);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void CPTIIC::CT_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	pinMode(gcptsda, OUTPUT);     //sda线输出 	    
    //CT_IIC_SCL_LOW;//拉低时钟开始数据传输
	digitalWrite(gcptscl,LOW);
	for(t=0;t<8;t++)
    {              
        if((txd&0x80)>>7)
        {
			//CT_IIC_SDA_HIGH;
			digitalWrite(gcptsda,HIGH);
		}
		else
		{
			//CT_IIC_SCL_LOW;
			digitalWrite(gcptsda,LOW);
		}
        txd<<=1; 	      
		//CT_IIC_SCL_HIGH;
		digitalWrite(gcptscl,HIGH);
		delayMicroseconds(10);
		//CT_IIC_SCL_LOW;	
		digitalWrite(gcptscl,LOW);
		delayMicroseconds(10);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t CPTIIC::CT_IIC_Read_Byte(uint8_t ack)
{
	uint8_t i,receive=0;
 	pinMode(gcptsda, INPUT);      //SDA设置为输入 
    for(i=0;i<8;i++ )
	{
        //CT_IIC_SCL_LOW;
        digitalWrite(gcptscl,LOW);
		delayMicroseconds(30);
		//CT_IIC_SCL_HIGH;  
		digitalWrite(gcptscl,HIGH);
		receive<<=1;
		if(digitalRead(gcptsda))
		{
			receive += 1;
		}
		else
		{
			receive += 0;
		}
	}	  				 
	if (!ack)CT_IIC_NAck();//发送nACK
	else CT_IIC_Ack(); //发送ACK   
 	return receive;
}




























