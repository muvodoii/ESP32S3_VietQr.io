#ifndef __MYCT_IIC_H
#define __MYCT_IIC_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class CPTIIC
{
  public:	
  	CPTIIC(int8_t cptscl, int8_t cptsda);
	//IIC���в�������
	void CT_IIC_Init(int8_t cptscl, int8_t cptsda); 		//��ʼ��IIC��IO��				 
	void CT_IIC_Start(void);				//����IIC��ʼ�ź�
	void CT_IIC_Stop(void); 				//����IICֹͣ�ź�
	void CT_IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
	uint8_t CT_IIC_Read_Byte(uint8_t ack); //IIC��ȡһ���ֽ�
	uint8_t CT_IIC_Wait_Ack(void);				//IIC�ȴ�ACK�ź�
	void CT_IIC_Ack(void);					//IIC����ACK�ź�
	void CT_IIC_NAck(void); 				//IIC������ACK�ź�
	int8_t gcptsda,gcptscl;
		
  private:
// 	#ifdef __AVR__
		volatile uint8_t *cptsclPort, *cptsdaPort;
		uint8_t  cptsclPinSet, cptsdaPinSet;
//	#endif
};

#endif







