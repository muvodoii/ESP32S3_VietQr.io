#ifndef __FT5426_DRIVER_H
#define __FT5426_DRIVER_H
#include "cptiic.h"

#define TOUCH_MAX 5

//IIC command
#define FT_IIC_CMD_WR   0x70    //write command
#define FT_IIC_CMD_RD   0x71    //read command

//FT5426 regist define
#define FT_DEVIDE_MODE 			0x00   		//FT5206ģʽ���ƼĴ���
#define FT_REG_NUM_FINGER       0x02		//����״̬�Ĵ���

#define FT_TP1_REG 				0X03	  	//��һ�����������ݵ�ַ
#define FT_TP2_REG 				0X09		//�ڶ������������ݵ�ַ
#define FT_TP3_REG 				0X0F		//���������������ݵ�ַ
#define FT_TP4_REG 				0X15		//���ĸ����������ݵ�ַ
#define FT_TP5_REG 				0X1B		//��������������ݵ�ַ  
 

#define	FT_ID_G_LIB_VERSION		0xA1		//�汾		
#define FT_ID_G_MODE 			0xA4   		//FT5426�ж�ģʽ���ƼĴ���
#define FT_ID_G_THGROUP			0x80   		//������Чֵ���üĴ���
#define FT_ID_G_PERIODACTIVE	0x88   		//����״̬�������üĴ���

class FT5426:public CPTIIC
{
  public:
	FT5426(int8_t cptint, int8_t cptrst,int8_t cptscl, int8_t cptsda);	
	uint8_t FT5426_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len);
	void FT5426_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len);
	uint8_t FT5426_Init(uint8_t r,uint16_t w, uint16_t h);
	uint8_t FT5426_Scan(void); 
	
  	uint8_t ctp_status,lcd_r;
  	uint16_t x[TOUCH_MAX],y[TOUCH_MAX],lcd_w,lcd_h;
  	
  private:
 #ifdef __AVR__
	volatile uint8_t *cptintPort, *cptrstPort;
	uint8_t  cptintPinSet, cptrstPinSet;
 #else
	volatile uint32_t *cptintPort, *cptrstPort;
	uint32_t  cptintPinSet, cptrstPinSet;
#endif
};


#endif
