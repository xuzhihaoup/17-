#include "headfile.h"

#include "uart_report.h"

uint8 BUFF[30];

void sent_data(int16 A,int16 B,int16 C,int16 D)
{
	uint8 i;
	uint8 sumcheck = 0;
	uint8 addcheck = 0;
	uint8 _cnt=0;
	BUFF[_cnt++]=0xAA;//帧头
	BUFF[_cnt++]=0xFF;//目标地址
	BUFF[_cnt++]=0XF1;//功能码
	BUFF[_cnt++]=0x08;//数据长度
	BUFF[_cnt++]=BYTE0(A);//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE1(A);//需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);	
	BUFF[_cnt++]=BYTE0(C);
	BUFF[_cnt++]=BYTE1(C);
	BUFF[_cnt++]=BYTE0(D);
	BUFF[_cnt++]=BYTE1(D);
	//SC和AC的校验直接抄最上面上面简介的即可
	for(i=0;i<BUFF[3]+4;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	
	for(i=0;i<_cnt;i++) 
		uart_putchar(UART_0,BUFF[i]);//串口逐个发送数据
}

