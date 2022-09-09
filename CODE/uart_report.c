#include "headfile.h"

#include "uart_report.h"

uint8 BUFF[30];

void sent_data(int16 A,int16 B,int16 C,int16 D)
{
	uint8 i;
	uint8 sumcheck = 0;
	uint8 addcheck = 0;
	uint8 _cnt=0;
	BUFF[_cnt++]=0xAA;//֡ͷ
	BUFF[_cnt++]=0xFF;//Ŀ���ַ
	BUFF[_cnt++]=0XF1;//������
	BUFF[_cnt++]=0x08;//���ݳ���
	BUFF[_cnt++]=BYTE0(A);//��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++]=BYTE1(A);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);	
	BUFF[_cnt++]=BYTE0(C);
	BUFF[_cnt++]=BYTE1(C);
	BUFF[_cnt++]=BYTE0(D);
	BUFF[_cnt++]=BYTE1(D);
	//SC��AC��У��ֱ�ӳ�������������ļ���
	for(i=0;i<BUFF[3]+4;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	
	for(i=0;i<_cnt;i++) 
		uart_putchar(UART_0,BUFF[i]);//���������������
}

