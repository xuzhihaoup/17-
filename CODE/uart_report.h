#ifndef _uart_report
#define _uart_report
#include "headfile.h"


/**********为了匿名四轴上位机的协议定义的变量****************************/
//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	 //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	

void sent_data(int16 A,int16 B,int16 C,int16 D);
#endif
