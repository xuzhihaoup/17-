#include "headfile.h"
#include "key.h"

/*---------------------------------------------------------------
 ����    ����Key_Init
 ����    �ܡ�������ʼ��
 ����    ������
 ���� �� ֵ����
 ��ע�����2022/03/23
 ----------------------------------------------------------------*/
void Key_Init(void)
{
    //������ʼ��
        gpio_init(KEY1,GPI,0,PULLUP);
        gpio_init(KEY2,GPI,0,PULLUP);
        gpio_init(KEY3,GPI,0,PULLUP);
        gpio_init(KEY4,GPI,0,PULLUP);
}
/*---------------------------------------------------------------
 ����    ����Key_Proc
 ����    �ܡ���������
 ����    ������
 ���� �� ֵ����
 ��ע�����2022/03/23
 ----------------------------------------------------------------*/
extern uint8 HuandaoFlag;
extern uint8 HuandaoGet;
extern uint8 shit;
extern uint8 keyflag;
extern int32 leftspeed;
extern int32 rightspeed;
extern int16 lefttargetspeed,righttargetspeed;
extern uint8 Game_Start;
extern uint8 Out_Garage_flag;
uint8 lcd_flag = 1;
void Key_Proc(void)
{
    if(!gpio_get(KEY1))
    {
        Game_Start = 1;
        Out_Garage_flag = 1;            //�����
    }
//    while(!gpio_get(KEY1));
    if(!gpio_get(KEY4))
    {
        Game_Start = 2;
    }
//    while(!gpio_get(KEY4));
    if(!gpio_get(KEY3))
    {
        Out_Garage_flag = 2;            //�ҳ���
    }
//    while(!gpio_get(KEY3));
    if(!gpio_get(KEY2))
    {
        Game_Start = 1;
        Out_Garage_flag = 2;            //�ҳ���
    }
 //   while(!gpio_get(KEY2));
}
/*---------------------------------------------------------------
 ����    ����Key_Scan
 ����    �ܡ�����ɨ��
 ����    ������
 ���� �� ֵ����
 ��ע�����2022/05/05
 ----------------------------------------------------------------*/
uint8 In=0, Back=0, Ok=0, DWON=0;
uint8 Key_Scan(void)
{
    if(!gpio_get(KEY1)==1||!gpio_get(KEY2)==1||!gpio_get(KEY3)==1)
    {
        systick_delay_ms(STM0, 100);
        if(!gpio_get(KEY1))         return 1;
        else if (!gpio_get(KEY2))   return 2;
        else if (!gpio_get(KEY3))   return 3;
    }
    return 0;
}
