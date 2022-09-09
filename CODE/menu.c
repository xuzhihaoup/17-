#include "headfile.h"
#include  "menu.h"
#include  "image.h"
#include "shexiangtou.h"
#include "zf_stm_systick.h"
#include "math.h"
#include "stdlib.h"
#include "motor.h"
#include  "key.h"
#include <string.h>


char buf1[1024];//������
uint8 set_flag = 0;
/*---------------------------------------------------------------
 ����    ����menu_Shows
 ����    �ܡ��˵���ʾ
 ����    ������
 ���� �� ֵ����
 ��ע�����2022/05/05
 ----------------------------------------------------------------*/
//void menu_Shows(void)
//{
//    //TFT_Shows();
//    sprintf(buf1,"In:%d",In);
//    lcd_showstr(30,6,buf1);
//
//    sprintf(buf1,"Back:%d",Back);
//    lcd_showstr(30,7,buf1);
//
//    sprintf(buf1,"Ok:%d",Ok);
//    lcd_showstr(30,8,buf1);
//
//    sprintf(buf1,"Dwon:%d",DWON);
//    lcd_showstr(30,9,buf1);
//
//}
uint16 Right_pwm=900,Mid_pwm=980,Left_pwm=1060,kcd=0, left=0;
//�����˵�ҳ��ṹ������
//һ���˵�
Menu menu1_main[3];
//�����˵�
Menu menu2_duoji[3];
Menu menu2_speed[1];
Menu menu2_pid[1];
//�����˵�
Menu menu3_duoji[3];
//�ṹ���ʼ��//�˵�����,�����ｫÿһ���˵��Ĺ������ú�
Menu menu1_main[3] = // ��1�� ���˵�
{
    {3, "duoji",'',  TYPE_SUBMENU, NULL, menu2_duoji, NULL},
    {3, "speed",'', TYPE_SUBMENU, NULL, menu2_speed, NULL},
    {3, "pid"  ,'',  TYPE_SUBMENU, NULL, menu2_pid, NULL}
};
Menu menu2_duoji[3] =
{
   {3, "Right_pwm", &Right_pwm,  TYPE_SUBMENU, NULL, menu3_duoji, menu1_main},
   {3, "Mid_pwm",  &Mid_pwm,      TYPE_SUBMENU, NULL, menu3_duoji, menu1_main},
   {3, "Left_pwm", &Left_pwm,    TYPE_SUBMENU, NULL, menu3_duoji, menu1_main}
};
Menu menu3_duoji[3] =
{
        {3, "Right_pwm", &Right_pwm,  TYPE_PARAM, NULL, NULL, menu2_duoji},
        {3, "Mid_pwm",   &Mid_pwm,    TYPE_PARAM, NULL, NULL, menu2_duoji},
        {3, "Left_pwm",  &Left_pwm,   TYPE_PARAM, NULL, NULL, menu2_duoji}
};
Menu menu2_speed[1] =
{
   {1, "left",&left,TYPE_SUBMENU, NULL, NULL, menu1_main}
};
Menu menu2_pid[1] =
{
   {1, "kd", &kcd, TYPE_SUBMENU, NULL, NULL, menu1_main}
};
//����˵�������Ҫ��ȫ�ֱ���
Menu *cur_item = menu1_main;  //��ʼ����ǰ�˵�Ϊ��һ��(main_menu)
Menu *prev_item = NULL;     //��ʼ����һ���˵�Ϊ��
uint8  item_index = 0;//��ǰ�˵�������
/*---------------------------------------------------------------
 ����    ����Disp_menu
 ����    �ܡ����Ʋ˵�
 ����    ������
 ���� �� ֵ����
 ��ע�����2022/05/06
 ----------------------------------------------------------------*/
void Disp_menu(void)
{
    char item_label[1024] ;
    char buf4[1024] ;
    char buf3[1024] ;
    uint8 menu_num = cur_item[0].num; //��ȡ��ǰ�˵�����Ŀ����
    uint8 i,num = menu_num > MENU_MAX_ROW ? MENU_MAX_ROW : menu_num;
    lcd_clear(WHITE);
    if(item_index>=MENU_MAX_ROW || item_index>=menu_num)//�˵�������ѡ���Ƿ�Խ��
        {
            if(item_index==0XFF)  //�����Ϊ item_index=0 �ٰ��ϼ���0-1=0XFF
            {
                item_index = menu_num - 1;   //ѭ�����ص����һ����������ֵ
            }
            if(item_index>=menu_num)  //�����Ϊ����������һ����������ֵ
            {
                item_index = 0;  //ѭ�����ص���һ����������ֵ
            }
            if(item_index>=MENU_MAX_ROW)
            {
                item_index = 0;
            }
        }
    for (i=0; i<num; i++)//����ĳһ���˵��µĹ��ܼ�
    {
    strcpy(item_label,cur_item[i].label);
    if(cur_item!=menu1_main)
    {
            sprintf(buf4,":%d",*cur_item[i].value);
        }
        strcat(item_label,buf4);
        lcd_showstr(30,5+i,item_label); //���Ʊ���
        memset(buf4,'\0',sizeof(buf4)); //�������

                //Ч��
//                ->  pid
//                    speed
//                    duoji
   }

    sprintf(buf3,"->");
    lcd_showstr(15,5+item_index,buf3);//���Ƽ�ͷָʾ
}
/*---------------------------------------------------------------
 ����    ����Disp_menu_KEY
 ����    �ܡ��˵���������
 ����    ������
 ���� �� ֵ����
 ��ע�����2022/05/06
 ----------------------------------------------------------------*/
void Disp_menu_KEY(uint8 key_value)
{
    if(key_value==KEY_UP_PRESS || key_value==KEY_DOWN_PRESS || key_value==KEY_ENTER_PRESS || key_value==KEY_RETURN_PRESS)
        {
            switch(key_value)//��ⰴ����������Ӧ����
            {
                case KEY_UP_PRESS:
                    if(set_flag==1)
                    {
                     *(cur_item[item_index].value)=*(cur_item[item_index].value)+10;
                    }
                    else
                    item_index++;

                    Disp_menu();
                    break;
                case KEY_DOWN_PRESS:
                    if(set_flag==1)
                    {
                     *(cur_item[item_index].value)=*(cur_item[item_index].value)-10;
                    }
                    else
                    item_index--;

                    Disp_menu();
                    break;
                case KEY_ENTER_PRESS:
                    switch(cur_item[item_index].type)//��⹦��������ͣ�������Ӧ����
                    {
                        case TYPE_SUBMENU: //�����Ӳ˵��Ĳ˵���
                            if(cur_item[item_index].next != NULL)
                            {
                                prev_item = cur_item;//�˼��˵��������һ���˵�
                                cur_item = cur_item[item_index].next;//��ָ�����һ���˵�����Ϊ��ǰ�˵�
                                item_index = 0;//�����Ӳ˵�������
                                Disp_menu();  //��ʾ�Ӳ˵�������
                            }
                            break;
                        case TYPE_PARAM:  //���в������õĲ˵���
                            if(cur_item[item_index].Function != NULL)
                            {
                                //������Ӧ�Ķ�������,�����ݲ��� һ��ִ�к���
                                cur_item[item_index].Function((const char *)cur_item[item_index].label);
                            }
                            set_flag = 1;//�������ñ�־λ��һ
                            break;
                        default:
                            break;
                    }
                    break;
                case KEY_RETURN_PRESS:
                    set_flag = 0;//�������ñ�־λ����
                    if (prev_item != NULL)//������һ���˵��Ĳ���
                    {
                        cur_item = prev_item;  //������һ���˵�Ϊ��ǰ�˵�
                        prev_item = cur_item[0].prev;  //���õ�ǰ�˵�����һ���˵�
                        item_index = 0;  //�����Ӳ˵�������
                        Disp_menu();  //��ʾ�˵�������
                    }
                    break;
                default:
                    break;
            }
        }


}



