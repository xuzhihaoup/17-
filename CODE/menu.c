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


char buf1[1024];//缓存区
uint8 set_flag = 0;
/*---------------------------------------------------------------
 【函    数】menu_Shows
 【功    能】菜单显示
 【参    数】无
 【返 回 值】无
 【注意事项】2022/05/05
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
//各级菜单页面结构体声明
//一级菜单
Menu menu1_main[3];
//二级菜单
Menu menu2_duoji[3];
Menu menu2_speed[1];
Menu menu2_pid[1];
//三级菜单
Menu menu3_duoji[3];
//结构体初始化//菜单定义,在这里将每一个菜单的关联设置好
Menu menu1_main[3] = // 第1级 主菜单
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
//定义菜单操作需要的全局变量
Menu *cur_item = menu1_main;  //初始化当前菜单为第一级(main_menu)
Menu *prev_item = NULL;     //初始化上一级菜单为空
uint8  item_index = 0;//当前菜单项索引
/*---------------------------------------------------------------
 【函    数】Disp_menu
 【功    能】绘制菜单
 【参    数】无
 【返 回 值】无
 【注意事项】2022/05/06
 ----------------------------------------------------------------*/
void Disp_menu(void)
{
    char item_label[1024] ;
    char buf4[1024] ;
    char buf3[1024] ;
    uint8 menu_num = cur_item[0].num; //获取当前菜单的项目数量
    uint8 i,num = menu_num > MENU_MAX_ROW ? MENU_MAX_ROW : menu_num;
    lcd_clear(WHITE);
    if(item_index>=MENU_MAX_ROW || item_index>=menu_num)//菜单项上下选择是否越界
        {
            if(item_index==0XFF)  //此情况为 item_index=0 再按上键，0-1=0XFF
            {
                item_index = menu_num - 1;   //循环，回到最后一个功能索引值
            }
            if(item_index>=menu_num)  //此情况为到达最下面一个功能索引值
            {
                item_index = 0;  //循环，回到第一个功能索引值
            }
            if(item_index>=MENU_MAX_ROW)
            {
                item_index = 0;
            }
        }
    for (i=0; i<num; i++)//绘制某一级菜单下的功能键
    {
    strcpy(item_label,cur_item[i].label);
    if(cur_item!=menu1_main)
    {
            sprintf(buf4,":%d",*cur_item[i].value);
        }
        strcat(item_label,buf4);
        lcd_showstr(30,5+i,item_label); //绘制标题
        memset(buf4,'\0',sizeof(buf4)); //清除缓存

                //效果
//                ->  pid
//                    speed
//                    duoji
   }

    sprintf(buf3,"->");
    lcd_showstr(15,5+item_index,buf3);//绘制箭头指示
}
/*---------------------------------------------------------------
 【函    数】Disp_menu_KEY
 【功    能】菜单按键解析
 【参    数】无
 【返 回 值】无
 【注意事项】2022/05/06
 ----------------------------------------------------------------*/
void Disp_menu_KEY(uint8 key_value)
{
    if(key_value==KEY_UP_PRESS || key_value==KEY_DOWN_PRESS || key_value==KEY_ENTER_PRESS || key_value==KEY_RETURN_PRESS)
        {
            switch(key_value)//检测按键，进入相应动作
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
                    switch(cur_item[item_index].type)//检测功能项的类型，进入相应动作
                    {
                        case TYPE_SUBMENU: //具有子菜单的菜单项
                            if(cur_item[item_index].next != NULL)
                            {
                                prev_item = cur_item;//此级菜单变成了上一级菜单
                                cur_item = cur_item[item_index].next;//将指向的下一级菜单设置为当前菜单
                                item_index = 0;//重置子菜单项索引
                                Disp_menu();  //显示子菜单项索引
                            }
                            break;
                        case TYPE_PARAM:  //具有参数设置的菜单项
                            if(cur_item[item_index].Function != NULL)
                            {
                                //调用相应的动作函数,并传递参数 一键执行函数
                                cur_item[item_index].Function((const char *)cur_item[item_index].label);
                            }
                            set_flag = 1;//参数设置标志位置一
                            break;
                        default:
                            break;
                    }
                    break;
                case KEY_RETURN_PRESS:
                    set_flag = 0;//参数设置标志位清零
                    if (prev_item != NULL)//返回上一级菜单的操作
                    {
                        cur_item = prev_item;  //设置上一级菜单为当前菜单
                        prev_item = cur_item[0].prev;  //设置当前菜单的上一级菜单
                        item_index = 0;  //重置子菜单项索引
                        Disp_menu();  //显示菜单项索引
                    }
                    break;
                default:
                    break;
            }
        }


}



