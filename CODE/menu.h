#ifndef   _menu_h
#define   _menu_h
#include "headfile.h"



#define MENU_MAX_ROW  3 // 菜单最大显示行数



#define TYPE_SUBMENU     101  //具有子菜单的菜单项
#define TYPE_PARAM       102  //参数项（用于执行参数设置）



#define KEY_ENTER_PRESS  1 //对应KEY1
#define KEY_RETURN_PRESS 2 //对应KEY2
#define KEY_UP_PRESS     3 //对应KEY3
#define KEY_DOWN_PRESS   4 //对应KEY4


typedef void (*MENU_FUN)(const char *);
//定义菜单
typedef struct menu
{
    uint8 num;        //当前菜单功能项总数
//    char * title;       //当前菜单标题
    char   *label;       //功能项标题
    uint16 *value;       //参数值
    uint8 type;         //当前功能项的类型
    MENU_FUN Function;  //选择某一功能后执行的功能函数
    struct menu *next;  //下一级菜单
    struct menu *prev;  //上一级菜单

} Menu;
extern uint16 Right_pwm,Mid_pwm,Left_pwm,kcd,left;
void menu_Shows(void);
void Disp_menu(void);


#endif
