#ifndef   _menu_h
#define   _menu_h
#include "headfile.h"



#define MENU_MAX_ROW  3 // �˵������ʾ����



#define TYPE_SUBMENU     101  //�����Ӳ˵��Ĳ˵���
#define TYPE_PARAM       102  //���������ִ�в������ã�



#define KEY_ENTER_PRESS  1 //��ӦKEY1
#define KEY_RETURN_PRESS 2 //��ӦKEY2
#define KEY_UP_PRESS     3 //��ӦKEY3
#define KEY_DOWN_PRESS   4 //��ӦKEY4


typedef void (*MENU_FUN)(const char *);
//����˵�
typedef struct menu
{
    uint8 num;        //��ǰ�˵�����������
//    char * title;       //��ǰ�˵�����
    char   *label;       //���������
    uint16 *value;       //����ֵ
    uint8 type;         //��ǰ�����������
    MENU_FUN Function;  //ѡ��ĳһ���ܺ�ִ�еĹ��ܺ���
    struct menu *next;  //��һ���˵�
    struct menu *prev;  //��һ���˵�

} Menu;
extern uint16 Right_pwm,Mid_pwm,Left_pwm,kcd,left;
void menu_Shows(void);
void Disp_menu(void);


#endif
