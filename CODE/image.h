#ifndef _image_h
#define _image_h
#include "headfile.h"

#define dandiaoyuzhi 7

extern uint8 leftline_ave_point ;//左平均点
extern uint8 rightline_ave_point;//右平均点
extern uint8 error_flag;
extern uint8 car_over_flag ;//车全局变量
extern uint8 start_h; //起始高 压缩后图像大小(60,94)
extern uint8 start_w; //起始宽（60，47）
extern uint8 find_left_line[60];   //左线数组
extern uint8 find_right_line[60];  //右线数组
extern uint8 lose_left_line[60];     //左丢线
extern uint8 lose_right_line[60];    //右丢线
extern uint8 centre_point[60];
extern uint8   error;
extern uint8 ShiziFlag ;      //十字标志位
extern uint8 guaidian_lie_left;           //左拐点列数
extern uint8 guaidian_lie_right;          //右拐点列数
extern int16 error_w;       //中心偏离
extern uint8  lose_left,lose_right,lose_left_flag,lose_right_flag,left_width_flag,right_width_flag;
extern uint8 left_width[60];     //左宽度
extern uint8 right_width[60];    //右宽度
extern int     left_dandiao,right_dandiao;
extern uint8   num_width[60];          //总宽度
void gary2binaryzation(uint8 (*yasuo_image)[94]);
void find2centerline(uint8  (*yasuo_image_t)[94]);
void  Bin_Image_Filter(uint8  (*yasuo_image)[94]);
uint8   find_huandaorukou_buxian(void);
void error_num(int8 N,uint8 hang);
void First_Scanf(void);
int   find_line_dandiao ( uint8 linesuzhu[],uint8 flag);
uint8 find_huxian(  uint8 bianxian[],uint8 status,uint8 numb,uint8 *hudian);
void GameCar(void);
void bianxian_buxian(void);
void Huandao_Process(uint8 *flag);
void Huandao_Process_two(uint8 rightline[] ,uint8 leftline[],uint8 *flag);
void AddLine(uint8 leftline[],uint8 rightline[],uint8 start_hang,uint8 start_lie,uint8 end_hang,uint8 end_lie,uint8 flag);
void Lcd_ShowLine(uint8 bianxian[],uint16 color);
void min2_way(uint8 data_line[60],int8 N);
float min2_line_K(uint8  data_line[60],int N);
float min2_line_b (uint8  data_line[60],int8 N,double K);
float min2_line(uint8 data_line[],int8 N,uint8 flag);
uint8 Find_guaidian(uint8 bianxian[],uint8 flag,uint8 UD);
uint8 UpAddLine(uint8 StartLine,uint8 StartLie,uint8 EndLine);
uint8 min2_dandiao(uint8 lineshuzu[],uint8 nihe_num);
void find2centerline_sobel(uint8  (*yasuo_image_t)[94]);
void Find_ShangBianxian(uint8  (*image)[94]);
uint8 find_UpGuaidian(uint8 shangbianxian[]);       //上边线拐点
void First_Scanf_Sobel(void);
uint8 Find_zuoshangguaidain(uint8 bianxian[]);
void yuansu_get(void);
void TFT_Shows(void);
uint8 Find_Hudian(uint8 bianxian[],uint8 flag,uint8 startpoint);        //边线上拐点
uint8 Find_upguaidian(uint8 bianxian[],uint8 flag,uint8 starthang);
void P_RoadProcess(uint8 *flag);
uint8 Find_PRoad(void);
void sancha_precess(uint8 *mode);
uint8 sancha_get(void);
uint8 Find_sanchaguaidian(uint8 bianxian[],uint8 flag);
uint8 find_dingdian(void);
void shizi_process(uint8 *flag);
uint8 find_shizi_buxian(void);
uint8 find_shizi_in_buxian(void);
uint8 PH_general_process(void);
void ServoPID(void);
uint8 road_width_change(uint8  flag);
void PH_shibie(uint8 flag);
uint8 road_line_judge(uint8 flag);
uint8 out_circle_judge(uint8 find_hang);
uint16 garage_judge(void);
void garage_stop(uint8 flag);
void garage_process(void);
uint8 find_diuxian(uint8 hang);
uint8 find_diuxian_line(uint8 hang,uint8 bianxian[]);
uint8 find_garage_line_num(uint8 bianxian[],uint8 flag);
uint8 find_sancha_guai(uint8 hang);
void Out_GarageProcess(void);
void Speed_Conctrl(void);
uint8 find_diuxian_garage(uint8 hang);
uint8 find_diuxian_huandao(uint8 hang,uint8 flag);
void ramp_judge(void);
void  ramp_process();
void Car_Recieve(void);
void garge_scanf_line(uint8 flag);
#endif
