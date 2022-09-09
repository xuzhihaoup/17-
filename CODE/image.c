#include  "image.h"
#include "headfile.h"
#include "shexiangtou.h"
#include "zf_stm_systick.h"
#include "math.h"
#include "stdlib.h"
#include "motor.h"


/*            全局               */
uint16 black_patch = 0;       //斑马线黑点
uint16 white_patch = 0;       //斑马线白点

uint8 leftline_ave_point =  27;//左平均点
uint8 rightline_ave_point = 57;//右平均点
uint8 dingdian_hang;
uint8 dingdian_lie;
uint8 sancha_guaidian_hang_left;
uint8 sancha_guaidian_hang_right;
uint8 sancha_guaidian_lie_left;
uint8 sancha_guaidian_lie_right;
uint8 lukuan_flag = 0,dingdian_flag = 0,line_flag = 0;
uint8 sanchaflag  = 0;    //三叉标志位
uint8 in_sanchaflag = 0;
uint8 out_sanchaflag = 0;
uint8 sancha_mode = 1;    //进出三叉  1左 0 右
uint8 hudian_hang,hudian_lie;       //弧点行弧点列
uint8 Up_right_guaidian_hang,Up_right_guaidian_lie;
uint8 Up_left_guaidian_hang,Up_left_guaidian_lie;   //上拐点行上拐点列
uint8 HuandaoFlag = 0;      //环岛标志位
uint8 Stop_car   = 0;      //停车标志位
uint8 stop = 0;
uint8 HuandaoIn = 0;      //环岛方向标志位
uint8 ShiziFlag = 0;      //十字标志位
uint8 ShiziIn = 0;          //十字进入标志位
uint8 zuohuxian_flag = 0,youhuxian_flag = 0;        //左线拐点和右线拐点标志位
uint8 guaidian_hang_left;           //左拐点行数
uint8 guaidian_hang_right;          //右拐点行数
uint8 garage_hang;              //车库行
uint8 garage_lie;               //车库列
uint8 Up_guaidian_hang;                 //上拐点赛道宽度拐点
uint8 guaidian_lie_left;           //左拐点列数
uint8 guaidian_lie_right;          //右拐点列数
uint8 car_over_flag = 0;  //车全局变量
uint8 go_ok_huandao_flag = 0;//成功入环标志  入环置1  出环置0
uint8 to_ok_huandao_flag = 1;//成功出环标志  入环置0  出环置1
uint8  start_h=59;           //起始高（60，47）(60 41)  (60 51)
uint8  start_w=47;             //起始宽（60，47）(60 41)  (60 51)
uint8 last_center_point=47;   //扫线位置标志位     初值位首行中线值
uint8 find_left_line[60];     //左线数组
uint8 find_right_line[60];    //右线数组
uint8 centre_point[60];       //中线数组
uint8 lose_left_line[60];     //左丢线数组
uint8 lose_right_line[60];    //右丢线数组
uint8 left_head_line[60];     //左线决策数组
uint8 right_head_line[60];    //右线决策数组
uint8 mid_head_line[60];        //中线决策数组
uint8 find_up_line[94];
uint8 find_down_line[94];
uint8  lose_left=0,lose_right=0,lose_left_flag,lose_right_flag,left_width_flag,right_width_flag;   //左右丢线数
uint8 left_width[60];     //左宽度
uint8 right_width[60];    //右宽度
uint8 num_width[60];          //总宽度
float bianxian_K;
float bianxian_B;
int left_dandiao=0,right_dandiao=0;       //左右单调标志
int32 leftspeed=0,rightspeed=0;
uint8 shiziflag;
extern int16 ServoDuty;
/*---------------------------------------------------------------
 【函    数】gary2binaryzation
 【功    能】二值化
 【参    数】传入图像数组
 【返 回 值】无
 【注意事项】255白  0 黑  1红
 ----------------------------------------------------------------*/
void gary2binaryzation(uint8  (*yasuo_image)[94])
{
    uint8 i,j;

    for(i=0;i<60;i++)
        {
            for(j=0;j<94;j++)
            {
                if(yasuo_image[i][j]>BlackThres)        { bin_image[i][j]=255;xianshi[i][j] = 255;find_line_image[i][j] = 255;}
                else                                    { bin_image[i][j]=0;xianshi[i][j] = 0;find_line_image[i][j] = 0;}
            }
        }
}
/*---------------------------------------------------------------
 【函    数】Bin_Image_Filter
 【功    能】过滤噪点
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void  Bin_Image_Filter(uint8  (*yasuo_image)[94])
{
    uint8 row;   //行
    uint8 line;  //列  对压缩后的（60行，94列）进行处理
    for(row=1;row<start_h;row++)
    {
        for(line=1;line<93;line++)
        {
            if( (yasuo_image[row][line]==0)// 0黑    255白
                 &&yasuo_image[row-1][line]+yasuo_image[row+1][line]+yasuo_image[row][line+1]+yasuo_image[row][line-1] > 510)
            {
                yasuo_image[row][line] = 255;
            }
            else if( (yasuo_image[row][line]==255)
                     &&yasuo_image[row-1][line]+yasuo_image[row+1][line]+yasuo_image[row][line-1]+yasuo_image[row][line+1] < 510)
            {
                yasuo_image[row][line] = 0;
            }

        }
    }
}
/*---------------------------------------------------------------
 【函    数】find2centerline_sobel
 【功    能】寻找中线获取各种数据(sobel)
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void find2centerline_sobel(uint8  (*yasuo_image_t)[94])
{
    uint8 i,j;
    uint8 picflag = 1;
      /*
                 *首行处理
                 *首行处理
                 *首行处理
                 *首行处理
                 *首行处理
      */

       //各个数组复位清零
            for(i=0;i<60;i++)
            {
               find_left_line[i]   = 0;
               find_right_line[i]  = 0;
               centre_point[i]     = 0;
               lose_left_line[i]   = 0;
               lose_right_line[i]  = 0;
               left_width[i]       = 0;
               right_width[i]      = 0;
               num_width[i]        = 0;
            }
               lose_left           = 0;
               lose_right          = 0;
            if(yasuo_image_t[start_h][44]==255&&
               yasuo_image_t[start_h][43]==255&&
               yasuo_image_t[start_h][45]==255&&
               yasuo_image_t[start_h][47]==255&&
               yasuo_image_t[start_h][48]==255&&
               yasuo_image_t[start_h][46]==255)           //从59行开始，因为60行噪点太多
            {
                last_center_point=47;
            }
            /*
                                        第一行中点为黑线，非正常查找
                                        第一行中点为黑线，非正常查找
//            */
            else
            {
                First_Scanf_Sobel();
                last_center_point = (find_right_line[i] +find_left_line[i])/2;
            }
     for(i=start_h;i>20;i--)  //从59行开始，因为60行噪点太多
    {

         //每行标志复位
         left_width_flag = 0;
         right_width_flag= 0;
/******************************************/
         //左线  向左扫  起始为哨兵位子 防止数组越界
         /*
                            第一行中点为白线，正常查找
                            第一行中点为白线，正常查找
          */


             for(j=last_center_point;j>4;j--)//扫描左边，条件：左边扫描到黑线、当前点与边缘距离大于4像素（留出查找的余地）
             {      // 0黑       255白
                    left_width_flag++;//左赛道宽度+1
                    if((yasuo_image_t[i][j]==0          &&
                            yasuo_image_t[i][j+3]==255    &&
                            yasuo_image_t[i][j-3]==255))
                    {      //
                             find_left_line[i]  =  j;//左线记录所在列  不使用他防止多次处理改变边线数组
                             left_head_line[i]  =  j;//决策数组 也就是最终使用的边线
                             xianshi[i][j] = 3;
                             xianshi[i][j-1] = 3;
                             lose_left_flag     =  0;//左丢线标志
                             break;
                    }
                    else
                    {
                             find_left_line[i] = 0;//无边线强制赋值
                             left_head_line[i] = 0;
                             lose_left_flag = 1;
                    }

             }
                        if(lose_left_flag)
                        {
                            lose_left++;              //左丢线数
                            lose_left_line[i]=1;      //具体丢线行
                        }
             left_width[i]=left_width_flag;//每行左赛道宽度
             left_width_flag=0;    //赛道宽度置零



             //右线  向右扫   起始为哨兵位子防止越界
             for(j=last_center_point;j<89;j++)
             {      // 0黑  255白
                     right_width_flag++;
                     if((yasuo_image_t[i][j]==0      &&
                         yasuo_image_t[i][j-2]==255  &&
                         yasuo_image_t[i][j+2]==255))
                     {
                             find_right_line[i] =  j;  //左线记录所在列  不使用他防止多次处理改变边线数组
                             right_head_line[i]  = j;  //决策数组 也就是最终使用的边线
                             xianshi[i][j+1] = 4;
                             xianshi[i][j] = 4;
                             lose_right_flag    =  0;
                             break;
                     }
                     else
                     {
                              find_right_line[i] = 94;//无右线
                              right_head_line[i] = 94;
                              lose_right_flag    =  1;
                     }
             }
             right_width[i] = right_width_flag;   //每行右赛道宽度
             num_width[i]   = right_width_flag+left_width_flag;  //赛道总宽
             //右丢线数
               if(lose_right_flag)
               {
                   lose_right++;          //右丢线数
                   lose_right_line[i]=1;  //具体丢线行
               }


         //计算每行中点            //理想中线start_w=47;
         centre_point[i]=(find_right_line[i] +find_left_line[i])/2;
         if(i<=58)
         {
             if(abs(centre_point[i] - last_center_point)>=10)
             {
                 last_center_point = last_center_point;
                 xianshi[i][last_center_point] = 6;
             }

             else
             {
                 last_center_point = centre_point[i];
                 xianshi[i][last_center_point] = 6;
             }
         }

         //重新装载下一次扫线起始点

     }
}
/*---------------------------------------------------------------
 【函    数】find2centerline
 【功    能】寻找中线获取各种数据
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void find2centerline(uint8  (*yasuo_image_t)[94])
{
    uint8 i,j;
      /*
                 *首行处理
                 *首行处理
                 *首行处理
                 *首行处理
                 *首行处理
      */

       //各个数组复位清零
            for(i=0;i<60;i++)
            {
               find_left_line[i]   = 0;
               find_right_line[i]  = 0;
               centre_point[i]     = 0;
               lose_left_line[i]   = 0;
               lose_right_line[i]  = 0;
               left_width[i]       = 0;
               right_width[i]      = 0;
               num_width[i]        = 0;
            }
               lose_left           = 0;
               lose_right          = 0;

            if( yasuo_image_t[start_h][37]==255&&
                yasuo_image_t[start_h][38]==255&&
                yasuo_image_t[start_h][39]==255&&
                yasuo_image_t[start_h][40]==255&&
                yasuo_image_t[start_h][41]==255&&
                yasuo_image_t[start_h][42]==255&&
                yasuo_image_t[start_h][44]==255&&
                yasuo_image_t[start_h][43]==255&&
                yasuo_image_t[start_h][45]==255&&
                yasuo_image_t[start_h][47]==255&&    // 255 白  0 黑
                yasuo_image_t[start_h][48]==255&&
                yasuo_image_t[start_h][46]==255&&
                yasuo_image_t[start_h][49]==255&&
                yasuo_image_t[start_h][50]==255&&
                yasuo_image_t[start_h][51]==255&&
                yasuo_image_t[start_h][52]==255&&
                yasuo_image_t[start_h][56]==255&&
                yasuo_image_t[start_h][57]==255)
            {
                last_center_point=47;
            }
            /*
                                        第一行中点为黑线，非正常查找
                                        第一行中点为黑线，非正常查找
            */
            else
            {

                First_Scanf();

                last_center_point = (find_right_line[59] +find_left_line[59])/2;
               //printf("l=%d , r=%d , l=%d \r\n",last_center_point,find_right_line[59],find_left_line[59]);
            }
     for(i=start_h;i>5;i--)
    {

         //每行标志复位
//         left_width_flag = 0;
//         right_width_flag= 0;
/******************************************/
         //左线  向左扫  起始为哨兵位子 防止数组越界
         /*
                            第一行中点为白线，正常查找
                            第一行中点为白线，正常查找
          */


             for(j=last_center_point;j>4;j--)//扫描左边，条件：左边扫描到黑线、当前点与边缘距离大于4像素（留出查找的余地）
             {      // 0黑       255白
                    //left_width_flag++;//左赛道宽度+1
                    if(yasuo_image_t[i][j]==0      &&
                       yasuo_image_t[i][j-1]==0    &&
                       yasuo_image_t[i][(j+1)]==255&&
                       yasuo_image_t[i][(j+2)]==255)
                    {      //白点后4个点为黑   则认定为边线
                             find_left_line[i]  =  j;//左线记录所在列  不使用他防止多次处理改变边线数组
                             left_head_line[i]  =  j;//决策数组 也就是最终使用的边线
                             xianshi[i][j] = 3;
                             xianshi[i][j-1] = 3;
                             lose_left_flag     =  0;//左丢线标志
                             break;
                    }
                    else
                    {
                             find_left_line[i] = 0;//无边线强制赋值
                             left_head_line[i] = 0;
                             lose_left_flag = 1;
                    }

             }
                        if(lose_left_flag)
                        {
                            lose_left++;              //左丢线数
                            lose_left_line[i]=1;      //具体丢线行
                            lose_left_flag=0;
                        }
//             left_width[i]=left_width_flag;//每行左赛道宽度
//             left_width_flag=0;    //赛道宽度置零



             //右线  向右扫   起始为哨兵位子防止越界
             for(j=last_center_point;j<90;j++)
             {      // 0黑  255白
                     //right_width_flag++;
                     if(yasuo_image_t[i][j]==0      &&
                        yasuo_image_t[i][(j+1)]==0  &&
                        yasuo_image_t[i][(j-1)]==255&&
                        yasuo_image_t[i][(j-2)]==255)
                     {
                             find_right_line[i] =  j;  //左线记录所在列  不使用他防止多次处理改变边线数组
                             right_head_line[i]  = j;  //决策数组 也就是最终使用的边线
                             xianshi[i][j] = 4;
                             xianshi[i][j+1] = 4;
                             lose_right_flag    =  0;
                             break;
                     }
                     else
                     {
                              find_right_line[i] = 94;//无右线
                              right_head_line[i] = 94;
                              lose_right_flag    =  1;
                     }
             }
//             right_width[i] = right_width_flag;   //每行右赛道宽度
//             right_width_flag = 0;
             left_width[i]  = abs(47-find_left_line[i]);
             right_width[i]  = abs(find_right_line[i]-47);
             num_width[i]   = find_right_line[i]-find_left_line[i];  //赛道总宽
             //右丢线数
               if(lose_right_flag)
               {
                   lose_right++;          //右丢线数
                   lose_right_line[i]=1;  //具体丢线行
                   lose_right_flag=0;
               }

         //计算每行中点            //理想中线start_w=47;
         //重新装载下一次扫线起始点
         if(lose_left_line[i])
         {
             if(find_right_line[i]!=94)
             {
                 last_center_point = find_right_line[i] - 20;
             }
         }
         else if(lose_right_line[i])
         {
             if(find_left_line[i]!=0)
             {
                 last_center_point = find_left_line[i] + 20;
             }
         }
         else
         {
             last_center_point = (find_right_line[i]+find_left_line[i])/2;

         }
         centre_point[i]   = (find_right_line[i]+find_left_line[i])/2;
         xianshi[i][last_center_point] = 6;
    }
}
/*---------------------------------------------------------------
 【函    数】GameCar
 【功    能】整车程序
 【参    数】无
 【返 回 值】无
 【注意事项】2022/03/23
 ----------------------------------------------------------------*/
extern uint16 vol;          //电压
uint8 keyflag;
uint8 HuandaoGet = 0;
uint8 shiziGet = 0;
extern uint8 mt9v03x_finish_flag;
extern uint8 sys_flag;
uint8 ss = 0 ;

char buf2[1024];

uint8 guaidian_right_hang;        //右拐点的行
uint8 guaidian_right_lie;
uint8 guaidian_left_hang;
uint8 guaidian_left_lie;
uint8 Up_guaidian_Hang;          //拐点向上行
uint8 Up_guaidian_Lie;           //拐点向上列

uint8 dz=0,xz=0;

uint8 Guai_flag;
float right_k = 0;                      //右线斜率
float left_k  = 0;                      //左线斜率

int16 lefttargetspeed  = 0;           //车子目标速度
int16 righttargetspeed = 0;
extern float angle;
uint8 shiziflagflag = 0;

uint8 Game_Start = 0;                   //小车开关
uint8 Out_Garage_flag = 0;              //出库方向标志位    1:左出库      2:右出库
extern uint8  GarageGet;
uint8 Game_Start_OpenSpeed = 1; //速度开关
void GameCar(void)
{
     if(Game_Start == 1&&Out_Garage_flag)
     {
        Out_GarageProcess();
     }
     if(Game_Start == 2)            //出库完之后
     {
         //全局起始速度
         if(Game_Start_OpenSpeed==1)
         {
             lefttargetspeed = 270;
             righttargetspeed = 270;
             Game_Start_OpenSpeed=0;
         }

         yuansu_get();                  //各个元素判断
         min2_way(mid_head_line,20);    //中线拟合
         error_num(20,59);                 //偏差值计算
         ServoPID();
         Speed_Conctrl();

     }


}
/*---------------------------------------------------------------
 【函    数】Huandao_Process
 【功    能】环岛程序
 【参    数】flag:环岛标志位
 【返 回 值】无
 【注意事项】2022/03/17
 ----------------------------------------------------------------*/
uint8 ArcHang,ArcLie;
char buf[1024];
extern uint8 PandHuandaoGet,PandHuandaoFlag;
//  1 左  2 右
void Huandao_Process(uint8 *flag)
{
    static uint8 getArc;
    static uint8 xielv_flag;
    uint8 guai_hang,guai_lie;
    static uint8 chuhuan_flag;
//左环岛
    if(HuandaoIn==1)
    {
        //printf("error_w:%d\r\n",error_w);
        switch(*flag)//第一步，补线
                {
                    case 1:
                        lefttargetspeed = 300;
                        righttargetspeed = 300;
                        find_huxian(0,1,0,0);           //左边线判断xz dz

                        if((xz - dz)>=3)
                        {
                            *flag = 2;
                            printf("左圆环:%d\r\n",*flag);
                        }

                        break;
                    case 2:         //确定左边最低点行，右边补线
//                            lefttargetspeed = 220;
//                            righttargetspeed = 220;
                            find_huxian(0,2,0,0);
                            AddLine(left_head_line,right_head_line,30,20,59,45,1);
                            AddLine(left_head_line,right_head_line,30,0,59,20,0);        //补左线

                            if(xz >= 13)
                            {
                                *flag = 3;
                                printf("左圆环:%d\r\n",*flag);
                            }
                        break;
                    case 3:         //进入环岛，开始找环岛出口
                     //   printf("ServoDuty:%d\r\n",ServoDuty);
                        find_huxian(0,2,0,0);
                        if(Guai_flag==0)
                        {
                            if(xz<=10)          //当递减数量在减少
                           {
                                gpio_set(BEEP, GPIO_HIGH);
                                if(Find_guaidian(find_right_line,2,1))        //寻找拐点
                                {
                                    guaidian_right_lie = find_right_line[guaidian_hang_right];      //拐点列
                                    if(guaidian_hang_right>=40&&(right_width[guaidian_hang_right-4]>=20||
                                                                right_width[guaidian_hang_right-5]>=20||
                                                                right_width[guaidian_hang_right-6]>=20||
                                                                right_width[guaidian_hang_right-10]>=20 ))         //当拐点过于接近时，补线
                                    {
                                        gpio_set(BEEP, GPIO_HIGH);
                                        AddLine(left_head_line,right_head_line,10,45,59,guaidian_right_lie+5,1);//补右线，右拐点到左拐点上端
                                        Guai_flag = 1;
                                        printf("我补线了\r\n");
                                    }

                                }
                           }
                        }

                        if(Guai_flag)
                        {
//                            lefttargetspeed = 190;
//                            righttargetspeed = 190;
                            AddLine(left_head_line,right_head_line,20,20,59,50,1);//补右线，右拐点到左拐点上端&&find_right_line[59]<70&&find_right_line[58]<70&&find_right_line[57]<70&&find_right_line[56]<70
                            if(out_circle_judge(20)==1)
//                                if()
                            {
                                lefttargetspeed = 315;
                                righttargetspeed = 315;
                                *flag = 4;
                                printf("左圆环:%d\r\n",*flag);
                            }

                        }
                        break;
                    case 4:
//                        lefttargetspeed = 250;
//                        righttargetspeed = 250;
                           // AddLine(left_head_line,right_head_line,Up_left_guaidian_hang,Up_left_guaidian_lie,59,find_right_line[59]-35,0);//补左边线
                            error_flag = 3;
//                        left_dandiao =  min2_dandiao(find_left_line,8);
//                        right_dandiao = min2_dandiao(find_right_line,8);
//                        leftspeed = 50;
//                        rightspeed = 50;

                        if(find_diuxian_line(30,lose_left_line) == 0)
                        {
//                            lefttargetspeed = 0;
//                            righttargetspeed = 0;

                            error_flag = 1;
                            *flag = 0;
                            HuandaoGet = 0;
                            Guai_flag = 0;
                            HuandaoIn = 0;
                            xielv_flag = 0;
                            PandHuandaoGet = 0;
                            PandHuandaoFlag = 0;
                            gpio_set(BEEP, GPIO_LOW);
                            printf("左圆环:结束\r\n");
                        }

                        break;
                }

    }
    //右环岛
    if(HuandaoIn==2)
    {
        switch(*flag)//第一步，补线
            {
                case 1:

                    lefttargetspeed = 300;
                    righttargetspeed = 300;
                    find_huxian(0,2,0,0);           //右边线判断xz dz


                    if((dz-xz)>=5)
                    {
                        *flag = 2;
                        printf("右圆环:%d\r\n",*flag);
                    }

                    break;      //回到while循环
                case 2:         //确定左边最低点行，右边补线
//                        lefttargetspeed = 350;
//                        righttargetspeed = 350;
                        find_huxian(0,1,0,0);       //观察左边线单调递减情况
                        AddLine(left_head_line,right_head_line,30,80,59,45,0);
                        AddLine(left_head_line,right_head_line,30,94,59,70,1);
                        if(dz >= 13&&find_left_line[59]>=20&&find_left_line[58]>=20&&find_left_line[56]>=20&&find_left_line[55]>=20)
                        {
                            *flag = 3;
                            printf("右圆环:%d\r\n",*flag);
                        }

                    break;
                case 3:         //进入环岛，开始找环岛出口
//                    lefttargetspeed = 400;
//                    righttargetspeed = 400;
                    find_huxian(0,1,0,0);
                    if(Guai_flag==0)
                    {
                        if(dz<12)
                        {
                            if(Find_guaidian(find_left_line,1,1))
                            {
                                guaidian_left_lie = find_left_line[guaidian_hang_left];
                                if(guaidian_hang_left>=40&&(left_width[guaidian_hang_left-4]>=20||
                                                            left_width[guaidian_hang_left-5]>=20||
                                                            left_width[guaidian_hang_left-6]>=20))
                                {

                                    gpio_set(BEEP, GPIO_HIGH);
                                    AddLine(left_head_line,right_head_line,30,45,59,guaidian_left_lie-5,0);//补左线，左拐点到右拐点上端
                                    Guai_flag = 1;
            //                        lefttargetspeed = 300;
            //                        righttargetspeed = 300;
                                }
                            }
                        }
                    }

                    if(Guai_flag)
                    {
//                        AddLine(left_head_line,right_head_line,40,80,59,50,0);//补左线，左拐点到右拐点上端               //小环    R50   补线
                        AddLine(left_head_line,right_head_line,20,55,59,guaidian_left_lie-5,0);//补左线，左拐点到右拐点上端               //小环    R60   补线
                        gpio_set(BEEP, GPIO_HIGH);//&&find_left_line[59]>20&&find_left_line[58]>20&&find_left_line[57]>20&&find_left_line[56]>20
                        if(out_circle_judge(20)==1)
                        {
                           *flag = 4;
//                           Game_Start = 0;
                           printf("右圆环:%d\r\n",*flag);
                        }

                    }
                    break;
                case 4:

//                    if(Find_upguaidian(find_right_line,1,55))
//                    {
                        //AddLine(left_head_line,right_head_line,Up_right_guaidian_hang,Up_right_guaidian_lie,59,find_left_line[59]+35,1);//补右边线
                        error_flag = 2;
//                    }

//                    left_dandiao =  min2_dandiao(find_left_line,10);
//                    right_dandiao = min2_dandiao(find_right_line,10);

                    if(find_diuxian_line(30,lose_right_line) == 0)
                    {
                        error_flag = 1 ;
                        *flag = 0;
                        HuandaoGet = 0;
                        ss = 0;
                        Guai_flag = 0;
                        PandHuandaoGet = 0;
                        PandHuandaoFlag = 0;
                        HuandaoIn = 0;
                        chuhuan_flag = 0;
                        gpio_set(BEEP, GPIO_LOW);
                        printf("右圆环:%d\r\n",*flag);
                    }
                    break;
            }
     }
}
/*---------------------------------------------------------------
 【函    数】find_huandaorukou_buxian
 【功    能】环岛入口补线----边线数组重赋值
 【参    数】无abs(num_width[guaidian_hang_left]-num_width[guaidian_hang_left+5])-abs(num_width[guaidian_hang_left]-num_width[guaidian_hang_left-1]))>=10
 【返 回 值】HuandaoGet 1:左环岛发现 0：环岛没发现  2:右环岛发现
 【注意事项】2021/11/16
 ----------------------------------------------------------------*/
uint8 find_huandaorukou_buxian()
{


    zuohuxian_flag  =  Find_guaidian(find_left_line,1,1);//判断左边线
    youhuxian_flag  =  Find_guaidian(find_right_line,2,1);//判断右边线

    if(zuohuxian_flag == 1&& youhuxian_flag == 0)
    {
//        if(guaidian_hang_left>=40)
//        {
//            right_k = min2_line(find_right_line,20,2);
//            if(Find_Hudian(find_left_line,0,guaidian_hang_left)&&(right_k >= 0.05 &&right_k<=0.25)
//            &&lose_right_line[guaidian_hang_left]==0&&lose_right_line[guaidian_hang_left-3]==0&&lose_right_line[guaidian_hang_left-5]==0)
//            {
//                return 1;
//            }
//            else
//                return 0;
//        }

    }
//    if(zuohuxian_flag == 0 && youhuxian_flag == 1)
//    {
//        if(guaidian_hang_right>=40)
//        {
//            left_k = min2_line(find_left_line,20,2);
//            if(Find_Hudian(find_right_line,1,guaidian_hang_right)&&(left_k >= -0.3 &&left_k<=-0.05))
//                return 2;
//            else
//                return 0;
//        }
//
//    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】error_num
 【功    能】偏差值计算
 【参    数】N ：看多少行
 【返 回 值】无
 【注意事项】2021/03/17
      理想中线start_w=47
     最下面15行为有效行（根据具体情况判断使用多少行）
  error_h 高度上的误差    error_w  宽度上的误差
 ----------------------------------------------------------------*/
int error_i,error_see;
uint8 error_h=0;
int16 error_w=0;
uint8 error_flag=1;
void error_num(int8 N,uint8 hang)
{
    uint8 valid_num=0;
//    for(int i = start_h;i>start_h -N;i--)
//    {
//        mid_head_line[i]=(left_head_line[i] +right_head_line[i])/2;
//    }

    for(int i=(hang);i>(hang) - N;i--)
    {
        if(error_flag==1)
        {
          error_w += (int16)mid_head_line[i];
        }
        else if(error_flag==2)//左线偏差
        {
          error_w += (int16)left_head_line[i];
        }
        else if(error_flag==3)//右线偏差
        {
         error_w += (int16)right_head_line[i];
        }
        else if(error_flag==4)
        {
          error_w=0;
        }
    }
  //计算偏差

        if(error_flag==1)
        { //中线偏差法
          error_w=(error_w / N)-start_w+5;
        }
        else if(error_flag==2)//左线偏差法
        {
          error_w=(error_w / N)-leftline_ave_point;
        }
        else if(error_flag==3)//右线偏差法
        {
          error_w=(error_w / N)-rightline_ave_point;
        }
        else if(error_flag==4)
        {
          error_w=0;
        }

}
/*---------------------------------------------------------------
 【函    数】find_huxian
 【功    能】判断边线是否存在弧形
 【参    数】传入边线数组
 【参    数】 status： 1：左边线         2：右边线
 【参    数】 numb： 弧度判断阈值
 【返 回 值】1： 有                0：无
 【注意事项】2022/1/12
 ----------------------------------------------------------------*/
uint8   find_huxian(uint8 bianxian[],uint8 status,uint8 numb,uint8 *hudian)
{
    uint16 now_h,n=0,danz_flag = 0,danj_flag = 0;
    uint8 ssss;
    n=0;dz=0;xz=0;danz_flag = 0;danj_flag = 0;

    switch(status)
    {
        case 1:
        for(now_h=59;now_h>39;now_h--)
           {
                if(!lose_left_line[now_h]&&!lose_left_line[now_h-1])//如果没有丢线
                {
                       if(find_left_line[now_h]<find_left_line[now_h-1])//单调增
                       {
                             dz++;
                       }
                       else if(find_left_line[now_h]>find_left_line[now_h-1])
                       {
                             xz++;
                       }
                 }
           }
            break;
        case 2:
               for(now_h=59;now_h>39;now_h--)
              {
                   if(find_right_line[now_h]!=94&&!find_right_line[now_h-1]!=94)//如果没有丢线
                   {
                          if(find_right_line[now_h]<find_right_line[now_h-1])//单调增
                          {
                                dz++;
                          }
                          else if(find_right_line[now_h]>find_right_line[now_h-1])
                          {
                                xz++;
                          }
                    }
              }
               break;
    }
     return 0;
}
/*---------------------------------------------------------------
 【函    数】find_line_dandiao
 【功    能】判断边线是否单调
 【参    数】传入边线数组  flag 0:左 flag 1: 右
 【返 回 值】0：不单调or错误， 1：单调递增， 2：单调递减
 【注意事项】2022/1/13
 ----------------------------------------------------------------*/
uint8 dandiao_num=0;
int find_line_dandiao(uint8 linesuzhu[],uint8 flag)
{
    uint8 dandiao_i;
    dandiao_num=0;
    if(flag==0)
    {
      for(dandiao_i=59;dandiao_i>39;dandiao_i--)
      {
          if(lose_left_line[dandiao_i])
              continue;
          if(find_left_line[dandiao_i-1]<find_left_line[dandiao_i]||find_left_line[dandiao_i-2]<find_left_line[dandiao_i]||find_left_line[dandiao_i-2]<find_left_line[dandiao_i-1])
          {
              dandiao_num++;
          }
          else
          {
              dandiao_num = 0;
              continue;
          }
          if(dandiao_num>dandiaoyuzhi)
          {
              return 1;
          }
      }
      dandiao_num=0;
      for(dandiao_i=59;dandiao_i>39;dandiao_i--)
      {
          if(lose_left_line[dandiao_i])
             continue;
            if(find_left_line[dandiao_i-1]>find_left_line[dandiao_i]||find_left_line[dandiao_i-2]>find_left_line[dandiao_i]||find_left_line[dandiao_i-2]>find_left_line[dandiao_i-1])
            {
                dandiao_num++;
            }
            else
            {
                dandiao_num = 0;
                continue;
            }
            if(dandiao_num>dandiaoyuzhi)
            {
                 return 2;
            }

       }
      return 0;
    }
    else
    {
        for(dandiao_i=59;dandiao_i>39;dandiao_i--)
         {
             if(lose_right_line[dandiao_i])
                 continue;
             if(find_right_line[dandiao_i-1]<=find_right_line[dandiao_i])
             {
                 dandiao_num++;

             }
              else
              {
                  dandiao_num = 0;
                  continue;
              }
             if(dandiao_num>dandiaoyuzhi)
              {
                   return 1;
              }
         }
         dandiao_num=0;
         for(dandiao_i=59;dandiao_i>39;dandiao_i--)
         {
             if(lose_right_line[dandiao_i])
                continue;
               if(find_right_line[dandiao_i-1]>=find_right_line[dandiao_i])
               {
                   dandiao_num++;
                   if(dandiao_num>dandiaoyuzhi)
                   {
                        return 2;
                   }
               }
              else
              {
                  dandiao_num = 0;
                  continue;
              }
          }
         return 0;

    }

}
/*---------------------------------------------------------------
 【函    数】First_Scanf
 【功    能】中点为黑色的首行处理
 【参    数】无
 【返 回 值】无
 【注意事项】255白   0黑   1红
                        首行中点（60，47）
        last_center_point       下行扫线中点
        find_left_line[60];     左线数组
        find_right_line[60];
 ----------------------------------------------------------------*/
void First_Scanf(void)
{
     uint8   now_i;
     uint8   Left_flag=0,Right_flag=0;
  for(now_i=90;now_i>=10;now_i--)
  {
    if(     bin_image[59][now_i-3]==0  &&
            bin_image[59][now_i-3]==0  &&
            bin_image[59][now_i-2]==0  &&  //黑
            bin_image[59][now_i-1]==0  &&  //黑      //255 白       0 黑
            bin_image[59][now_i  ]==255&&  //白
            bin_image[59][now_i+1]==255&&  //白
            bin_image[59][now_i+2]==255&&
            bin_image[59][now_i+3]==255&&Left_flag==0)
    {

        find_left_line[59] = now_i;
        Left_flag          = 1;

    }
    if(     bin_image[59][now_i-4]==255 &&
            bin_image[59][now_i-3]==255 &&
            bin_image[59][now_i-2]==255 && //白白
            bin_image[59][now_i-1]==255 &&
            bin_image[59][now_i  ]==0   && //黑黑黑
            bin_image[59][now_i+1]==0   &&
            bin_image[59][now_i+2]==0   &&
            bin_image[59][now_i+3]==0   &&Right_flag==0)
    {
        find_right_line[59] = now_i;
        Right_flag          = 1;
    }
    if(Left_flag==1&&Right_flag==1)
    {
        break;
    }
    if(Left_flag==0&&Right_flag==0&&now_i==9)
    {
       find_left_line[59]  = 30;
       find_right_line[59] =64;
    }
  }
  if(Left_flag==1&&Right_flag==0)
  {
      if(find_left_line[59]>=55)
          find_right_line[59] = find_left_line[59] + 39;//右边线强行赋值   20瞎填的根据赛道来 这是首行右边丢线的情况
      else
          find_right_line[59] = 94;
  }
  if(Left_flag==0&&Right_flag==1)
  {
      if(find_right_line[59]>=39)
          find_left_line[59] =  find_right_line[59] - 39;//20瞎填的根据赛道来 这是首行左边丢线的情况
      else
          find_left_line[59] = 0;
  }
  if(Left_flag==1&&Right_flag==1)
  {
      if(find_left_line[59]<=47)
      {
          if(find_right_line[59]<find_left_line[59])
              find_right_line[59] = 94;
      }
      else
      {
          find_left_line[59] = 0;
      }
  }
}
/*---------------------------------------------------------------
 【函    数】AddLine
 【功    能】两点之间，斜率补线
 【参    数】翻译字译  flag 0:左边线 1：右边线
 【返 回 值】无
 【注意事项】 一定    end_hang > start_hang
        find_left_line[60];     左线数组                             hang : Y   lie : X
        find_right_line[60];                    Y  = kX + b
 ----------------------------------------------------------------*/// 40 20 59 2
float k,b;
void AddLine(uint8 leftline[],uint8 rightline[],uint8 start_hang,uint8 start_lie,uint8 end_hang,uint8 end_lie,uint8 flag)
{
    int i;

    uint8 y;
    switch(flag)
    {
        case 0:
            k =(float)((float)(end_lie - start_lie) /(float)(end_hang - start_hang));          //的他Y/的他X
            b = (float)(start_lie-(float)start_hang*k);           //b = y - k*x
            for(i = start_hang;i < end_hang;i++)
            {
                y = ((float)(k*i + b));
                leftline[i] = y;
                xianshi[i][y]=2;
            }
            break;
        case 1:
            k =(float)((float)(end_lie - start_lie) /(float)(end_hang - start_hang));          //的他Y/的他X
            b = (float)(start_lie-(float)start_hang*k);
            for(i = start_hang;i < end_hang;i++)
            {
                  y = ((float)(k*i + b));
                  rightline[i] = y;
                  xianshi[i][y]=2;
            }
            break;
    }
}
///*---------------------------------------------------------------
// 【函    数】Huandao_Process_two
// 【功    能】环岛程序方案二
// 【参    数】flag:环岛标志位
// 【返 回 值】无
// 【注意事项】2022/03/17
// ----------------------------------------------------------------*/
//void Huandao_Process_two(uint8 rightline[] ,uint8 leftline[],uint8 *flag)
//{
//    //左环
//    uint8 w_h=0,width=0,lose_line_flag=0,huxing_flag=0,ArcHang=0,ArcLie=0,first_hang=0,first_lie=0,sec_lie=0,sec_hang=0,ru_huan = 1;//宽度行标志
//    uint8 state_flag=0,left_dandiao_flag=0,right_dandiao_flag=0,thr_hang = 0,thr_lie = 0,fou_hang = 0,fou_lie = 0,fiv_hang = 0,fiv_lie = 0;
//       state_flag = *flag;//基本flag == 1 每次进来从 case 1 开始执行
//    while(ru_huan)
//    {
//        switch(state_flag)
//        {  case 1:
//            {
//                if(go_ok_huandao_flag == 1)//已经入环
//                {
//                    state_flag = 4;
//                    break;
//                }
//                if(go_ok_huandao_flag == 0 && go_ok_huandao_flag == 1)//成功入环、出环 准备补通过环的最后一条线
//                {
//                    state_flag = 5;
//                    break;
//                }
//                for(w_h=59;w_h>1;w_h--)
//              {
//                  huxing_flag = find_huxian(leftline,1,10,&ArcHang);
//                  width =  left_width[w_h-1] -  left_width[w_h];  //赛道宽度突变阈值
//                  lose_line_flag = lose_left_line[w_h];    //丢线被置为1
//                  if(width>10&&lose_line_flag==1&&lose_left>100&&huxing_flag==1)    //检测第一个拐点  lose_left>100   100是环的口子丢线数 实际应该更高
//                  {
//                      /*
//                                      *  圆环检测成功   补第一条线
//                       */
//                      first_hang = w_h;            //第一个拐点所在行
//                      first_lie  = leftline[w_h];  //第一个拐点所在列
//                      ArcLie = leftline[ArcHang];  //弧点找到  第二个点
//                      AddLine(find_left_line,find_right_line,ArcHang,ArcLie,first_hang,first_lie,0);//补线
//                      state_flag = 2;
//                  }
//              }
//                break;
//            }
//        case 2:
//           {
//                  if(lose_left_line[59]==1)//丢线为1 丢线说明第一个拐点已经消失 补线到左下角
//                  {
//                      AddLine(find_left_line,find_right_line,ArcHang,ArcLie,59,20,0);  //补线到左下角
//                      state_flag = 3;
//                  }
//                  else
//                  {
//                      ru_huan = 0; //退出环岛 不补第二条线 后面都不执行
//                  }
//
//
//             break;
//           }
//        case 3:
//        {
//                 if(lose_left_line[59]==0)// 左线不丢线  说明走到环岛中间了准备补第二条   直接往右下角补
//                 {
//                     for(w_h = 59;w_h>1;w_h--) //寻找第二个拐点
//                     {
//                         if(lose_left_line[w_h]==0&&lose_left_line[w_h-1]==1)//这一行丢线下一行不丢线判断为第二个拐点
//                         {
//                             sec_hang = w_h-1;
//                             sec_lie  = leftline[w_h];
//                             AddLine(find_left_line,find_right_line,sec_hang,sec_lie,59,60,1); //60乱添的 右下角在的列 补右线
//                             go_ok_huandao_flag = 1; // 入环成功
//                             go_ok_huandao_flag = 0; // 准备出
//                             ru_huan = 0;      //退出环岛 回到正常巡线
//                             break;
//                         }
//                     }
//                 }
//                 else
//                 {
//                     state_flag = 3;//回到 case 2 继续往左下角补线
//                 }
//            break;
//        }
//        case 4:
//        {
//            for(w_h = 59;w_h>1;w_h--)
//            {  // 判断出环口 赛道宽度增加20 （瞎写的） 右边线单调增 左边线单调减 说明到了出环口   不用丢线判断是因为在环上不知道能看到那边的线
//              if( (num_width[w_h]> (num_width[w_h-1]+20) )&&find_line_dandiao(rightline,1) == 1 && find_line_dandiao(leftline,0) == 2)
//              {
//                thr_hang = w_h-1;           //第 3 个点  行
//                thr_lie  = rightline[w_h];  //第 3 个点  列
//                fou_hang = 0; //第四个点直接拉到最顶行
//                fou_lie  = leftline[w_h]; //第四个点的列
//                AddLine(find_left_line,find_right_line,thr_hang,thr_lie,fou_hang,fou_lie,1);//补右线
//                go_ok_huandao_flag = 0; // 入环成功 标志位清零
//                go_ok_huandao_flag = 1; // 出环成功
//                ru_huan = 0;      //退出环岛 回到正常巡线
//                break;
//              }
//            }
//            ru_huan = 0;//没到出环正常巡线
//            break;
//        }
//        case 5:
//        {
//            for(w_h = 59;w_h>1;w_h--)
//            {
//                if(lose_left_line[w_h]==0&&lose_left_line[w_h-1]==1)//这一行丢线下一行不丢线判断为第最后个拐点
//                {
//                    fiv_hang = w_h-1;
//                    fiv_lie  = leftline[w_h];  //最后一个拐点的 列
//                    AddLine(find_left_line,find_right_line,fiv_hang,fiv_lie,59,20,0); //20乱添的 右下角在的列 补左线
//                    ru_huan = 0;      //退出环岛 回到正常巡线
//                    break;
//                }
//            }
//            break;
//        }
//      }
//    }
//
//}
/*---------------------------------------------------------------
 【函    数】bianxian_buxian()
 【功    能】决策边线补线
 【参    数】无
 【返 回 值】无
 【注意事项】2022/03/23
uint8 find_left_line[60];     //左线数组
uint8 left_head_line[60];     //左线决策数组
uint8 find_right_line[60];    //右线数组
uint8 right_head_line[60];    //右线决策数组
uint8  start_h=59;           //起始高（60，47）(60 41)  (60 51)
 ----------------------------------------------------------------*/
void bianxian_buxian()
{
    uint8 hang_i = 0,L_bu_line_flag = 0,R_bu_line_flag = 0,buxian_i = 0;
    for(hang_i=start_h;hang_i>15;hang_i--)  //行   补左决策边线
    {
        if(find_left_line[hang_i]==0)//丢线
        {
          for(buxian_i=hang_i;buxian_i>15;buxian_i--)
          {
              if(left_head_line[buxian_i]!=0)
              {
                  break;
              }
              else
              {
                  left_head_line[buxian_i] = find_left_line[hang_i-1];
              }
          }
        }
    }
//    for(hang_i=start_h;hang_i>15;hang_i--)  //行   补右决策边线
//    {
//        if(find_right_line[hang_i]==0)//丢线
//        {
//          for(buxian_i=hang_i;buxian_i>15;buxian_i--)
//          {
//              if(find_right_line[buxian_i]!=0)
//              {
//                  break;
//              }
//              else
//              {
//                  right_head_line[buxian_i] = find_right_line[hang_i-1];
//              }
//          }
//        }
//    }
}
/*---------------------------------------------------------------
 【函    数】Lcd_ShowLine
 【功    能】边线画点
 【参    数】无
 【返 回 值】无
 【注意事项】2022/03/23
uint8 find_left_line[60];     //左线数组
uint8 left_head_line[60];     //左线决策数组
uint8 find_right_line[60];    //右线数组
uint8 right_head_line[60];    //右线决策数组
uint8  start_h=59;           //起始高（60，47）(60 41)  (60 51)
 ----------------------------------------------------------------*/
void Lcd_ShowLine(uint8 bianxian[],uint16 color)
{
    for(int i = start_h;i>0;i--)
    {
        lcd_drawpoint(bianxian[i]+10,i,color);
    }
}
/*---------------------------------------------------------------
 【函    数】min2_way
 【功    能】最小二乘法 拟合直线
 【参    数】(*data_line)[60]： Y 的数组     N:拟合数据个数
 【返 回 值】无
 【注意事项】2022/03/24
 Y = K X+B   行为X 必须行为X
uint8 find_left_line[60];     //左线数组
uint8 left_head_line[60];     //左线决策数组
uint8 find_right_line[60];    //右线数组
uint8 right_head_line[60];    //右线决策数组
uint8  start_h=59;           //起始高（60，47）(60 41)  (60 51)
 ----------------------------------------------------------------*/

void min2_way(uint8 data_line[],int8 N)
{
    uint8 hang = N;
    uint8 X;
    float K = 0, b = 0;
    for(int i = start_h;i>start_h -N;i--)
    {
        data_line[i]=(left_head_line[i] +right_head_line[i])/2;
        if(right_head_line[i] == 94 && left_head_line[i] == 0)
        {
            hang--;
            if(i!=start_h)
            {
               data_line[i] = data_line[i+1];
            }
        }
    }

    K = min2_line_K(data_line,hang);
    b = min2_line_b(data_line,hang,K);

    for(X = start_h; X > start_h - N; X--)
    {
        data_line[X] =  abs((uint8)((K * X) + b));

    }

}
/*---------------------------------------------------------------
 【函    数】min2_line_K
 【功    能】最小二乘法 拟合直线  求斜率
 【参    数】(*data_line)[60]： Y 的数组     N:拟合数据个数
 【返 回 值】斜率 K
 【注意事项】2022/03/24
 Y = K X+B   行为X 必须行为X
uint8  start_h=59;
 ----------------------------------------------------------------*/
int nXX,nXY;
int num;
float min2_line_K(uint8 data_line[],int N)
{
    int X_X;
    int fenzi,fenmu;
    int  i = 0, X = 0,X_NUM = 0,Y_NUM = 0;
    int data_x = start_h;
    float K = 0;
    nXX = 0;
    for(i = 0; i < N; i++)
    {
        if((data_line[data_x]!=0||data_line[data_x]!=94))
        {
          X       = data_x;  //行
          nXY    += (int)(X * (int)(data_line[data_x]));
          Y_NUM  += (data_line[data_x]);
          nXX    += (X * X);
          X_NUM  += X ;
          num++;
        }
        data_x--;
    }
    X_X = X_NUM * X_NUM;
    nXX = nXX * num;
    nXY = num * nXY;

    fenzi = (nXY - (X_NUM * Y_NUM));
    fenmu = (nXX - X_X);

    if(fenmu!=0)
        K = fenzi*1.0/ fenmu;

    X = 0,X_NUM = 0,Y_NUM = 0,X_X = 0, nXY = 0,nXX = 0;
    fenzi = 0,fenmu = 0;
    num = 0;
    return K;
}
/*---------------------------------------------------------------
 【函    数】min2_line_b
 【功    能】最小二乘法 拟合直线  求截距
 【参    数】(*data_line)[60]： Y 的数组     N:拟合数据个数    K： 斜率
 【返 回 值】斜率 b
 【注意事项】2022/03/24
 ----------------------------------------------------------------*/
int num_b;
float min2_line_b (uint8 data_line[],int8 N,float K)
{
    int   i = 0, X = 0, Y_NUM = 0, X_NUM = 0,X_P = 0,Y_P = 0,data_x = start_h;
    float b = 0;
    for(i = 0; i < N; i++)//行
    {
        if(data_line[data_x]!=0||data_line[data_x]!=94)
        {
            X = data_x;
            X_NUM += X;
            Y_NUM += (sint8)(data_line[data_x]);
            num_b++;
        }
        data_x--;
    }

    Y_P = Y_NUM / num_b;
    X_P = X_NUM / num_b;
    b = Y_P - K * X_P;
    Y_NUM = 0, X_NUM = 0,X_P = 0,Y_P = 0,num_b=0;
    return b;
}
/*---------------------------------------------------------------
 【函    数】Find_guaidian
 【功    能】寻找拐点
 【参    数】uint8 bianxian[]边线数组    flag :  1 左线   2  右线    UD:  1： 下往上扫   0 ：上往下扫
 【返 回 值】 1 有  0无
 【注意事项】2022/03/26
 ----------------------------------------------------------------*/
uint8 Find_guaidian(uint8 bianxian[],uint8 flag,uint8 UD)
{
  if(UD)
  {
    switch(flag)
    {
        case 1:
            for(uint8 i = start_h-2;i>=33;i--)
            {
                if(lose_left_line[i])
                  continue;
                if(!lose_left_line[i+1])
                {
                    if(bianxian[i]-bianxian[i-1]>1&&abs(bianxian[i+1]-bianxian[i])<=6&&bianxian[i]-bianxian[i-2]>1&&abs(bianxian[i+2]-bianxian[i])<=6)
                    {
                        guaidian_hang_left = i;
                        return 1;
                    }
                }
            }
            break;
        case 2:
            for(uint8 i = start_h-2;i>=33;i--)
            {
                if(lose_right_line[i])
                   continue;
                if(!lose_right_line[i+1])
                {
                   if(bianxian[i-1]-bianxian[i]>1&&abs(bianxian[i+1]-bianxian[i])<=6&&bianxian[i-2]-bianxian[i]>1&&abs(bianxian[i+2]-bianxian[i])<=6)
                   {
                       guaidian_hang_right = i;
                       return 1;
                   }
                }
            }
            break;
    }
    return 0;
  }
  else
  {
      switch(flag)
          {
              case 1:
                  for(uint8 i = 15;i>=(start_h-2);i++)
                  {
                      if(lose_left_line[i])
                        continue;
                      if(!lose_left_line[i+1])
                      {
                          if((bianxian[i+1]-bianxian[i])>=0&&
                             (bianxian[i+2]-bianxian[i])>=0&&
                             (bianxian[i]-bianxian[i-1])>10&&
                             (bianxian[i]-bianxian[i-2])>10
                            )
                          {
                              guaidian_hang_left = i;
                              return 1;
                          }
                      }
                  }
                  break;
              case 2:
                  for(uint8 i = 15;i>=(start_h-2);i++)
                  {
                      if(lose_right_line[i])
                         continue;
                      if(!lose_right_line[i+1])
                      {
                         if(bianxian[i-1]-bianxian[i]>1&&
                                 abs(bianxian[i+1]-bianxian[i])<=2&&bianxian[i-2]-bianxian[i]>1&&abs(bianxian[i+2]-bianxian[i])<=2)
                         {
                             guaidian_hang_right = i;
                             return 1;
                         }
                      }
                  }
                  break;
          }
          return 0;
        }
}
/*---------------------------------------------------------------
 【函    数】Find_upguaidain
 【功    能】寻找拐点
 【参    数】uint8 bianxian[]边线数组  flag 0: 左边线 1：右边线
 【返 回 值】行数
 【注意事项】2022/04/01       寻找边线的上拐点
 uint8 Up_left_guaidian_hang,Up_left_guaidian_lie;   //上拐点行上拐点列
 ----------------------------------------------------------------*/
uint8 Find_upguaidian(uint8 bianxian[],uint8 flag,uint8 starthang)
{
//   switch(flag)
//   {
//       case 0:
//          for(uint8 i = starthang;i>=10;i--)
//          {
//              if(lose_left_line[i])
//                continue;
//              if((lose_left_line[i+1]||lose_left_line[i+2]||lose_left_line[i+3]||lose_left_line[i+4]||lose_left_line[i+5]||lose_left_line[i+6])
//                       &&abs(bianxian[i-1]-bianxian[i])<=2)
//              {
//                  if(abs(bianxian[i]-bianxian[starthang])<=5&&(bianxian[i]>bianxian[starthang]))
//                  {
//                      Up_left_guaidian_hang = i;
//                      Up_left_guaidian_lie = bianxian[i];
//                      return 1;
//                  }
//                  else
//                  {
//                      continue;
//                  }
//
//              }
//          }
//           break;
//       case 1:
//          for(uint8 i = starthang;i>=10;i--)
//          {
//             if(lose_right_line[i])
//               continue;
//             if((lose_right_line[i+1]||lose_right_line[i+2]||lose_right_line[i+3]||lose_right_line[i+4]||lose_right_line[i+5]||lose_right_line[i+6])
//                      &&abs(bianxian[i-1]-bianxian[i])<=2)
//             {
//                 if(abs(bianxian[i]-bianxian[starthang])<=5&&(bianxian[i]<bianxian[starthang]))
//                 {
//                     Up_right_guaidian_hang = i;
//                     Up_right_guaidian_lie = bianxian[i];
//                     return 1;
//                 }
//                 else
//                 {
//                     continue;
//                 }
//
//             }
//          }
//           break;
//   }


    for(uint8 i = start_h-4;i>10;i--)
    {
        if((abs(num_width[i]-num_width[i+3])-abs(num_width[i]-num_width[i-1]))>=15)
        {
            Up_guaidian_hang = i;
            Up_right_guaidian_lie = find_right_line[Up_guaidian_hang-6];
            Up_left_guaidian_lie  = find_left_line[Up_guaidian_hang-6];
            return 1;
        }
    }

   return 0;
}
/*---------------------------------------------------------------
 【函    数】min2_line
 【功    能】边线拟合
 【参    数】uint8 data_line[]边线数组 N 拟合的行数  flag 1:改变数组 2：不改变数组
 【返 回 值】拟合斜率
 【注意事项】2022/03/25
 ----------------------------------------------------------------*/
float min2_line(uint8 data_line[],int8 N,uint8 flag)
{
    float K;
    float B;
    uint8 X;
    K = min2_line_K(data_line,N);


   if(flag==1)
   {
      B = min2_line_b(data_line,N,K);
      for(X = start_h; X > start_h - N; X--)
      {
          data_line[X] =  abs((uint8)((K * X) + b));

      }
      for(uint8 i =start_h;i>30;i--)
      {
          data_line[i] = abs((uint8)(K * i +B));
          //bin_image[i][abs((uint8)(K * i +B))] = 2;
      }

   }
    return K;

}
/*---------------------------------------------------------------
 【函    数】min2_line
 【功    能】边线拟合
 【参    数】uint8 data_line[]边线数组 N 拟合的行数  flag 1:改变数组 2：不改变数组
 【返 回 值】1:单调递增 2:单调递减 0：错误
 【注意事项】2022/03/25
 ----------------------------------------------------------------*/
uint8 min2_dandiao(uint8 lineshuzu[],uint8 nihe_num)
{
    float dandiao_k;
    dandiao_k = min2_line(lineshuzu,nihe_num,2);        //不改变数组的找斜率
//    sprintf(buf2,"%.2f",dandiao_k);
//    lcd_showstr(50,5,buf2);
    if(dandiao_k<0&&dandiao_k>=-1)
        return 1;
    else if(dandiao_k>0&&dandiao_k<=1)
        return 2;
    else
        return 0;
}
;
/*---------------------------------------------------------------
 【函    数】Find_ShangBianxianguaidian
 【功    能】寻找上下边线
 【参    数】uint8  (*image)[94] 传入数组
 【返 回 值】1:单调递增 2:单调递减 0：错误
 【注意事项】
 //uint8 find_up_line[60];
//uint8 find_down_line[60]
 ----------------------------------------------------------------*/
void Find_ShangBianxianguaidian(uint8  (*image)[94])
{
    find_up_line[47] = 0;
    find_down_line[47] = 59;
    /*先找到中间列的上下边线*/
    for(uint8 i = 30; i >= 0; i--)
      {
          if(image[i][start_w] == 0&&image[i+1][start_w] == 255&&image[i-1][start_w] == 0)
          {
              find_up_line[47] = i;
              break;
          }
      }

      for(uint8 i = 30; i < 60; i++)
      {
          if(image[i][start_w] == 0&&image[i+1][start_w] == 0&&image[i-1][start_w]==255)
          {
              find_down_line[47] = i;
              break;
          }
      }
    for(uint8 lie = start_w-1;lie>2;lie--)      //向左找上下边线
    {//0：黑  1：白
        find_up_line[lie] = 0;
        find_down_line[lie]= 59;        //防止没有值，溢出
        for(int hang = find_up_line[lie+1]+5; hang >= 0;hang--)//寻找上边线 (lie+1)是为了沿着边线去找    +10是为了向下走10行再往上去找
        {
            if(image[hang][lie] == 0&&image[hang+2][lie] == 255&&image[hang+1][lie] == 255&&image[hang-1][lie]==0&&image[hang-2][lie]==0)    //上边线
            {
                find_up_line[lie] = hang;
                xianshi[hang][lie] = 5;
                xianshi[hang-1][lie] = 5;
                break;
            }

        }
        for(int hang = find_down_line[lie+1]-5;hang >=59 ; hang++)     //下边线
        {
            if(image[hang][lie] == 0&&image[hang-2][lie] == 255&&image[hang-1][lie] == 255&&image[hang-2][lie] == 255)
            {
                find_down_line[lie] = hang;
                xianshi[hang][lie] = 6;
                xianshi[hang+1][lie] = 6;
                break;
            }

        }
    }
    for(int lie = start_w+1;lie<92;lie++)
    {
        find_up_line[lie] = 0;
        find_down_line[lie]= 59;        //防止没有值，溢出
        for(int hang = find_up_line[lie-1]+5;hang>=0;hang--)           //上边线
        {
            if(image[hang][lie] == 0&&image[hang+2][lie] == 255&&image[hang+1][lie] == 255&&image[hang-1][lie]==0&&image[hang-2][lie]==0)
           {
               find_up_line[lie] = hang;
               xianshi[hang][lie] = 5;
               xianshi[hang-1][lie] = 5;
               break;
           }
        }
        for(int hang = find_down_line[lie-1]-5;hang >=59 ; hang++)     //下边线
       {
           if(image[hang][lie] == 0&&image[hang-2][lie] == 255&&image[hang-1][lie] == 255&&image[hang-2][lie] == 255)
           {
               find_down_line[lie] = hang;
               xianshi[hang][lie] = 6;
               xianshi[hang+1][lie] = 6;
               break;
           }

       }
    }
}
/*---------------------------------------------------------------
 【函    数】find_UpGuaidian
 【功    能】寻找上边线拐点
 【参    数】shangbianxian 上边线数组
 【返 回 值】最大行所在列
 【注意事项】2022/4/1
 //uint8 find_up_line[60];
//uint8 find_down_line[60]  寻找上边线拐点
 ----------------------------------------------------------------*/
uint8 find_UpGuaidian(uint8 shangbianxian[])
{
    uint8 MaxLie;
    uint8 HangMax;
    for(int lie = 1;lie<92;lie++)
    {
        if(shangbianxian[lie] - shangbianxian[lie-1]>=3)
            continue;
        if(shangbianxian[lie]>HangMax)
        {
            MaxLie = lie;
            HangMax = shangbianxian[lie];
        }
    }
    return MaxLie;
}
/*---------------------------------------------------------------
 【函    数】First_Scanf_Sobel
 【功    能】Sobel版首行扫描
 【参    数】无
 【返 回 值】无
 【注意事项】2022/4/1
 //uint8 find_up_line[60];
 //uint8 find_down_line[60]
 ----------------------------------------------------------------*/
void First_Scanf_Sobel(void)
{
     uint8   now_i;
     uint8   y_c=60,x_c=94,Left_flag=0,Right_flag=0;
  for(now_i=0;now_i<91;now_i++)
  {
    if(bin_image[59][now_i-2]==255 &&
       bin_image[59][now_i  ]==0   &&
       bin_image[59][now_i+2]==255)
        {
         find_right_line[59] = now_i;
         Right_flag          = 1;
        }
    if(Left_flag==1&&Right_flag==1)
        {
        Left_flag         = 0;
        Right_flag        = 0;
        break;
        }
    if(Left_flag==1&&Right_flag==0&&now_i==90)
        {
        find_right_line[59] = find_left_line[59] + 35;//右边线强行赋值   20瞎填的根据赛道来 这是首行右边丢线的情况
        Left_flag         =  0;
        Right_flag        =  0;
        }
    if(Left_flag==0&&Right_flag==1&&now_i==90)
        {
        find_left_line[59] =  find_right_line[59] - 35;//20瞎填的根据赛道来 这是首行左边丢线的情况
        Left_flag         =  0;
        Right_flag        =  0;
        }
    if(Left_flag==0&&Right_flag==0&&now_i==90)
        {
         Left_flag     = 0;
         Right_flag    = 0;
        }
  }
}
/*---------------------------------------------------------------
 【函    数】yuansu_get()
 【功    能】元素判断
 【参    数】无
 【返 回 值】无
 【注意事项】2022/4/1
 //uint8 find_up_line[60];
 //uint8 find_down_line[60]
 ----------------------------------------------------------------*/
uint8 PRoadIn;
uint8 PRoadGet;
uint8 PRoadFlag;
uint8 sanchain = 0;
uint8 PandHuandaoGet=0;
uint8 PandHuandaoFlag = 0;
uint8 left_stop_flag = 0; //左库识别标志
uint8 right_stop_flag = 0; //右库识别标志
uint8  garage_flag_num  = 0;       //车库计数
uint8  GarageGet  = 0;       //车库标志位
uint8  GarageIn  = 0;       //车库标志位
uint8  garage_detch_flag = 0;
uint8  ramp_flag = 0;   //坡道标志位
extern uint8 P_UpSpeed_Flag;
extern uint8 LongRoad_UpSpeed_Flag;
void yuansu_get(void)
{

    //车库元素判断
     if(sanchaflag==0&&HuandaoFlag==0&&PRoadFlag==0&&ShiziFlag==0&&PandHuandaoFlag==0&&GarageGet == 0&&ramp_flag==0)
     {
        ramp_judge();
     }
     if(sanchaflag==0&&HuandaoFlag==0&&PRoadFlag==0&&ShiziFlag==0&&PandHuandaoFlag==0&&GarageGet==0&&ramp_flag==0)
     {
         if((Find_guaidian(find_left_line,1,1)||Find_guaidian(find_right_line,2,1))&&GarageGet==0)
         {
             //printf("拐点找到\r\n");
             garage_detch_flag = 1;
         }
         if(garage_detch_flag)
         {
             if(garage_judge()==1)
             {
                 if(guaidian_hang_left >= 33)
                 {
                     if(find_garage_line_num(find_left_line,1))         //左库
                     {
                        GarageGet = 1;        //检测到车库
                        GarageIn = 1;
                        garge_scanf_line(2);
                        error_flag  = 3;      //方向打直
                        printf("左车库识别\r\n");
                     }
                 }
                 if(guaidian_hang_right >= 33)
                 {
                     if(find_garage_line_num(find_right_line,2))          //右库
                     {
                         GarageGet = 2;        //检测到车库
                         GarageIn = 2;
                         garge_scanf_line(1);
                         error_flag  = 2;        //方向打直
                         printf("右车库识别\r\n");
                     }
                 }
             }
             else
                 garage_detch_flag = 0;
         }
     }
   //环岛   P    元素判断
    if(sanchaflag==0&&HuandaoFlag==0&&PRoadFlag==0&&ShiziFlag==0&&GarageGet == 0&&ramp_flag==0)
    {
        if(PandHuandaoGet==0)
        {
            //printf("PH识别\r\n");
            PandHuandaoGet = PH_general_process(); //   P  环岛 通用初步识别
        }

        if((PandHuandaoGet==1||PandHuandaoFlag==1))
        {
//            lefttargetspeed = 280;
//            righttargetspeed = 280;
            PandHuandaoFlag = 1;
            PH_shibie(PandHuandaoFlag);
        }
        if((PandHuandaoGet==2||PandHuandaoFlag==2))
        {
//            lefttargetspeed = 280;
//            righttargetspeed = 280;
            PandHuandaoFlag = 2;
            PH_shibie(PandHuandaoFlag);
        }
        //环岛处理
        switch(HuandaoGet)
        {
            case 1:
                HuandaoIn = 1;//左环岛
                HuandaoFlag = 1;
                break;
            case 2:
                HuandaoIn = 2;//右环岛
                HuandaoFlag = 1;
        }
        //P路口处理
        switch(PRoadGet)
        {
           case 1:
              PRoadFlag = 1;
              PRoadIn = 1;
              break;       //右缺失P
           case 2:
              PRoadFlag = 1;
              PRoadIn = 2;
              break;       //左缺失P
        }

    }
    //三叉元素判断
    if(sanchaflag==0&&HuandaoFlag==0&&PRoadFlag==0&&ShiziFlag==0&&PandHuandaoFlag==0&&GarageGet == 0&&ramp_flag==0)
    {
      //  printf("三叉第一重判断\r\n");
        if(Find_sanchaguaidian(find_left_line,1)&&Find_sanchaguaidian(find_right_line,2)&&road_width_change(0)&&road_width_change(1))
        {
         //  printf("三叉第三重判断\r\n");
            if(abs((sancha_guaidian_hang_right-sancha_guaidian_hang_left))<=12&&out_circle_judge(10) == 0)
            {
                lefttargetspeed = 270;
                righttargetspeed = 270;
                sanchaflag = 1;
                sanchain = 1;       //往左走
                printf("三叉识别\r\n");
            }
        }
    }




        //坡道处理
        if(ramp_flag)
        {
            ramp_process(&ramp_flag);
        }
        //车库处理
        if(GarageGet)
        {
            garage_process();
        }
        //环岛实际处理
        if(HuandaoGet)
        {
            Huandao_Process(&HuandaoFlag);
        }
        //P元素实际处理
        if(PRoadGet)
        {
            P_RoadProcess(&PRoadFlag);
        }
        //三叉元素处理
        if(sanchaflag)
        {
           sancha_precess(&sanchaflag);
        }
   //三叉运行结束



//    //十字元素判断
//          if(sanchaflag==0&&HuandaoFlag==0&&PRoadFlag==0&&ShiziFlag==0&&PandHuandaoFlag==0&&GarageGet == 0&&ramp_flag==0)
//          {
//              shiziGet = find_shizi_buxian();
//              if(shiziGet)
//              {
//                  ShiziIn = 1;
//                  ShiziFlag = 1;
//              }
//          }
//          if(shiziGet)
//          {
//              shizi_process(&ShiziFlag);
//          }
//    //十字元素判断
}

/*---------------------------------------------------------------
 【函    数】TFT_Shows()
 【功    能】数据显示
 【参    数】无
 【返 回 值】无
 【注意事项】2022/4/1
 ----------------------------------------------------------------*/
float volvol;
extern int16 LeftSpeed,RightSpeed;
extern short yuzhi;
extern int16 righttargetspeed,lefttargetspeed;
extern struct PID pidleft,pidright;
extern float ServoKp,ServoKd;      //舵机Kd微分系数
extern uint8 rdandiao,ldandiao;
uint8 left_shiziflag;
uint8 right_shiziflag;
extern uint8 huandao_detect_flag;
extern float angle;
void TFT_Shows(void)
{
//    volvol = (float)((vol/4096.0)*9.9);
//
    lcd_displayimage(xianshi[0],94,60);//显示二值图像

//    sprintf(buf2,"Y:%04d",icm_gyro_y);
//    lcd_showstr(0,9,buf2);
//    sprintf(buf2,"X:%04d",icm_gyro_x);
//    lcd_showstr(0,7,buf2);

    sprintf(buf2,"err:%2d",error_w);
    lcd_showstr(100,0,buf2);
    sprintf(buf2,"EF:%2d",error_flag);
    lcd_showstr(100,1,buf2);

//    sprintf(buf2,"Lg:%d",LeftRoadFlag);
//    lcd_showstr(100,2,buf2);
//
//    sprintf(buf2,"Rg:%d",RightRoadFlag);
//    lcd_showstr(100,3,buf2);



//
//    sprintf(buf2,"Gar:%2d",GarageGet);
//    lcd_showstr(0,6,buf2);
//
//    sprintf(buf2,"SanF:%2d",sanchaflag);
//    lcd_showstr(0,7,buf2);
//
//    sprintf(buf2,"PHGet:%2d",PandHuandaoGet);
//    lcd_showstr(0,5,buf2);
//
//    sprintf(buf2,"HuanF:%2d",HuandaoFlag);
//    lcd_showstr(0,8,buf2);
//
//    sprintf(buf2,"LSpeed:%3d",lefttargetspeed);
//    lcd_showstr(0,9,buf2);

//    sprintf(buf2,"Huandao:%d",HuandaoFlag);
//    lcd_showstr(0,8,buf2);
//
//    sprintf(buf2,"HuandaoIn:%2d",HuandaoIn);
//    lcd_showstr(0,9,buf2);
//
//    sprintf(buf2,"huhang:%3d",hudian_hang);
//    lcd_showstr(0,8,buf2);
//
//    sprintf(buf2,"hulie:%3d",hudian_lie);
//    lcd_showstr(0,9,buf2);
////

//    Find_sanchaguaidian(find_left_line,1);

//    sprintf(buf2,"kp:%.2f",pidright.kp);
//    lcd_showstr(0,0,buf2);
//
//    sprintf(buf2,"kd:%.2f",pidright.kd);
//    lcd_showstr(0,1,buf2);
//
//    sprintf(buf2,"ki:%.2f",pidright.ki);
//    lcd_showstr(0,2,buf2);
//    sprintf(buf2,"numhei:%d",garage_judge());
//    lcd_showstr(0,6,buf2);
//
//    sprintf(buf2,"dz:%2d",dz);
//    lcd_showstr(100,7,buf2);




//    sprintf(buf2,"HDF:%2d",huandao_detect_flag);
//    lcd_showstr(0,8,buf2);
//    sprintf(buf2,"dz:%2d",dz);
//    lcd_showstr(0,7,buf2);
//
//    sprintf(buf2,"guailie:%3d",guaidian_hang_right);
//    lcd_showstr(60,8,buf2);
//
//    sprintf(buf2,"guaihang:%3d",guaidian_lie_right);
//    lcd_showstr(60,9,buf2);

//    sprintf(buf2,"vol:%.2fV",volvol);
//    lcd_showstr(100,4,buf2);
//
//    sprintf(buf2,"out:%d",Out_Garage_flag);
//    lcd_showstr(100,5,buf2);
//
//    sprintf(buf2,"Game:%d",Game_Start);
//    lcd_showstr(100,6,buf2);
//
//    sprintf(buf2,"RS:%3d",right_stop_flag);
//    lcd_showstr(100,9,buf2);
////
//    sprintf(buf2,"LS:%3d",left_stop_flag);
//    lcd_showstr(100,8,buf2);
////
//    sprintf(buf2,"sancha:%d",sanchaflag);
//    lcd_showstr(95,4,buf2);
//
    sprintf(buf2,"LK:%.2f",left_k);
    lcd_showstr(0,9,buf2);

    sprintf(buf2,"RK:%.2f",right_k);
    lcd_showstr(0,8,buf2);

    sprintf(buf2,"Ls:%d",leftspeed);
    lcd_showstr(100,4,buf2);

    sprintf(buf2,"Rs:%d",rightspeed);
    lcd_showstr(100,5,buf2);

    sprintf(buf2,"Kp:%.2f",ServoKp);
    lcd_showstr(90,6,buf2);

    sprintf(buf2,"Kd:%.2f",ServoKd);
    lcd_showstr(90,7,buf2);

    sprintf(buf2,"f:%d",LongRoad_UpSpeed_Flag);
    lcd_showstr(90,8,buf2);
//    sprintf(buf2,"RP:%d",pidright.actual_speed);
//    lcd_showstr(95,6,buf2);
//    sprintf(buf2,"lhang:%d",sancha_guaidian_hang_left);
//    lcd_showstr(50,5,buf2);
//
//    sprintf(buf2,"rhang:%d",sancha_guaidian_hang_right);
//    lcd_showstr(50,6,buf2);
//

//    sprintf(buf2,"PRoadIn:%d",PRoadIn);
//    lcd_showstr(80,7,buf2);

//    sprintf(buf2,"yuzhi:%.2f",BlackThres);
//    lcd_showstr(40,6,buf2);
}
/*---------------------------------------------------------------
 【函    数】Find_Hudian()
 【功    能】寻找弧点
 【参    数】无
 【返 回 值】无
 【注意事项】2022/4/1
 ----------------------------------------------------------------*/

uint8 Find_Hudian(uint8 bianxian[],uint8 flag,uint8 startpoint)
{
    uint8 xzz;
    uint8 dzz;
    switch(flag)
    {
        case 0:
            for(int i =startpoint;i>10;i--)
            {
                if(lose_left_line[i])
                    continue;
                if(bianxian[i-1]>bianxian[i])
                    xzz++;
                if(bianxian[i-1]<bianxian[i])
                    dzz++;
                if(((bianxian[i]-bianxian[i-3])>0&&(bianxian[i]-bianxian[i+3])>=0)&&(xzz>=6))
                {
                    if(bianxian[i]-bianxian[startpoint]>0)
                    {
                        hudian_hang = i;
                        hudian_lie = bianxian[i];
                        return 1;
                    }

                }
            }
            break;
        case 1:
            for(int i =startpoint;i>10;i--)
            {
                if(lose_right_line[i])
                    continue;
                if(bianxian[i-1]>bianxian[i])
                    xzz++;
                if(bianxian[i-1]<bianxian[i])
                    dzz++;
                if(((bianxian[i-3]-bianxian[i])>0&&(bianxian[i+3]-bianxian[i])>=0)&&(xzz>=6||dzz>=5))
                {
                    hudian_hang = i;
                    hudian_lie = bianxian[i];
                    return 1;
                }
            }
            break;
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】P_RoadProcess(uint8 *flag)
 【功    能】P路口运行程序
 【参    数】P路口标志位
 【返 回 值】无
 【注意事项】2022/4/3
 ----------------------------------------------------------------*/
void P_RoadProcess(uint8 *flag)
{
    static uint8 buxianflag;
   static uint8 PGuaiFlag = 0;
   xz = 0;
   dz = 0;
   if(PRoadIn == 1)     //右P
   {
//     ServoKp= 3.5;      //舵机Kp比例系数
//     ServoKd= 6;      //舵机Kd微分系数

     switch(*flag)
     {
         case 1:
         {

               find_huxian(0,1,0,0);
//               sprintf(buf2,"dz:%d",dz);
//               lcd_showstr(100,9,buf2);
                gpio_set(BEEP, GPIO_HIGH);
               if(PGuaiFlag==0)
               {
                  gpio_set(BEEP, GPIO_HIGH);
                  if(dz>=15)          //当递减数量在减少
                  {
                      PGuaiFlag = 1;
                      gpio_set(BEEP, GPIO_LOW);
                      *flag = 2;
                      printf("右P:%d\r\n",*flag);
                  }
               }
            break;
          }
          case 2:
          {
              find_huxian(0,1,0,0);
//              sprintf(buf2,"dz:%d",dz);
//              lcd_showstr(100,9,buf2);
              if(dz<=10&&PGuaiFlag&&buxianflag==0)
              {
                  gpio_set(BEEP, GPIO_HIGH);
                  if(Find_guaidian(find_left_line,1,1))        //寻找拐点
                  {
                      guaidian_left_lie = find_left_line[guaidian_hang_left];      //拐点列
                      if(guaidian_hang_left>=30)         //当拐点过于接近时，补线
                      {

                            buxianflag = 1;
                            printf("我补线啦\r\n");
                            AddLine(left_head_line,right_head_line,10,30,59,30,1);     //补左线
                            PGuaiFlag = 0;
                      }
                  }
              }
              if(buxianflag)
              {
                    lefttargetspeed = 275;
                    righttargetspeed = 275;
                    AddLine(left_head_line,right_head_line,30,35,59,35,1);     //补左线&&find_left_line[59]>=15&&find_left_line[58]>=15&&find_left_line[57]>=15
                    if(out_circle_judge(25)==1)
                    {
                       *flag = 0;
                       printf("右P结束\r\n",*flag);
                       buxianflag = 0;
                       gpio_set(BEEP, GPIO_LOW);
                       PRoadGet = 0;
                       PGuaiFlag = 0;
                       PRoadIn = 0;
                       PandHuandaoGet = 0;
                       PandHuandaoFlag = 0;
                    }
              }
            break;
          }
       }
   }
   if(PRoadIn == 2)         //左P
   {
       //printf("error_w:%d\r\n",error_w);
       switch(*flag)
       {
           case 1:
           {
//               ServoKp= 3.5;      //舵机Kp比例系数
//               ServoKd= 6;      //舵机Kd微分系数
               find_huxian(0,2,0,0);
             //  printf("xz:%d\r\n",xz);
               gpio_set(BEEP, GPIO_HIGH);
               if(PGuaiFlag==0)
               {
                   if(xz>=16)          //当递减数量在减少
                   {
                       PGuaiFlag = 1;
                       gpio_set(BEEP, GPIO_LOW);
                       *flag = 2;
                       printf("左P:%d\r\n",*flag);
                   }
               }
               break;
           }
           case 2:
           {
               P_UpSpeed_Flag = 1;
               find_huxian(0,2,0,0);
              // printf("xz:%d\r\n",xz);
               if(xz<=7&&PGuaiFlag&&buxianflag==0)
               {
                   gpio_set(BEEP, GPIO_HIGH);
                   if(Find_guaidian(find_right_line,2,1))        //寻找拐点
                   {
                       guaidian_right_lie = find_right_line[guaidian_hang_right];      //拐点列
                       if(guaidian_hang_right>=43)         //当拐点过于接近时，补线
                       {
                           buxianflag = 1;
                           AddLine(left_head_line,right_head_line,10,45,59,45,0);     //补左线
                           printf("我补线啦\r\n");
//                           lefttargetspeed = 0;
//                           righttargetspeed = 0;
                           PGuaiFlag = 0;
                       }
                   }
               }
               if(buxianflag)
               {
//                     lefttargetspeed = 240;
//                     righttargetspeed = 240;
                       AddLine(left_head_line,right_head_line,39,65,59,65,0);     //补左线

//                       left_k = min2_line(find_left_line,10,2);        //不改变数组的找斜率
//                       if(((left_k >= -0.5 &&left_k<=0)||(left_k<=0.35&&left_k>=0))&&num_width[58]<=45&&num_width[57]<=45&&num_width[56]<=45)&&find_right_line[59]<=65&&find_right_line[58]<=65&&find_right_line[57]<=65
                       if(out_circle_judge(28)==1)
                       {
//                           ServoKp= 3.5;      //舵机Kp比例系数
//                           ServoKd= 2;      //舵机Kd微分系数

                           buxianflag = 0;
                           *flag = 0;
                           printf("左P结束\r\n",*flag);
                           gpio_set(BEEP, GPIO_LOW);
                           PRoadGet = 0;
                           PGuaiFlag = 0;
                           PRoadIn = 0;
                           PandHuandaoGet = 0;
                           PandHuandaoFlag = 0;
                           P_UpSpeed_Flag = 0;
//                           Game_Start = 0;
                       }
               }
               break;
           }
       }
   }

}
/*---------------------------------------------------------------
 【函    数】Find_PRoad(void)
 【功    能】寻找P路口
 【参    数】无
 【返 回 值】无
 【注意事项】2022/4/3
 ----------------------------------------------------------------*/
uint8 Find_PRoad(void)
{
    uint8 zuoxianflag,youxianflag;

    zuoxianflag = Find_guaidian(find_left_line,1,1);
    youxianflag = Find_guaidian(find_right_line,2,1);

    if(zuoxianflag == 1&&youxianflag == 0)      //左线有拐点
    {
        if(guaidian_hang_left>=40)             //如果右线拐点行大于了40
        {
            if(Find_upguaidian(find_left_line,0,guaidian_hang_left))
            {
                return 1;
            }
        }
    }
    if(zuoxianflag == 0&&youxianflag == 1)      //右线有拐点
    {
        if(guaidian_hang_right>=40)             //如果右线拐点行大于了40
        {
            if(Find_upguaidian(find_right_line,1,guaidian_hang_right))
            {
                return 2;
            }
        }
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】sancha_precess()
 【功    能】三叉判断
 【参    数】mode  三叉标志位
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void sancha_precess(uint8 *mode)
{
    static uint8 guai_flag;
    static uint8 chu_flag;
    if(sanchain == 1)       //走左边，补右线
    {
        switch(*mode)
        {
            case 1:
                if(Find_sanchaguaidian(find_left_line,1)&&Find_sanchaguaidian(find_right_line,2))
                {
                    if(guai_flag==0)
                    {
                        if(sancha_guaidian_hang_left>=45)
                        {
                            guai_flag = 1;
                        }
                    }
                }
                if(guai_flag)
                {
                    AddLine(left_head_line,right_head_line,20,20,59,find_left_line[58]+27,1);
                    right_k =min2_line(find_right_line,10,2);
                    if(!Find_sanchaguaidian(find_right_line,2))
                    {
                        if(right_k<1&&right_k>0)
                        {
                           *mode = 2;
                           printf("三叉:%d\r\n",*mode);
                        }
                    }

                }
                gpio_set(BEEP, GPIO_HIGH);
                break;
            case 2:
//                lefttargetspeed = 250;
//                righttargetspeed = 250;
                gpio_set(BEEP, GPIO_HIGH);
                if(Find_sanchaguaidian(find_left_line,1)&&Find_sanchaguaidian(find_right_line,2)&&lukuan_flag == 0)
                {
                    printf("find guaidian\r\n");
                    if(abs((sancha_guaidian_hang_right-sancha_guaidian_hang_left))<=12)
                    {
                        lukuan_flag = 1;
                        printf("true !\r\n");
                    }
                }
                if(lukuan_flag)
                {
                    AddLine(left_head_line,right_head_line,30,30,59,find_left_line[58]+27,1);

                    if(!Find_sanchaguaidian(find_left_line,1)&&!Find_sanchaguaidian(find_right_line,2))
                        chu_flag = 1;

                    if(chu_flag)
                    {
                        if(out_circle_judge(30)==1&&find_right_line[59]<=70&&find_right_line[56]<=70&&find_right_line[54]<=70)
                        {
                            gpio_set(BEEP, GPIO_LOW);
//                            lefttargetspeed  = 0;
//                            righttargetspeed = 0;
//                            leftspeed  = 0;
//                            rightspeed = 0;
//                            Game_Start = 0;
                            *mode = 0;
                            chu_flag = 0;
                            sanchain = 0;
                            guai_flag = 0;
                            lukuan_flag = 0;
                            printf("三叉结束\r\n");
                        }
                    }

                }
                break;
        }
    }
}
/*---------------------------------------------------------------
 【函    数】sancha_get()
 【功    能】三叉判断
 【参    数】无
 【返 回 值】 1：有三叉  0： 无三叉
 【注意事项】2022/04/04
 //uint8 find_up_line[60];
 //uint8 find_down_line[60]
 ----------------------------------------------------------------*/
uint8 sancha_get()
{

      for(int i = 59;i>10;i--)
      {
          if(((num_width[i-5] - num_width[i])>=8))
           {
              lukuan_flag = 1;
           }

          if(((find_left_line[i] - find_left_line[i-5])>5)&&((find_right_line[i-5] - find_right_line[i]))>=5)
          {

              line_flag = 1;
          }
      }
      for(int i = 59;i>1;i--)
      {
           if( bin_image[i][(find_right_line[59]+find_left_line[59])/2] == 0)
           {
               dingdian_flag = 1;
           }
      }

        if(lukuan_flag == 1&&dingdian_flag == 1&&line_flag ==1)
        {
            return 1;
        }

        return 0;
}
/*---------------------------------------------------------------
 【函    数】Find_sanchaguaidian()
 【功    能】寻找三叉拐点
 【参    数】无
 【返 回 值】 1：有拐点  0： 无拐点
 【注意事项】2022/04/04
 ----------------------------------------------------------------*/
uint8 Find_sanchaguaidian(uint8 bianxian[],uint8 flag)
{
    switch(flag)
    {
        case 1:
            for(uint8 i =56;i>40;i--)
            {
                if(lose_left_line[i])
                    continue;
                if(bianxian[i]-bianxian[i-10]>=5&&bianxian[i]-bianxian[i-5]>=3&&bianxian[i]-bianxian[i+3]>=0&&bianxian[i]-bianxian[i+2]>=0&&bianxian[i]-bianxian[i+1]>=0)
                {
                    sancha_guaidian_hang_left = i;
                    sancha_guaidian_lie_left = bianxian[i];
                 //   printf("三叉左拐点\r\n");
                    return 1;
                }
            }
            break;
         case 2:
             for(uint8 i =56;i>40;i--)
             {
                 if(lose_right_line[i])
                     continue;
                 if(bianxian[i-10]-bianxian[i]>=5&&bianxian[i-5]-bianxian[i]>=3&&bianxian[i+3]-bianxian[i]>=0&&bianxian[i+2]-bianxian[i]>=0&&bianxian[i+1]-bianxian[i]>=0)
                 {
                     sancha_guaidian_hang_right =i;
                     sancha_guaidian_lie_right = bianxian[i];
                   //  printf("三叉右拐点\r\n");
                     return 1;
                 }
             }
             break;
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】find_dingdian()
 【功    能】寻找三叉顶点
 【参    数】无
 【返 回 值】 1：有顶点  0： 无顶点
 【注意事项】
2022/04/04
 ----------------------------------------------------------------*/
uint8 find_dingdian(void)
{
   for(int i = 59;i>15;i--)
   {
       if(bin_image[i][(find_right_line[59]+find_left_line[59])/2] == 0)
       {
           dingdian_hang = i;
           dingdian_lie = (find_right_line[59]+find_left_line[59])/2;
         //  printf("三叉顶点出现\r\n");
           return 1;
       }
   }
   return 0 ;
}
/*---------------------------------------------------------------
 【函    数】shizi_process()
 【功    能】十字元素处理
 【参    数】 flag 处理状态
 【返 回 值】 无
 【注意事项】
2022/07/16
 ----------------------------------------------------------------*/
void shizi_process(uint8 *flag)
{
    switch(*flag)
    {
        case 1:  //补线
        {
            if(!find_shizi_buxian())
            {
                if(find_shizi_in_buxian())
                   *flag= 2;
            }

            if((left_shiziflag&&right_shiziflag))
            {
            AddLine(left_head_line,right_head_line,20,guaidian_lie_left+8,guaidian_hang_left,guaidian_lie_left,0);
            AddLine(left_head_line,right_head_line,20,guaidian_lie_right-8,guaidian_hang_right,guaidian_lie_right,1);
            }
            if(left_shiziflag&&lose_right_line[55]==1&&lose_right_line[56]==1&&lose_right_line[54]==1)
            {
                AddLine(left_head_line,right_head_line,20,guaidian_lie_left+8,guaidian_hang_left,guaidian_lie_left,0);
                AddLine(left_head_line,right_head_line,20,guaidian_lie_left+25,59,guaidian_lie_left+35,1);
            }
            if(right_shiziflag&&lose_left_line[55]==1&&lose_left_line[56]==1&&lose_left_line[54]==1)
            {
                AddLine(left_head_line,right_head_line,20,guaidian_lie_right-25,59,guaidian_lie_right-35,0);
                AddLine(left_head_line,right_head_line,20,guaidian_lie_right-8,guaidian_hang_right,guaidian_lie_right,1);
            }
            gpio_set(BEEP, GPIO_HIGH);
            /**
             *
             *
                       * 找上拐点

             *
             */
            /**
             *
             *
                       *  找到拐点
             *
             *  *flag= 2
             */
            break;
        }
        case 2:
        {
            //补线
            if(num_width[57]<=45&&num_width[56]<=45&&num_width[55]<=45)
            {
                *flag= 3;
                break;
            }

            if(find_shizi_in_buxian())
            {
                AddLine(left_head_line,right_head_line,Up_guaidian_hang,Up_right_guaidian_lie,59,Up_right_guaidian_lie+8,0);
                AddLine(left_head_line,right_head_line,Up_guaidian_hang,Up_left_guaidian_lie,59,Up_left_guaidian_lie-8,1);
                gpio_set(BEEP, GPIO_HIGH);
            }
            /**
             *
             *
                       *  拐点消失
                       *  宽度恢复
             *
                       *   圆环结束
             *  *flag= 0
             *
             */
            break;

        }
        case 3: //寻找右下拐点
            gpio_set(BEEP, GPIO_LOW);
            if (Find_guaidian(find_right_line,2,1)&&(num_width[guaidian_hang_right-1]>=50||num_width[guaidian_hang_right-2]>=50||num_width[guaidian_hang_right-3]>=50))
            {
                *flag= 4;
            }
            break;
        case 4:
            if (Find_guaidian(find_right_line,2,1)==0&&
               (num_width[58]>=50||num_width[57]>=50||num_width[56]>=50)&&
               (lose_right_line[50]==1&&lose_right_line[51]==1&&lose_right_line[49]==1)
               )
            {
                *flag= 5;
            }
            break;
        case 5:
            if((lose_right_line[57]==0&&lose_right_line[56]==0&&lose_right_line[55]==0)&&
               (lose_left_line[57]==0&&lose_left_line[56]==0&&lose_left_line[55]==0)&&
               (num_width[57]<=45&&num_width[56]<=45&&num_width[55]<=45)
               )
            {
                *flag= 0;
                gpio_set(BEEP, GPIO_LOW);
                ShiziIn = 0;
                shiziGet = 0;
                break;
            }
            if(find_shizi_in_buxian())
            {
                AddLine(left_head_line,right_head_line,Up_guaidian_hang,Up_right_guaidian_lie,59,Up_right_guaidian_lie+8,0);
                AddLine(left_head_line,right_head_line,Up_guaidian_hang,Up_left_guaidian_lie,59,Up_left_guaidian_lie-8,1);
            }
            gpio_set(BEEP, GPIO_HIGH);
            break;
    }
}
/*---------------------------------------------------------------
 【函    数】find_shizi_buxian()
 【功    能】寻找十字入口并且补线
 【参    数】
 【返 回 值】 无
 【注意事项】
2022/07/16
 ----------------------------------------------------------------*/
uint8 find_shizi_buxian(void)
{

    if(Find_guaidian(find_left_line,1,1))
    {
        left_shiziflag = 1;
        guaidian_lie_left  = find_left_line[guaidian_hang_left];           //左拐点列数
    }
    else
    {
        left_shiziflag = 0;
        guaidian_hang_left = 0;
        guaidian_lie_left   = 0;
    }
    if(Find_guaidian(find_right_line,2,1))
    {
        right_shiziflag = 1;
        guaidian_lie_right = find_right_line[guaidian_hang_right];
    }
    else
    {
        right_shiziflag = 0;
        guaidian_hang_right = 0;
        guaidian_lie_right   = 0;
    }
    if((left_shiziflag&&right_shiziflag)||
       (left_shiziflag&&lose_right_line[55]==1&&lose_right_line[56]==1&&lose_right_line[54]==1)||
       (right_shiziflag&&lose_left_line[55]==1&&lose_left_line[56]==1&&lose_left_line[54]==1)
      )
    {
        if((abs(num_width[guaidian_hang_right-3]-num_width[guaidian_hang_right])-abs(num_width[guaidian_hang_right]-num_width[guaidian_hang_right+1])>=10)
         ||(abs(num_width[guaidian_hang_left-3]-num_width[guaidian_hang_left])-abs(num_width[guaidian_hang_left]-num_width[guaidian_hang_left+1])>=10))
        {
            return 1;
        }
    }


    else
    {
        shiziflag = 0;
        return 0;
    }
}
/*---------------------------------------------------------------
 【函    数】find_shizi_in_buxian()
 【功    能】十字中并且补线
 【参    数】
 【返 回 值】 1 进入十字  0 未进
 【注意事项】
2022/07/16
 ----------------------------------------------------------------*/
uint8 find_shizi_in_buxian(void)
{
 //   if(Find_guaidian(find_left_line,1,0))
//        {
//            left_shiziflag = 1;
//            guaidian_lie_left  = find_left_line[guaidian_hang_left];           //左拐点列数
//        }
//        else
//        {
//            left_shiziflag = 0;
//            guaidian_hang_left = 0;
//            guaidian_lie_left   = 0;
//        }
//        if(Find_guaidian(find_right_line,2,0))
//        {
//            right_shiziflag = 1;
//            guaidian_lie_right = find_right_line[guaidian_hang_right];
//        }
//        else
//        {
//            right_shiziflag = 0;
//            guaidian_hang_right = 0;
//            guaidian_lie_right   = 0;
//        }
//        if(left_shiziflag&&right_shiziflag)
//        {
//            if((abs(num_width[guaidian_hang_right-3]-num_width[guaidian_hang_right])-abs(num_width[guaidian_hang_right]-num_width[guaidian_hang_right+1])>=10)
//             ||(abs(num_width[guaidian_hang_right-3]-num_width[guaidian_hang_right])-abs(num_width[guaidian_hang_right]-num_width[guaidian_hang_right+1])>=10))
//            {
//                return 1;
//            }
//        }
//
//
//        else
//        {
//            shiziflag = 0;
//            return 0;
//        }
    for(uint8 i = start_h-4;i>10;i--)
    {
        if((abs(num_width[i]-num_width[i+3])-abs(num_width[i]-num_width[i-1]))>=15)
        {
            Up_guaidian_hang = i;
            Up_right_guaidian_lie = find_right_line[Up_guaidian_hang-6];
            Up_left_guaidian_lie  = find_left_line[Up_guaidian_hang-6];
            return 1;
        }
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】ServoPID()
 【功    能】分段pid
 【参    数】
 【返 回 值】
 【注意事项】
2022/07/17
 ----------------------------------------------------------------*/
extern float ServoKp,ServoKd;      //舵机Kd微分系数

                    //    P  D
float Servo_PID[2][2] = {{4,2},
                         {5,10}};
void ServoPID(void)
{
    if((out_circle_judge(25) == 1&&abs(error_w <=6))&&HuandaoFlag==0&&PRoadFlag==0&&sanchaflag==0&&ShiziFlag==0&&GarageGet==0&&error_flag==1)
    {
        ServoKp= Servo_PID[0][0];
        ServoKd= Servo_PID[0][1];
    }
    else if((out_circle_judge(25) == 0&&abs(error_w > 6))&&HuandaoFlag==0&&PRoadFlag==0&&sanchaflag==0&&ShiziFlag==0&&GarageGet==0&&error_flag==1)
    {
        ServoKp= Servo_PID[1][0];
        ServoKd= Servo_PID[1][1];
    }
    if(HuandaoIn==1) //圆环        p:1 D:1  较好
    {
        ServoKp= 3.5;
        ServoKd= 1;
    }
    if(HuandaoIn==2)
    {
        ServoKp= 3.5;
        ServoKd= 1;
    }
    if(PRoadIn==2) //zuoP
    {
        ServoKp= 3.8;
        ServoKd= 1;
    }
    if(PRoadIn==1) //P
    {
        ServoKp= 3.5;
        ServoKd= 1;
    }
    if(sanchaflag)
    {
        ServoKp= 3.5;
        ServoKd= 7;
    }
    if(GarageGet)
    {
        ServoKp= 3.5;
        ServoKd= 1;
    }
}
/*---------------------------------------------------------------
 【函    数】PH_general_process()
  【功    能】P和环岛通用
 【参    数】
 【返 回 值】1:左边丢线 2：右边丢线
 【注意事项】
2022/07/17
 ----------------------------------------------------------------*/
uint8 PH_Guai_Left_Flag,PH_Guai_Right_Flag;
uint8 PH_general_process()
{
    uint8 line_left_flag = 1;
    uint8 line_right_flag = 1;
    uint8 LeftRoadFlag = 0,RightRoadFlag = 0;
    uint8 getPHflagzuo = 0,getPHflagyou = 0;
    uint8 kuanduflagzuo = 0,kuanduflagyou = 0,kuanduzuo = 0,kuanduyou = 0,kuanduzuo_you,kuanduyou_zuo;

//    LeftRoadFlag =  road_line_judge(1);
//    //Find_guaidian(find_left_line,1,1) == 1||Find_guaidian(find_right_line,2,1) == 1||||road_line_judge(1)==1||road_line_judge(2)==1
//    RightRoadFlag = road_line_judge(2);

    if(find_diuxian_huandao(48,1) == 0)
       LeftRoadFlag = 1;
    if(find_diuxian_huandao(48,2) == 0)
       RightRoadFlag = 1;

//    sprintf(buf2,"Lg:%d",LeftRoadFlag);
//    lcd_showstr(100,2,buf2);
//
//    sprintf(buf2,"Rg:%d",RightRoadFlag);
//    lcd_showstr(100,3,buf2);


    if(LeftRoadFlag == 1 && RightRoadFlag == 0)         //右线异常      左线正常
    {
       // printf("左边线异常！！\r\n");m0
        getPHflagyou = 1;
        if(getPHflagzuo)
            getPHflagzuo = 0;
    }

    if(LeftRoadFlag == 0 && RightRoadFlag == 1)         //左线异常      右线正常
    {
        //printf("右边线异常！！\r\n");
        getPHflagzuo = 1;
        if(getPHflagyou)
            getPHflagyou = 0;
    }

    if(getPHflagzuo){
       if(find_diuxian_huandao(40,2) == 1)
           return 0;

       right_k = min2_line(find_right_line,20,2);        //不改变数组的找斜率(right_k>=-0.45&&right_k<=0)||(right_k<=0.45&&right_k>=0)&&find_diuxian_line(15,lose_right_line)==0
        if(((right_k>=-0.45&&right_k<=0)||(right_k<=0.45&&right_k>=0))&&find_diuxian_line(20,lose_right_line)==0)
        {
           // printf("斜率满足！！\r\n");
            for(uint8 i = 58;i>=40;i--)
            {
                if(left_width[i]>35&&find_left_line[i]<15&&abs(find_right_line[i] - find_left_line[i])>=50)
                {

                    if(right_width[i]<=30)
                    {
                        kuanduzuo++;
                   }
                }

                if(kuanduzuo>=5)
                {
                   // printf("最后一个满足！！\r\n");
                   kuanduflagzuo = 1;
                   break;
                }


            }
            if(kuanduflagzuo&&garage_judge()==0)
            {
                error_flag = 3;
                lefttargetspeed = 315;
                righttargetspeed = 315;
                printf("识别到3！！！\r\n");
                getPHflagzuo = 0;
                return 1;
            }
            else
            {
                error_flag = 1;
                getPHflagzuo = 0;
                return 0;
            }
        }
    }
    if(getPHflagyou){
        if(find_diuxian_huandao(40,1) == 1)
           return 0;
        //printf("1重\r\n");
        left_k = min2_line(find_left_line,20,2);        //不改变数组的找斜率(left_k>=-0.45&&left_k<=0)||(left_k<=0.45&&left_k>=0)&&find_diuxian_line(15,lose_left_line)==0
        if(((left_k>=-0.45&&left_k<=0)||(left_k<=0.45&&left_k>=0))&&find_diuxian_line(20,lose_left_line)==0)
        {
           // printf("斜率满足！！\r\n");
            for(uint8 i = 58;i>=40;i--)
            {
                if(right_width[i]>35&&find_right_line[i]>80&&abs(find_right_line[i] - find_left_line[i])>=50)
                {     printf("RW\r\n");
                    if(left_width[i]<=30)
                    {
                         kuanduyou++;
                    }
                }
                if(kuanduyou>=5)
                {
                    //printf("最后一个满足！！\r\n");
                    kuanduflagyou = 1;
                    break;
                }
            }
            if(kuanduflagyou&&garage_judge()==0)
            {
                error_flag = 2;
                lefttargetspeed = 315;
                righttargetspeed = 315;
                printf("识别到2！！！\r\n");
                getPHflagyou = 0;
                return 2;
            }
            else
            {
                error_flag = 1;
                getPHflagyou = 0;
                return 0;
            }
        }
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】road_width_change(uint8  flag)
  【功    能】赛道宽度突变
 【参    数】flag 0:左突变  1: 右突变  2: 总突变    3:  寻找几次突变
 【返 回 值】1：突变  0: 未突变   else: 返回几次突变
 【注意事项】
2022/07/18
 ----------------------------------------------------------------*/
uint8  road_width_change(uint8  flag)
{   uint8 up_flag        = 0;
    int num_change = 0;
    switch(flag)
    {
        case 0:
        {
            for(int i = 50;i>=20;i--)
            {
                 if(
                     left_width[i-1]>left_width[i]
                   )
                 {
                     up_flag++;
                 }
                 if(up_flag>8)
                 {
                    // printf("三叉左突变\r\n");
                     return 1;

                 }
            }
            break;
        }
        case 1:
        {
            for(int i = 50;i>=20;i--)
                       {
                            if(
                                right_width[i-1]>right_width[i]
                              )
                            {
                                up_flag++;
                            }
                            if(up_flag>8)
                            {
                             //   printf("三叉右突变\r\n");
                                return 1;
                            }
                       }
                       break;
        }
        case 2:
        {
//            for(int i = 55;i<=20;i--)
//                            {
//                                 if(
//                                   )
//                                 {
//                                     return 1;
//                                     break;
//                                 }
//
//                            }
//                        return 0;
            break;
        }
        case 3:
        {
            for(int i = 55;i<=20;i--)
                            {
                                 if(  (
                                        ((num_width[i-4]-num_width[i-1])>=20)||
                                        ((num_width[i-5]-num_width[i-2])>=20)||
                                        ((num_width[i-6]-num_width[i-3])>=20)||
                                        ((num_width[i-7]-num_width[i-4])>=20)
                                       )                                     ||
                                       (
                                       ((num_width[i]-num_width[i+4])>=20) ||
                                       ((num_width[i]-num_width[i+5])>=20) ||
                                       ((num_width[i]-num_width[i+6])>=20) ||
                                       ((num_width[i]-num_width[i+7])>=20)
                                       )
                                   )
                                 {
                                     num_change++;
                                 }

                            }
                        return num_change;
            break;
        }
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】PH_shibie(uint8 flag)
  【功    能】赛道宽度突变
 【参    数】flag 环岛 OR P路口 执行状态
 【返 回 值】无
 【注意事项】
2022/07/18
 ----------------------------------------------------------------*/
uint8 huandao_detect_flag;
uint8 zuoguaiflag = 0;
void PH_shibie(uint8 flag)
{
    switch(flag)
    {
        case 1:

        if(find_left_line[59]<=10&&find_left_line[57]<=10&&find_left_line[56]<=10&&find_left_line[55]<=10)
            zuoguaiflag = 1;

        if(zuoguaiflag)
        {
            find_huxian(0,1,0,0);
            if(!huandao_detect_flag)
            {
                if(dz<=10&&dz>=4)
                {
                    if(find_diuxian(30)==0)
                    {
                        PRoadGet = 2;
                        error_flag = 1;
                        zuoguaiflag = 0;
//                        Game_Start = 0;
//                        leftspeed = 0;
//                        rightspeed = 0;
//                        lefttargetspeed = 0;
//                        righttargetspeed = 0;
                   lefttargetspeed =270;
                   righttargetspeed = 270;

                   printf("左P识别\r\n");
                    }
                }
            }
            if(dz>=12)
            {
                huandao_detect_flag = 1;
            }
            if(huandao_detect_flag)
            {
                if(xz >= 5)
                {
                    HuandaoGet = 1;
                    error_flag = 1;
                    huandao_detect_flag = 0;
                    zuoguaiflag = 0;
                    printf("左环识别\r\n");
                }
            }
        }
        break;
        case 2:
            if(find_right_line[59]>=70&&find_right_line[57]>=70&&find_right_line[56]>=70&&find_right_line[55]>=70)
                zuoguaiflag = 1;
            if(zuoguaiflag)
            {
                find_huxian(0,2,0,0);
                if(!huandao_detect_flag)
                {
                    if(xz<=10&&xz>=4)
                    {
                        if(find_diuxian(30)==0)
                        {
                            PRoadGet = 1;
                            error_flag = 1;
                            zuoguaiflag = 0;
                            lefttargetspeed =310 ;
                            righttargetspeed = 310  ;
                            printf("右P识别\r\n");
                        }
                    }
                }
                if(xz>=11)
                {
                    huandao_detect_flag = 1;
                }
                if(huandao_detect_flag)
                {
                    if(dz >= 3)
                    {
                        HuandaoGet = 2;
                        error_flag = 1;
                        huandao_detect_flag = 0;
                        zuoguaiflag = 0;
                        printf("右环识别\r\n");
                    }
                }
            }
            break;
    }
}
/*---------------------------------------------------------------
 【函    数】road_line_judge(uint8 flag)
  【功    能】直到判断
 【参    数】1：左直道判断    2：右直道判断     3： 双变判断
 【返 回 值】1：直道      0：不是直道
 【注意事项】
2022/07/20
 ----------------------------------------------------------------*/
uint8 road_line_judge(uint8 flag)
{
    uint16 down = 0,up=0,num_forty_five_ave=0,right_ave = 0,left_ave=0;
    uint16 left_flag=0,right_flag=0;
    uint8 right_forty_five_ave = 0;
    uint8 left_forty_five_ave = 0;
    for(int i = 59;i>=45;i--)
    {
        left_ave   += left_width[i];
        right_ave  += right_width[i];
    }
    left_ave  =  left_ave/15;

//    sprintf(buf2,"LA:%d", left_forty_five_ave);
//    lcd_showstr(100,2,buf2);

    right_ave =  right_ave/15;


//    sprintf(buf2,"RA:%d", right_forty_five_ave);
//    lcd_showstr(100,3,buf2);
//    (left_width[48]+left_width[49]+left_width+[50]+left_width[51]+left_width[52])/5;
   // num_forty_five_ave   =  (num_width[45]+num_width[46]+num_width[44])/3;

    left_forty_five_ave = (left_width[46]+left_width[45]+left_width[44])/3;
    right_forty_five_ave = (right_width[46]+right_width[45]+right_width[44])/3;
//    sprintf(buf2,"NA:%d", right_forty_five_ave);
//    lcd_showstr(100,4,buf2);

    switch(flag)
    {
        case 1:
        {
            for(uint8 i=58;i>=45;i--)
            {
                if(left_width[i]>=left_width[i-1])
                {
                    down++;
                }
            }  //17.6  15.6
            if((down>=9)&&(2<=left_forty_five_ave)&&(left_forty_five_ave<=25))
            {
//                sprintf(buf2,"DL:%d", down);
//                lcd_showstr(100,5,buf2);
               return 1;
            }

        break;

        }
        case 2:
        {
            for(uint8 i=58;i>=45;i--)
            {
                if(right_width[i]>=right_width[i-1])
                {
                    down++;
                }
            }
                     //14.6      12
            if((down>=9)&&(2<=right_forty_five_ave)&&(right_forty_five_ave<=25))
            {
//                sprintf(buf2,"DR:%d", down);
//                lcd_showstr(100,6,buf2);
               return 1;
            }

        break;

        }
        case 3:
        {
            for(uint8 i=58;i>=45;i--)
            {
                if(right_width[i]>=right_width[i-1])
                {
                    down++;
                }
            }
            if((down>=9)&&(8<=right_forty_five_ave)&&(right_forty_five_ave<=28))
            {
//                sprintf(buf2,"DR:%d", down);
//                lcd_showstr(100,6,buf2);
                right_flag = 1;
            }
             down =0;
            for(uint8 i=58;i>=45;i--)
            {
                if(left_width[i]>=left_width[i-1])
                {
                    down++;
                }
            }
            if((down>=9)&&(8<=left_forty_five_ave)&&(left_forty_five_ave<=28))
            {
//                sprintf(buf2,"DL:%d", down);
//                lcd_showstr(100,5,buf2);
                left_flag = 1;
            }
                    down=0;
            if(right_flag==1&&left_flag==1)
            {
                return 1;
            }

        break;

        }
    }
   return 0;
}
/*---------------------------------------------------------------
 【函    数】out_circle_judge()
  【功    能】出环判断
 【参    数】无
 【返 回 值】1：出      0：未出
 【注意事项】   255白  0黑
2022/07/22
 ----------------------------------------------------------------*/
uint8 out_circle_judge(uint8 find_hang)
{
    for(int i = 40;i>=find_hang;i--)
    {
        if(find_line_image[i][centre_point[58]]==0)
        {
            return 0;
            break;
        }
    }
    return 1;
}
/*---------------------------------------------------------------
 【函    数】garage_judge()
  【功    能】车库判断
 【参    数】无
 【返 回 值】1：库      0：未识别
 【注意事项】   255白  0黑
2022/07/22
 ----------------------------------------------------------------*/
uint16 garage_judge(void)
{
    black_patch = 0;       //斑马线黑点
    white_patch = 0;       //斑马线白点
   for(int i=58;i>20;i--)
   {
       black_patch = 0;
       for(int j=20;j<=60;j++)
       {

           if(  bin_image[i][j-1] == 255
                 &&bin_image[i][j] == 255
                 &&bin_image[i][j+1] == 0)
               {
                   black_patch++;
                   if(black_patch >=5)
                   {
                       garage_hang = i;

                       return 1;
                   }
               }

       }
   }
   return 0;
}
/*---------------------------------------------------------------
 【函    数】garage_stop()
  【功    能】车库刹车
 【参    数】状态
 【返 回 值】无
 【注意事项】   255白  0黑
2022/07/22
 ----------------------------------------------------------------*/
void garage_stop(uint8 flag)
{
    uint8 kuanduyou,kuanduyou_flag;
    switch(flag)
    {
        case 1:
            {  //左进库
                AddLine(left_head_line,right_head_line,10,80,58,47,0);//补左线，右拐点到左拐点上端
                error_flag = 2;
                find_huxian(0,1,0,0);
                if(dz>=12)
                {
                    stop=1;
                }
                if(stop==1)
                {
                    if(dz<=9)
                    {
                        error_flag = 4;
                         righttargetspeed = 0;
                         lefttargetspeed  = 0;//停车
                    }
                }
                break;
            }
        case 2:
        {  //右进库
            AddLine(left_head_line,right_head_line,10,5,58,40,1);//补左线，右拐点到左拐点上端
            error_flag = 3;
            find_huxian(0,2,0,0);
            if(!kuanduyou_flag)
            {
                for(int i = 58;i>45;i--)
                {
                    if(right_width[i]<=20)
                    {
                        kuanduyou++;
                    }
                    if(kuanduyou>=5)
                    {
                        kuanduyou_flag = 1;
                        break;
                    }
                }
            }
            if(xz>=12&&kuanduyou_flag)
            {
                stop=1;
            }
            if(stop==1)
            {
                if(xz<=7)
                {
                     error_flag = 4;
                     righttargetspeed = 0;
                     lefttargetspeed  = 0;//停车
                }
            }
            break;
        }
        break;
    }
}
/*---------------------------------------------------------------
 【函    数】garage_process()
  【功    能】车库处理程序
 【参    数】状态
 【返 回 值】无
 【注意事项】   255白  0黑
2022/07/22
 ----------------------------------------------------------------*/
void garage_process(void)
{
    static uint8 garage_hang_flag;
    static uint8 In_garage_flag;
   // printf("拐点行：%d\r\n",garage_hang);
      if(GarageIn==1)//车库在左
      {
          left_stop_flag =1;
          garge_scanf_line(2);
      }
      if(GarageIn==2)//车库在右
      {
          right_stop_flag =1;
          garge_scanf_line(1);
      }
      if(left_stop_flag==1&&right_stop_flag==1)
      {
             if(GarageIn==1)  //左入库
             {
                 if(garage_judge()==1)
                  {
                      if(garage_hang>=40)
                          In_garage_flag  = 1;
                  }
                 if(In_garage_flag)
                 {
                    // garage_stop(2);
                     error_w = -50;
                     righttargetspeed = 315;
                     lefttargetspeed = 315;
                     rightspeed = 315;
                     leftspeed = 315;
                     systick_delay_ms(STM0,280);
                     righttargetspeed = 0;
                     lefttargetspeed = 0;
                     Game_Start = 0;
                     leftspeed = 0;
                     rightspeed = 0;
                     error_flag = 4;

                 }

             }
             if(GarageIn==2)  //右入库
             {
                 if(garage_judge()==1)
                 {
                   if(garage_hang>=40)
                       In_garage_flag  = 1;
                  }
                  if(In_garage_flag)
                  {
                      error_w = 50;
                      righttargetspeed = 350;
                      lefttargetspeed = 350;
                      rightspeed = 350;
                      leftspeed = 400;
                      systick_delay_ms(STM0,270);
                      righttargetspeed = 0;
                      leftspeed = 0;
                      rightspeed = 0;
                      lefttargetspeed = 0;
                      error_flag = 4;
                      Game_Start = 0;
                  }

             }
      }
      if(garage_judge()==1)
      {
          if(garage_hang>=45)
          {
              garage_hang_flag = 1;
          }
      }

      if(garage_hang_flag)
      {
          if((left_stop_flag==0||right_stop_flag==0)&&(garage_judge()==0)&&((num_width[59]<=45&&num_width[59]>=30&&(lose_right_line[59]==0||lose_left_line[59]==0))
                                                                            &&(num_width[58]<=45&&num_width[58]>=30&&(lose_right_line[58]==0||lose_left_line[58]==0))
                                                                            &&(num_width[57]<=45&&num_width[57]>=30&&(lose_right_line[57]==0||lose_left_line[57]==0))
                                                                            &&(num_width[56]<=45&&num_width[56]>=30&&(lose_right_line[56]==0||lose_left_line[56]==0))
                                                                            &&(num_width[55]<=45&&num_width[55]>=30&&(lose_right_line[55]==0||lose_left_line[55]==0))
                                                                            &&(num_width[54]<=45&&num_width[54]>=30&&(lose_right_line[54]==0||lose_left_line[54]==0))))
         {
                error_flag  = 1;
                GarageIn = 0;
                if(!In_garage_flag)
                {
                    GarageGet = 0;
                    garage_hang_flag = 0;
                    printf("车库结束\r\n");
                }
         }
      }

}
/*---------------------------------------------------------------
 【函    数】find_diuxian(uint8 hang)
  【功    能】车库处理程序
 【参    数】状态
 【返 回 值】无
 【注意事项】   255白  0黑
2022/07/22
 ----------------------------------------------------------------*/
uint8 find_diuxian(uint8 hang)
{
    for(uint8 i = 58;i>hang;i--)
    {
        if(num_width[i]>45)
        {
            return 1;
        }
    }
    if((find_left_line[58]>20&&find_left_line[57]>20&&find_left_line[56]>20)&&
            (find_right_line[58]<70&&find_right_line[57]<70&&find_right_line[56]<70))
        return 0;
}
uint8 find_diuxian_line(uint8 hang,uint8 bianxian[])
{
    for(uint8 i = 59;i>hang;i--)
    {
        if(bianxian[i]==1)
        {
            return 1;
        }
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】find_garage_line_num(uint8 bianxian[],uint8 flag)
  【功    能】车库处理程序
 【参    数】状态
 【返 回 值】无
 【注意事项】  1 zuo 2 you
2022/07/22
 ----------------------------------------------------------------*/
uint8 find_garage_line_num(uint8 bianxian[],uint8 flag)
{
    uint8 line_num;
    switch(flag)
    {
        case 1:
            line_num = 0;
            for(uint8 i = garage_hang+15;i>20;i--)
            {
//                if(abs(bianxian[i]-bianxian[59])>20)
                if(bianxian[i]<=15)
                    line_num++;
                if(line_num>10)
                    return 1;
            }
            break;
        case 2:
            line_num = 0;
           for(uint8 i = garage_hang+15;i>20;i--)
           {
//               if(abs(bianxian[i]-bianxian[59])>20)
               if(bianxian[i]>=70)
                   line_num++;
               if(line_num>10)
                   return 1;
           }

           break;
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】find_sancha_guai(uint8 flag)
  【功    能】寻找三叉拐点
 【参    数】状态
 【返 回 值】无
 【注意事项】
2022/08/05
 ----------------------------------------------------------------*/
uint8 find_sancha_guai(uint8 hang)
{
    for(uint8 i = 59;i>hang;i--)
    {
        if(num_width[i]>=70)
        {
            return 1;
        }
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】Out_GarageProcess(uint8 flag)
  【功    能】出车库程序
 【参    数】状态
 【返 回 值】无
 【注意事项】
2022/08/05
 ----------------------------------------------------------------*/
void Out_GarageProcess(void)
{
    switch(Out_Garage_flag)
    {
        case 1:         //左出库，补右线
            AddLine(left_head_line,right_head_line,10,10,59,40,1);     //补右线
            righttargetspeed = 250;lefttargetspeed = 250;
            rightspeed = 270;      leftspeed = 270;
            min2_way(mid_head_line,15);    //中线拟合
            error_num(20,59);                 //偏差值计算
             if(find_diuxian_garage(35) == 0)
             {
                 //righttargetspeed = 250;lefttargetspeed = 250;
                 Game_Start = 2;
                 printf("我已出舱，感觉良好\r\n");
                 lefttargetspeed = 200;
                 righttargetspeed = 200;
             }
            break;
        case 2:         //右出库，补左线

            AddLine(left_head_line,right_head_line,10,90,59,70,0);     //补左线
            righttargetspeed = 250;lefttargetspeed = 250;
            rightspeed = 270;      leftspeed = 270;
            min2_way(mid_head_line,15);    //中线拟合
            error_num(20,59);                 //偏差值计算
            if(find_diuxian_garage(35) == 0)
            {
                righttargetspeed = 250;lefttargetspeed = 250;
                Game_Start = 2;
                printf("我已出舱，感觉良好\r\n");
            }
            break;
    }
}
/*---------------------------------------------------------------
 【函    数】Speed_Conctrl(void)
  【功    能】出车库程序
 【参    数】状态
 【返 回 值】无
 【注意事项】
2022/08/05
 ----------------------------------------------------------------*/
void Speed_Conctrl(void)
{

    if(Game_Start == 0)
    {
        leftspeed = 0;
        rightspeed = 0;
        lefttargetspeed = 0;
        righttargetspeed = 0;
    }


    if(out_circle_judge(25) == 1&&HuandaoFlag==0&&PRoadFlag==0&&ShiziFlag==0&&PandHuandaoFlag==0&&sanchaflag==0)
    {
//        if(LongRoad_UpSpeed_Flag == 0)
//        {
////            lefttargetspeed = 315;
////            righttargetspeed = 315;
//        }
        LongRoad_UpSpeed_Flag = 1;
    }
    else if(out_circle_judge(20) == 0&&HuandaoFlag==0&&PRoadFlag==0&&ShiziFlag==0&&PandHuandaoFlag==0&&sanchaflag==0)
    {
//        if(LongRoad_UpSpeed_Flag == 1)
//        {
////            lefttargetspeed = 310;
////            righttargetspeed = 310;
//        }
        LongRoad_UpSpeed_Flag = 0;
    }
    if(HuandaoFlag||PRoadFlag||PandHuandaoFlag||sanchaflag)
    {
        LongRoad_UpSpeed_Flag = 0;
    }
    if(abs(error_w)>=8&&out_circle_judge(25) == 0&&HuandaoFlag==0&&PRoadFlag==0&&sanchaflag==0)//&& sanchaflag == 0 *(1+0.18*(tan((angle*3.14)/180)/0.5))
    {
        leftspeed  = (int16)(lefttargetspeed *(1+0.18*(tan((angle*3.14)/180)/0.5)));
        rightspeed = (int16)(righttargetspeed *(1-0.18*(tan((angle*3.14)/180)/0.5)));
    }
    else if(PRoadFlag||HuandaoFlag||sanchaflag)
    {
        leftspeed  = (int16)(lefttargetspeed *(1+0.18*(tan((angle*3.14)/180)/0.5)));
        rightspeed = (int16)(righttargetspeed *(1-0.18*(tan((angle*3.14)/180)/0.5)));
    }
    else
    {
         leftspeed =  lefttargetspeed;
         rightspeed = righttargetspeed;
    }


}
/*---------------------------------------------------------------
 【函    数】find_normal_line(uint8 bianxian[],uint8 flag)
  【功    能】寻找正常的边线
 【参    数】状态
 【返 回 值】无
 【注意事项】
2022/08/05
 ----------------------------------------------------------------*/
uint8 find_diuxian_garage(uint8 hang)
{
    for(uint8 i = 58;i>hang;i--)
    {
        if(num_width[i]>45)
        {
            return 1;
        }
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】find_diuxian_huandao(uint8 hang,uint8 flag)
  【功    能】寻找环岛和P中的特殊元素
 【参    数】 flag 1:左 2：右
 【返 回 值】寻找到丢线
 【注意事项】   255白  0黑
2022/08/08
 ----------------------------------------------------------------*/
uint8 find_diuxian_huandao(uint8 hang,uint8 flag)
{
    switch(flag)
    {
        case 1:
            for(uint8 i = 58;i > hang;i--)
            {
                if(abs(find_right_line[i] - find_left_line[i])>=40)
                {
                    if(left_width[i] >=35)
                       return 1;
                }
            }
            break;
        case 2:
            for(uint8 i = 58;i > hang;i--)
            {
                if(abs(find_right_line[i] - find_left_line[i])>=40)
               {
                    if(right_width[i] >=35)
                       return 1;
               }
            }
            break;
    }
    return 0;
}
/*---------------------------------------------------------------
 【函    数】ramp_judge(void)
  【功    能】寻找坡道
 【参    数】 无
 【返 回 值】
 【注意事项】
2022/08/08
 ----------------------------------------------------------------*/
void ramp_judge(void)
{
    if(gpio_get(P10_6) == 0&&ramp_flag == 0)
    {
        gpio_set(BEEP, GPIO_HIGH);

        ramp_flag = 1;
        lefttargetspeed = 260;
        righttargetspeed = 260;
        error_flag = 4;
        printf("上坡\r\n");
    }
}
/*---------------------------------------------------------------
 【函    数】ramp_process(uint8 *flag)
  【功    能】坡道处理程序
 【参    数】 坡道状态位
 【返 回 值】
 【注意事项】
2022/08/08
 ----------------------------------------------------------------*/
void ramp_process(uint8 *flag)
{
    static uint8 zai_flag,kuan_flag,over_flag,light_flag;

    if(num_width[59]<=20||num_width[58]<=20||num_width[56]<=20)
    {
        zai_flag = 1;
        lefttargetspeed = 250;
        righttargetspeed = 250;
        error_flag = 1;
        printf("下坡\r\n");
    }
    if(zai_flag)
    {
        if(num_width[59]>=45&&num_width[58]>=45&&num_width[56]>=45)
        {
            kuan_flag = 1;
        }
    }
    if(kuan_flag)
    {
        if(gpio_get(P10_6) == 0)
        {
            light_flag = 1;
        }
        if(num_width[59]<=40&&num_width[58]<=40&&num_width[56]<=40&&gpio_get(P10_6) == 1&&light_flag)
            over_flag = 1;
    }
    if(over_flag)
    {
        *flag = 0;
        lefttargetspeed = 315;
        righttargetspeed = 315;
        zai_flag = 0;
        kuan_flag = 0;
        over_flag = 0;

        light_flag = 0;
        printf("结束\r\n");
        gpio_set(BEEP, GPIO_LOW);
    }
}
/*---------------------------------------------------------------
 【函    数】Car_Recieve(void)
  【功    能】小车接收程序
 【参    数】
 【返 回 值】
 【注意事项】
2022/08/08
 ----------------------------------------------------------------*/
//
//extern uint8 Rxflag;
//extern uint8 Rxbuf[64];
//uint8 cnt = 0;
//uint8 index = 0;
//void Car_Recieve(void)
//{
//    if(Rxflag)
//        {
//
//            Rxflag = 0;
//            if(Rxbuf[0] == 'K'){
//                switch(Rxbuf[1])
//                {
//                    case 'p':
//                          if('.' == Rxbuf[4]&&Rxbuf[6]!=0x0a)
//                          {
//                              pidright.kp = (Rxbuf[3]-'0') + (Rxbuf[5]-'0') * 0.1 + (Rxbuf[6]-'0') * 0.01;
//                          }
//                          else if('.' == Rxbuf[4]&&Rxbuf[6]==0x0a)
//                          {
//                              pidright.kp = (Rxbuf[3]-'0') + (Rxbuf[5]-'0') * 0.1;
//                          }
//                          if('.' == Rxbuf[5]&&Rxbuf[7]!=0x0a)
//                          {
//                              pidright.kp = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0') + (Rxbuf[6]-'0') * 0.1 + (Rxbuf[7]-'0') * 0.01;
//                          }
//                          else if('.' == Rxbuf[5]&&Rxbuf[7]==0x0a)
//                          {
//                              pidright.kp = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0') + (Rxbuf[6]-'0') * 0.1;
//                          }
//                          if(Rxbuf[4]==0x0a)
//                          {
//                              pidright.kp = (Rxbuf[3]-'0');
//                          }
//                          else if(Rxbuf[5]==0x0a)
//                          {
//                              pidright.kp = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0');
//                          }
//                          break;
//                    case 'd':
//                            if('.' == Rxbuf[4]&&Rxbuf[6]!=0x0a)
//                          {
//                              pidright.kd = (Rxbuf[3]-'0') + (Rxbuf[5]-'0') * 0.1 + (Rxbuf[6]-'0') * 0.01;
//
//                          }
//                          else if('.' == Rxbuf[4]&&Rxbuf[6]==0x0a)
//                          {
//                              pidright.kd = (Rxbuf[3]-'0') + (Rxbuf[5]-'0') * 0.1;
//                          }
//                          if('.' == Rxbuf[5]&&Rxbuf[7]!=0x0a)
//                          {
//                              pidright.kd = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0') + (Rxbuf[6]-'0') * 0.1 + (Rxbuf[7]-'0') * 0.01;
//                          }
//                          else if('.' == Rxbuf[5]&&Rxbuf[7]==0x0a)
//                          {
//                              pidright.kd = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0') + (Rxbuf[6]-'0') * 0.1;
//                          }
//                          if(Rxbuf[4]==0x0a)
//                          {
//                              pidright.kd = (Rxbuf[3]-'0');
//                          }
//                          else if(Rxbuf[5]==0x0a)
//                          {
//                              pidright.kd = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0');
//                          }
//                          break;
//                    case 'i':
//                          if('.' == Rxbuf[4]&&Rxbuf[6]!=0x0a)
//                          {
//                              pidright.ki = (Rxbuf[3]-'0') + (Rxbuf[5]-'0') * 0.1 + (Rxbuf[6]-'0') * 0.01;
//
//                          }
//                          else if('.' == Rxbuf[4]&&Rxbuf[6]==0x0a)
//                          {
//                              pidright.ki = (Rxbuf[3]-'0') + (Rxbuf[5]-'0') * 0.1;
//                          }
//                          if('.' == Rxbuf[5]&&Rxbuf[7]!=0x0a)
//                          {
//                              pidright.ki = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0') + (Rxbuf[6]-'0') * 0.1 + (Rxbuf[7]-'0') * 0.01;
//                          }
//                          else if('.' == Rxbuf[5]&&Rxbuf[7]==0x0a)
//                          {
//                              pidright.ki = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0') + (Rxbuf[6]-'0') * 0.1;
//                          }
//                          if(Rxbuf[4]==0x0a)
//                          {
//                              pidright.ki = (Rxbuf[3]-'0');
//                          }
//                          else if(Rxbuf[5]==0x0a)
//                          {
//                              pidright.ki = (Rxbuf[3]-'0') * 10 + (Rxbuf[4]-'0');
//                          }
//                          break;
//                    case 'r':
//                        for(cnt = 4;cnt <= 7;cnt++)
//                        {
//                            if(Rxbuf[cnt] == 0x0a)
//                            {
//                                index = cnt - 3;            //几位数字
//                                if(index == 1)
//                                {
//                                    righttargetspeed = Rxbuf[3] - '0';
//                                }
//                                else if(index == 2)
//                                {
//                                    righttargetspeed = (Rxbuf[3] - '0')*10 + (Rxbuf[4] - '0');
//                                }
//                                else if(index == 3)
//                                {
//                                    righttargetspeed = (Rxbuf[3] - '0')*100 + (Rxbuf[4] - '0')*10 + (Rxbuf[5] - '0');
//                                }
//                                else if(index == 4)
//                                {
//                                    righttargetspeed = (Rxbuf[3] - '0')*1000 + (Rxbuf[4] - '0')*100 + (Rxbuf[5] - '0')*10 + (Rxbuf[6] - '0');
//                                }
//                                break;
//                            }
//                        }
//
//                        break;
//                    case 'l':
//                        for(cnt = 4;cnt <= 7;cnt++)
//                        {
//                            if(Rxbuf[cnt] == 0x0a)
//                            {
//                                index = cnt - 3;            //几位数字
//                                if(index == 1)
//                                {
//                                    lefttargetspeed = Rxbuf[3] - '0';
//                                }
//                                else if(index == 2)
//                                {
//                                    lefttargetspeed = (Rxbuf[3] - '0')*10 + (Rxbuf[4] - '0');
//                                }
//                                else if(index == 3)
//                                {
//                                    lefttargetspeed = (Rxbuf[3] - '0')*100 + (Rxbuf[4] - '0')*10 + (Rxbuf[5] - '0');
//                                }
//                                else if(index == 4)
//                                {
//                                    lefttargetspeed = (Rxbuf[3] - '0')*1000 + (Rxbuf[4] - '0')*100 + (Rxbuf[5] - '0')*10 + (Rxbuf[6] - '0');
//                                }
//                                break;
//                            }
//                        }
//                        break;
//                    case 's':
//                        Game_Start = 0;
//                        lefttargetspeed = 0;
//                        righttargetspeed = 0;
//                        break;
//                    case 'a':
//                        lefttargetspeed = 300;
//                        righttargetspeed = 300;
//
//
//
//                        break;
//                }
//                pidleft.kp = pidright.kp;
//                pidleft.ki = pidright.ki;
//                pidleft.kd = pidright.kd;
//                righttargetspeed = lefttargetspeed;
////                printf("Kpss:%.2f",pidright.kp);
//
//            }
//
//           memset(Rxbuf,'\0',sizeof(Rxbuf));
//        }
//}
/*---------------------------------------------------------------
 【函    数】garge_guaidian(void)
  【功    能】车库拐点
 【参    数】1 ：左拐点 2：右拐点
 【返 回 值】拐点行数
 【注意事项】
2022/08/12
 ----------------------------------------------------------------*/
uint8 garge_guaidian_hang =0;
void garge_guaidian(uint8 flag)
{
    if(flag==1)
    {
        for(uint8 i = start_h;i>=25;i-- )
        {
            if(left_width[i]>10)
            {
             if(find_left_line[i-1] - find_left_line[i]>=20)
             {
                 garge_guaidian_hang = i-1;
                 break;
             }
            }
        }
    }
    if(flag==2)
    {
        for(uint8 i = start_h;i>=25;i-- )
        {
             if(find_right_line[i] - find_right_line[i-1]>=5)
             {
                 garge_guaidian_hang = i-1;
                 break;
             }
        }
    }
}
/*---------------------------------------------------------------
 【函    数】garge_scanf_line(void)
  【功    能】车库再次边线寻找
 【参    数】1 ：左线 2：右线
 【返 回 值】
 【注意事项】
2022/08/12
 ----------------------------------------------------------------*/
void garge_scanf_line(uint8 flag)
{
    if(flag==1)
    {
        for(uint8 i = 59; i>30 ;i--)
        {
            for(uint8 j = 10; j<47 ;j++)// 255 白  0 黑
            {
                if(find_line_image[i][j]==0&&
                   find_line_image[i][j-1]==0&&
                   find_line_image[i][j+1]==255)
                {
                    left_head_line[i] = j;
                    xianshi[i][j] = 5;
                    break;
                }
            }
        }
    }
    if(flag==2)
    {
        for(uint8 i = 59; i>30 ;i--)
        {
            for(uint8 j = 70; j>47 ;j--)// 255 白  0 黑
            {
                if(find_line_image[i][j]==255&&
                   find_line_image[i][j+1]==0&&
                   find_line_image[i][j+2]==0)// 255 白  0 黑
                {
                    right_head_line[i] = j;
                    xianshi[i][j] = 5;
                    break;
                }
            }
        }
    }
}
