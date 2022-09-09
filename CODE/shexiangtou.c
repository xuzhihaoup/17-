#include "shexiangtou.h"
#include "zf_stm_systick.h"
#include "math.h"
#include "headfile.h"

uint8 yasuo[HIGH][WIDTH];    //储存压缩图像
uint8 bin_image[HIGH][WIDTH];//储存二值图像
uint8 xianshi[HIGH][WIDTH];
uint8 find_line_image[HIGH][WIDTH];
uint16 daf=0;
float BlackThres;


short GetOSTU (unsigned char tmImage[HIGH][WIDTH],uint8 hang,uint8 lie)
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < hang; j++)
    {
        for (i = 0; i < lie; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}
/*---------------------------------------------------------------
 【函    数】yasuo_image
 【功    能】图像进行压缩    压缩后图像大小  60*94
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void yasuo_image(uint8* p)   //void yasuo_image(uint8* p,int yuzhi)
{
    uint16 i,j;
       for(i=0;i<60;i++)
       {
           for(j=0;j<94;j++)
           {
               yasuo[i][j]=*p;
               p+=2;
           }
       }
}
/*---------------------------------------------------------------
 【函    数】Gaosi_Image_Filter
 【功    能】高斯滤波
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void Gaosi_Image_Filter()
{
    /*     卷积因子                                      转置
           0.0761245  0.132451  0.0761245                 0.0761245        0.132451        0.0761245
           0.132451   0.21043   0.132451                  0.132451         0.21043         0.132451
           0.0761245  0.132451  0.0761245                 0.0761245        0.132451        0.0761245
    */
        uint8  conv2[64][98],conv_out_image[62][96];; //卷积层
        int i =0,j=0,con_i = 0,con_j = 0 , y = 0, x = 0;
        //图像输入卷积层
        for(i=2;i<62;i++)
        {
            for(j=2;j<96;j++)
            {
             conv2[i][j] = yasuo[con_i][con_j];
             con_j++;
            }
            con_i++;
            con_j=0;
        }
        //计算卷积
            for(i=0;i<62;i++)
            {//(conv2[x][y]+2*conv2[x+1][y]+conv2[x+2][y]) - (conv2[x][y+2]+2*conv2[x+1][y+2]+conv2[x+2][y+2])
                for(j=0;j<96;j++)
                {
                              conv_out_image[i][j] = (uint8)(0.0761245*conv2[x][y] + 0.132451*conv2[x][y+1] + 0.0761245*conv2[x][y+2]    +
                               0.132451*conv2[x+1][y] + 0.21043*conv2[x+1][y+1] + 0.132451*conv2[x+1][y+2]  +
                               0.0761245*conv2[x+2][y] + 0.132451*conv2[x+2][y+1]+ 0.0761245*conv2[x+2][y+2]);
                 y++;
                }
                x++;
                y=0;
            }
         //卷积输出
            y =0;x= 0;
         for(i=1;i<61;i++)
         {
             for(j=1;j<95;j++)
             {
                 yasuo[x][y]  = conv_out_image[i][j];
                 y++;
             }
             x++;
             y=0;
         }
}
/*---------------------------------------------------------------
 【函    数】Threshold_way
 【功    能】阈值处理   二值化
 【参    数】模式  0：  Prewitt 边缘检测        1：大津法    2：sobel 边缘检测      3：局部阈值
 【返 回 值】无
 【注意事项】 2022年3月25
 255白  0 黑  1红  [MT9V03X_H][MT9V03X_W]
 ----------------------------------------------------------------*/
void  Threshold_way(uint8 Threshold_mode)
{
    if(Threshold_mode==0)
    {
        PrewittAutoThreshold (yasuo,bin_image);
    }
    else if(Threshold_mode==1)
    {

       MyOSTU(94,60,yasuo[0]);
       gary2binaryzation(yasuo);
    }
    else  if(Threshold_mode==2)
    {
        sobelAutoThreshold();
    }
    else  if(Threshold_mode==3)
    {
        part_AutoThreshold();
    }
}
/*********大津阈值分割法（优化）********************/
//参数解释：宽 高 图像指针 起始行  起始列 处理行大小  处理列大小
#define GrayScale 256   //frame灰度级
typedef unsigned char uchar;
int pixel[256]={0};  //这个标志位不能定义在函数内部，
void MyOSTU(int width,int height,uint8 *Image)
{
    //systick_start(STM0);
    int threshold1=0;
    int32 sum_gray=0;
    int32 sum_pix_num=0;
    int32 pl_pix_num=0;
    int32 p2_pix_mum=0;
    int32 p1_sum_gray=0;
    float m1=0;
    float m2=0;
    float V=0;
    float variance=0;
    int i,j,k=0;

    for(i = 0;i<256;i++)
        pixel[i] = 0;

    //统计每个灰度级中像素的个数
    for(i = 0; i < height; i++)
    {
        for(j = 0;j < width;j++)
        {
            pixel[(int)Image[i * width + j]]++;
        }
    }

        for(k=0;k<GrayScale;k++)
        {
            sum_gray+=k*pixel[k];//灰度直方图质量矩
            sum_pix_num+=pixel[k];//总像素个数
        }

        for(k=0;k<GrayScale-1;k++)
        {
            pl_pix_num+=pixel[k];//第一部分像素个数
            p2_pix_mum=sum_pix_num-pl_pix_num;//第二部分像素个数
            p1_sum_gray+=k*pixel[k];   //第一部分质量矩
            m1=(float)p1_sum_gray/pl_pix_num;//第一部分灰度均值
            m2=(float)(sum_gray-p1_sum_gray)/p2_pix_mum;//第二部分灰度均值

            V=pl_pix_num*p2_pix_mum*(m1-m2)*(m1-m2);

            if(V>variance)//将类间方差较大时的灰度值作为阈值
            {
                variance=V;
                threshold1=k;
            }
        }
        BlackThres=threshold1;

}
/*---------------------------------------------------------------
 【函    数】PrewittAutoThreshold
 【功    能】Prewitt算子边缘检测
 【参    数】传入图像数组
 【返 回 值】无
 【注意事项】 2022年3月25
 255白  0 黑  1红  [MT9V03X_H][MT9V03X_W]
 ----------------------------------------------------------------*/
void PrewittAutoThreshold (uint8 imageIn[HIGH][WIDTH], uint8 imageOut[HIGH][WIDTH])
{
    /** 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = WIDTH - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = HIGH - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]                      // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];             // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]                      // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];             // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]                //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];       //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]                // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];       //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
            temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                    + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
                    + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)
            {
                imageOut[i][j] = 255;
            }
            else
            {
                imageOut[i][j] = 0;
            }

        }
    }
}
/*---------------------------------------------------------------
 【函    数】sobelAutoThreshold
 【功    能】sobel算子边缘检测
 【参    数】无
 【返 回 值】无
 【注意事项】 2022年3月25
             图像压缩后调用
             传入的图像大小必须为   60*94   因为卷积层被限定  如需更改卷积层、X层、Y层大小为 原图H + 4     原图W + 4
       255白  0 黑  1红
 ----------------------------------------------------------------*/
float conv_image[60][94];
uint8 image_conv[60][94];
uint32 MAX = 0,MIN = 1;
short yuzhi;
void sobelAutoThreshold()
{



/*     sobel卷积因子
       -1  0  1                                  1  2  1
       -2  0  2                                  0  0  0
       -1  0  1                                 -1 -2 -1
*/
/*     sobel卷积因子转置
        1  0  -1                                -1 -2 -1
        2  0  -2                                 0  0  0
        1  0  -1                                 1  2  1
*/
    uint8  conv2[64][98]; //卷积层
    float conv_x[62][96];//X维度
    float conv_y[62][96];//Y维度
    float SobelThreshold;
    float sum;
    int ss = 0;
    int i =0,j=0,con_i = 0,con_j = 0,x = 0,y = 0,xx = 0,yy = 0;

    MAX = 0; MIN = 0;
    //图像输入卷积层
    for(i=2;i<62;i++)
    {
        for(j=2;j<96;j++)
        {
         conv2[i][j] = yasuo[con_i][con_j];
         con_j++;
        }
        con_i++;
        con_j=0;
    }
////计算卷积
    for(i=0;i<62;i++)
    {
        for(j=0;j<96;j++)
        {
         conv_x[i][j] = (float)abs((conv2[x][y]+2*conv2[x+1][y]+conv2[x+2][y]) - (conv2[x][y+2]+2*conv2[x+1][y+2]+conv2[x+2][y+2])) / 255;
         conv_y[i][j] = (float)abs((conv2[x+2][y]+2*conv2[x+2][y+1]+conv2[x+2][y+2]) - (conv2[x][y]+2*conv2[x][y+1]+conv2[x][y+2])) / 255;
         y++;
        }
        x++;
        y=0;
    }
////图像卷积输出  输出大小于原图像大小相同并且二值化   255白  0 黑
    //图像卷积输出  输出大小于原图像大小相同并且二值化   255白  0 黑

  for(xx=0;xx<60;xx++)
   {
       for(yy=0;yy<94;yy++)
       {
           conv_image[xx][yy] = conv_x[xx][yy]+conv_y[xx][yy];
           if( conv_image[xx][yy] < 1.0)
          {
              image_conv[xx][yy] = (uint8)(conv_image[xx][yy] * 255);
              if(image_conv[xx][yy] == 0)
              MAX++;
          }
          else if(conv_image[xx][yy] >= 1.0)
          {
              image_conv[xx][yy] =  (uint8)(255 / conv_image[xx][yy]);
              MIN++;
          }
       }
   }

  yuzhi = GetOSTU(image_conv,60,94);
//  MyOSTU(94,60,image_conv);

  for(xx=0;xx<60;xx++)
    {
        for(yy=0;yy<94;yy++)
        {

            if (image_conv[xx][yy]>=yuzhi)
            {
                bin_image[xx][yy] = 0;
                xianshi[xx][yy] = 0;
            }
            else
            {
                bin_image[xx][yy] = 255;
                xianshi[xx][yy] = 255;
            }
        }
     }
}
/*---------------------------------------------------------------
 【函    数】part_AutoThreshold
 【功    能】局部阈值
 【参    数】传入图像数组
 【返 回 值】无
 【注意事项】 2022年3月30
 将压缩图片分为 20个区域 对每个区域进行阈值处理
 255白  0 黑  1红  [MT9V03X_H][MT9V03X_W]
 ----------------------------------------------------------------*/
uint8  part_image_first[6][47];  //第一个区域图像
uint8  part_image_sec[6][47];      //第二个区域图像
void  part_AutoThreshold()
{
    int image_flag = 1;
    float first_Thres = 0,sec_Thres = 0;
       for(int i = 0; i<60;i++)
       {
           for(int j = 0; j<94;j++)
           {
               if(j<=46)
               {
                   part_image_first[i][j] =  yasuo[i][j];
               }
               else
               {
                   part_image_sec[i][j]   =  yasuo[i][j];
               }
           }
           image_flag++;
           if(image_flag == 6)  //两个  6*47 大小的区域采集完成     60 * 94 大小的图像 分为了20个区域
           {
               //计算区域阈值
                MyOSTU(47,6,part_image_first);
               first_Thres = BlackThres;
               MyOSTU(47,6,part_image_sec);
               sec_Thres   = BlackThres;
               //对两个区域二值化
               for(int H = i - 6 , h = 0;H < i , h < 6; H++ ,h++)
               {
                   for(int W = 0; W<94;W++)
                   {
                                  if(W<=46)
                                  {
                                      bin_image[H][W] =  part_image_first[h][W] > first_Thres ? 0 : 255;
                                      xianshi[H][W] = bin_image[H][W];
                                  }
                                  else
                                  {
                                      bin_image[H][W] =  part_image_sec[h][W]   >  sec_Thres ? 0 : 255;
                                      xianshi[H][W] = bin_image[H][W];
                                  }
                   }
               }
               image_flag = 1;
           }

       }
}





//图像打印
void lcd_displayimage(uint8 *p, uint16 width, uint16 height)
{
    uint16 i,j=0;

    uint16 color = 0;
    uint16 temp = 0;

    uint16 coord_x = 0;
    uint16 coord_y = 0;

    coord_x = width>TFT_X_MAX?TFT_X_MAX:width;
    coord_y = height>TFT_Y_MAX?TFT_Y_MAX:height;
    lcd_set_region(0,0,coord_x-1,coord_y-1);


    for(i=59;i>=29;i=i-2)
    {
        if(mid_head_line[i] != 0)
            xianshi[i][mid_head_line[i]]=1;
    }
    for(j=0;j<coord_y;j++)
    {
        for(i=0;i<coord_x;i++)
        {

            temp = *(p+j*width+i*width/coord_x);//读取像素点

            if(temp == 1)
                color = RED;
            else if(temp == 2)
               color = GREEN;
            else if(temp == 3)
                color = YELLOW;
            else if(temp == 4)
                color = GREEN;
            else if(temp == 5)
                color = BLUE;
            else if(temp == 6)
                color = PURPLE;
            else
            {
                color=(0x001f&((temp)>>3))<<11;
                color=color|(((0x003f)&((temp)>>2))<<5);
                color=color|(0x001f&((temp)>>3));
            }

            lcd_writedata_16bit(color);

        }
    }
}
/*---------------------------------------------------------------
 【函    数】auto_threshold
 【功    能】计算边缘检测阈值
 【参    数】无
 【返 回 值】无
 【注意事项】 2022年3月26
       255白  0 黑  1红
 ----------------------------------------------------------------*/
int class_num;

char buf3[64];
int image_class[60] = {0};
float auto_threshold(float (*Image)[94])
{
   // systick_start(STM0);
    int i = 0, j =0,class_flag = 0 ,class_max = 0,class_max_flag = 0;
    float qujian = 0.35 ,Threshold = 0;  //0.57 使用excle拟合得到的区间间距
    for(int x = 0;x<60;x++)
        image_class[x] = 0;
    MAX = 0,MIN = 1;
   //寻找最大值 与最小值
    for(i = 0; i<58;i++)
    {
        for(j = 0; j<94;j++)
        {
            if(Image[i][j] > MAX)
           {
               MAX = Image[i][j];
               continue;
           }
           else if(Image[i][j] < MIN && Image[i][j] !=0 )
           {
               MIN = Image[i][j];
               continue;
           }
        }
    }
    //找到有多少类值
    class_num =(int) ((MAX-MIN) / qujian);
    //找到每个区间有多少个
    for(i = 0; i<60;i++)
    {
        for(j = 0; j<94;j++)
        {
            if(Image[i][j]>=MIN && Image[i][j]< MIN + qujian)
           {
               image_class[0] = image_class[0] + 1;
               continue;
           }
           else if(Image[i][j]>=(MIN + (class_num-1)*qujian) && Image[i][j]<= MAX)
           {
               image_class[class_num-1] = image_class[class_num-1] + 1;
               continue;
           }
          for(class_flag = 1;class_flag <= class_num - 2;class_flag++ )
          {

             if(Image[i][j]>=(MIN + class_flag*qujian) && Image[i][j]< MIN + (class_flag+1)*qujian)
              {
                  image_class[class_flag] = image_class[class_flag] + 1;
                  continue;
              }

          }
        }
    }
    //找出那个区间个数最少
    class_max = image_class[0];
   for(i=0;i<class_num-1;i++)
   {
      if(image_class[i] < class_max )
      {
          class_max = image_class[i];
          class_max_flag = i;
      }
   }
    Threshold = MIN + (class_max_flag + 1) * qujian;
//    sprintf(buf3,"Threshold:%.2f",Threshold);
//    lcd_showstr(0,7,buf3);
    return Threshold;
}
void Init(void)
{
    //用户在此处调用各种初始化函数等
    mt9v03x_init();     //摄像头初始化
    uart_init(UART_0,115200 , UART0_TX_P14_0, UART0_RX_P14_1);//串口初始化
    gtm_pwm_init(ATOM0_CH5_P02_5,17*1000,0);//电机初始化ATOM0_CH3_P10_6
    gtm_pwm_init(ATOM0_CH4_P02_4,17*1000,0);
    gtm_pwm_init(ATOM0_CH7_P02_7,17*1000,0);//电机初始化ATOM0_CH3_P10_6
    gtm_pwm_init(ATOM0_CH6_P02_6,17*1000,0);
   pit_interrupt_ms(CCU6_0, PIT_CH0, 10);

    gtm_pwm_init(ATOM0_CH1_P33_9,50,810);  //舵机初始化      1060      980    900     725       810     895
    gpt12_init(GPT12_T5,GPT12_T5INB_P10_3,GPT12_T5EUDB_P10_1);      //左编码器初始化
    gpt12_init(GPT12_T6,GPT12_T6INA_P20_3,GPT12_T6EUDA_P20_0);      //右编码器初始化
   //adc_init(ADC_0, ADC0_CH8_A8);           //ADC测电池电压
    Key_Init();           //按键初始化
    pid_left_init();
    pid_right_init();
    Beep_Init();
    gpio_init(P20_8, GPO, 0, PUSHPULL);//设置P20_8为输出 默认输出低电平  PUSHPULL：推挽输出
    gpio_init(P20_9, GPO, 0, PUSHPULL);
    gpio_init(P21_4, GPO, 0, PUSHPULL);
    gpio_init(P10_6,GPI,0,PUSHPULL);//PULLUP
//    icm20602_init_spi();
}


















