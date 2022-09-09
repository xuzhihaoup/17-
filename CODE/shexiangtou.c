#include "shexiangtou.h"
#include "zf_stm_systick.h"
#include "math.h"
#include "headfile.h"

uint8 yasuo[HIGH][WIDTH];    //����ѹ��ͼ��
uint8 bin_image[HIGH][WIDTH];//�����ֵͼ��
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
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < hang; j++)
    {
        for (i = 0; i < lie; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
        PixelFore = Amount - PixelBack;           //�������ص���
        OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
        OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
        PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
        MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}
/*---------------------------------------------------------------
 ����    ����yasuo_image
 ����    �ܡ�ͼ�����ѹ��    ѹ����ͼ���С  60*94
 ����    ������
 ���� �� ֵ����
 ��ע�����
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
 ����    ����Gaosi_Image_Filter
 ����    �ܡ���˹�˲�
 ����    ������
 ���� �� ֵ����
 ��ע�����
 ----------------------------------------------------------------*/
void Gaosi_Image_Filter()
{
    /*     �������                                      ת��
           0.0761245  0.132451  0.0761245                 0.0761245        0.132451        0.0761245
           0.132451   0.21043   0.132451                  0.132451         0.21043         0.132451
           0.0761245  0.132451  0.0761245                 0.0761245        0.132451        0.0761245
    */
        uint8  conv2[64][98],conv_out_image[62][96];; //�����
        int i =0,j=0,con_i = 0,con_j = 0 , y = 0, x = 0;
        //ͼ����������
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
        //������
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
         //������
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
 ����    ����Threshold_way
 ����    �ܡ���ֵ����   ��ֵ��
 ����    ����ģʽ  0��  Prewitt ��Ե���        1�����    2��sobel ��Ե���      3���ֲ���ֵ
 ���� �� ֵ����
 ��ע����� 2022��3��25
 255��  0 ��  1��  [MT9V03X_H][MT9V03X_W]
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
/*********�����ֵ�ָ���Ż���********************/
//�������ͣ��� �� ͼ��ָ�� ��ʼ��  ��ʼ�� �����д�С  �����д�С
#define GrayScale 256   //frame�Ҷȼ�
typedef unsigned char uchar;
int pixel[256]={0};  //�����־λ���ܶ����ں����ڲ���
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

    //ͳ��ÿ���Ҷȼ������صĸ���
    for(i = 0; i < height; i++)
    {
        for(j = 0;j < width;j++)
        {
            pixel[(int)Image[i * width + j]]++;
        }
    }

        for(k=0;k<GrayScale;k++)
        {
            sum_gray+=k*pixel[k];//�Ҷ�ֱ��ͼ������
            sum_pix_num+=pixel[k];//�����ظ���
        }

        for(k=0;k<GrayScale-1;k++)
        {
            pl_pix_num+=pixel[k];//��һ�������ظ���
            p2_pix_mum=sum_pix_num-pl_pix_num;//�ڶ��������ظ���
            p1_sum_gray+=k*pixel[k];   //��һ����������
            m1=(float)p1_sum_gray/pl_pix_num;//��һ���ֻҶȾ�ֵ
            m2=(float)(sum_gray-p1_sum_gray)/p2_pix_mum;//�ڶ����ֻҶȾ�ֵ

            V=pl_pix_num*p2_pix_mum*(m1-m2)*(m1-m2);

            if(V>variance)//����䷽��ϴ�ʱ�ĻҶ�ֵ��Ϊ��ֵ
            {
                variance=V;
                threshold1=k;
            }
        }
        BlackThres=threshold1;

}
/*---------------------------------------------------------------
 ����    ����PrewittAutoThreshold
 ����    �ܡ�Prewitt���ӱ�Ե���
 ����    ��������ͼ������
 ���� �� ֵ����
 ��ע����� 2022��3��25
 255��  0 ��  1��  [MT9V03X_H][MT9V03X_W]
 ----------------------------------------------------------------*/
void PrewittAutoThreshold (uint8 imageIn[HIGH][WIDTH], uint8 imageOut[HIGH][WIDTH])
{
    /** ����˴�С */
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
            /* ���㲻ͬ�����ݶȷ�ֵ  */
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

            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  */
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
 ����    ����sobelAutoThreshold
 ����    �ܡ�sobel���ӱ�Ե���
 ����    ������
 ���� �� ֵ����
 ��ע����� 2022��3��25
             ͼ��ѹ�������
             �����ͼ���С����Ϊ   60*94   ��Ϊ����㱻�޶�  ������ľ���㡢X�㡢Y���СΪ ԭͼH + 4     ԭͼW + 4
       255��  0 ��  1��
 ----------------------------------------------------------------*/
float conv_image[60][94];
uint8 image_conv[60][94];
uint32 MAX = 0,MIN = 1;
short yuzhi;
void sobelAutoThreshold()
{



/*     sobel�������
       -1  0  1                                  1  2  1
       -2  0  2                                  0  0  0
       -1  0  1                                 -1 -2 -1
*/
/*     sobel�������ת��
        1  0  -1                                -1 -2 -1
        2  0  -2                                 0  0  0
        1  0  -1                                 1  2  1
*/
    uint8  conv2[64][98]; //�����
    float conv_x[62][96];//Xά��
    float conv_y[62][96];//Yά��
    float SobelThreshold;
    float sum;
    int ss = 0;
    int i =0,j=0,con_i = 0,con_j = 0,x = 0,y = 0,xx = 0,yy = 0;

    MAX = 0; MIN = 0;
    //ͼ����������
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
////������
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
////ͼ�������  �����С��ԭͼ���С��ͬ���Ҷ�ֵ��   255��  0 ��
    //ͼ�������  �����С��ԭͼ���С��ͬ���Ҷ�ֵ��   255��  0 ��

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
 ����    ����part_AutoThreshold
 ����    �ܡ��ֲ���ֵ
 ����    ��������ͼ������
 ���� �� ֵ����
 ��ע����� 2022��3��30
 ��ѹ��ͼƬ��Ϊ 20������ ��ÿ�����������ֵ����
 255��  0 ��  1��  [MT9V03X_H][MT9V03X_W]
 ----------------------------------------------------------------*/
uint8  part_image_first[6][47];  //��һ������ͼ��
uint8  part_image_sec[6][47];      //�ڶ�������ͼ��
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
           if(image_flag == 6)  //����  6*47 ��С������ɼ����     60 * 94 ��С��ͼ�� ��Ϊ��20������
           {
               //����������ֵ
                MyOSTU(47,6,part_image_first);
               first_Thres = BlackThres;
               MyOSTU(47,6,part_image_sec);
               sec_Thres   = BlackThres;
               //�����������ֵ��
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





//ͼ���ӡ
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

            temp = *(p+j*width+i*width/coord_x);//��ȡ���ص�

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
 ����    ����auto_threshold
 ����    �ܡ������Ե�����ֵ
 ����    ������
 ���� �� ֵ����
 ��ע����� 2022��3��26
       255��  0 ��  1��
 ----------------------------------------------------------------*/
int class_num;

char buf3[64];
int image_class[60] = {0};
float auto_threshold(float (*Image)[94])
{
   // systick_start(STM0);
    int i = 0, j =0,class_flag = 0 ,class_max = 0,class_max_flag = 0;
    float qujian = 0.35 ,Threshold = 0;  //0.57 ʹ��excle��ϵõ���������
    for(int x = 0;x<60;x++)
        image_class[x] = 0;
    MAX = 0,MIN = 1;
   //Ѱ�����ֵ ����Сֵ
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
    //�ҵ��ж�����ֵ
    class_num =(int) ((MAX-MIN) / qujian);
    //�ҵ�ÿ�������ж��ٸ�
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
    //�ҳ��Ǹ������������
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
    //�û��ڴ˴����ø��ֳ�ʼ��������
    mt9v03x_init();     //����ͷ��ʼ��
    uart_init(UART_0,115200 , UART0_TX_P14_0, UART0_RX_P14_1);//���ڳ�ʼ��
    gtm_pwm_init(ATOM0_CH5_P02_5,17*1000,0);//�����ʼ��ATOM0_CH3_P10_6
    gtm_pwm_init(ATOM0_CH4_P02_4,17*1000,0);
    gtm_pwm_init(ATOM0_CH7_P02_7,17*1000,0);//�����ʼ��ATOM0_CH3_P10_6
    gtm_pwm_init(ATOM0_CH6_P02_6,17*1000,0);
   pit_interrupt_ms(CCU6_0, PIT_CH0, 10);

    gtm_pwm_init(ATOM0_CH1_P33_9,50,810);  //�����ʼ��      1060      980    900     725       810     895
    gpt12_init(GPT12_T5,GPT12_T5INB_P10_3,GPT12_T5EUDB_P10_1);      //���������ʼ��
    gpt12_init(GPT12_T6,GPT12_T6INA_P20_3,GPT12_T6EUDA_P20_0);      //�ұ�������ʼ��
   //adc_init(ADC_0, ADC0_CH8_A8);           //ADC���ص�ѹ
    Key_Init();           //������ʼ��
    pid_left_init();
    pid_right_init();
    Beep_Init();
    gpio_init(P20_8, GPO, 0, PUSHPULL);//����P20_8Ϊ��� Ĭ������͵�ƽ  PUSHPULL���������
    gpio_init(P20_9, GPO, 0, PUSHPULL);
    gpio_init(P21_4, GPO, 0, PUSHPULL);
    gpio_init(P10_6,GPI,0,PUSHPULL);//PULLUP
//    icm20602_init_spi();
}


















