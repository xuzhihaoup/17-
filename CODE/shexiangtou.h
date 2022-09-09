#ifndef _shexiangtou_h
#define _shexiangtou_h
#include "headfile.h"
#define HIGH    MT9V03X_H/2
#define WIDTH   MT9V03X_W/2



extern uint8  part_image_first[6][47];  //第一个区域图像
extern uint8  part_image_sec[6][47];      //第二个区域图像
extern uint8 bin_image[HIGH][WIDTH];
extern uint8 yasuo[HIGH][WIDTH];
extern uint8 mid_head_line[60];
extern uint8 xianshi[HIGH][WIDTH];
extern uint8 find_line_image[HIGH][WIDTH];
extern  float BlackThres;
void lcd_set_region(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end);
void lcd_writedata_16bit(uint16 dat);
void lcd_displayimage(uint8 *p, uint16 width, uint16 height);
void yasuo_image(uint8 * p);
void lcd_drawpoint(uint16 x,uint16 y,uint16 color);
void lcd_drawpoint(uint16 x,uint16 y,uint16 color);
void Init(void);
void MyOSTU(int width,int height,uint8 *Image);
void PrewittAutoThreshold (uint8 imageIn[HIGH][WIDTH], uint8 imageOut[HIGH][WIDTH]);
void sobelAutoThreshold();
void  Threshold_way(uint8 Threshold_mode);
float auto_threshold(float (*Image)[94]);
short GetOSTU (unsigned char tmImage[HIGH][WIDTH],uint8 hang,uint8 lie);
void  part_AutoThreshold();
void Gaosi_Image_Filter();

#endif
