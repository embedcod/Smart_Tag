#ifndef _EPD_GUI_H_
#define _EPD_GUI_H_

typedef struct {
  uint8_t *Image;
  uint16_t Width;
  uint16_t Height;
  uint16_t WidthMemory;
  uint16_t HeightMemory;
  uint16_t Color;
  uint16_t Rotate;
  uint16_t WidthByte;
  uint16_t HeightByte;
} PAINT;
extern PAINT Paint;

#define ROTATE_0            0 
#define ROTATE_90           90  
#define ROTATE_180          180 
#define ROTATE_270          270 

#define WHITE          0xFF   
#define BLACK          0x00   

void Paint_NewImage(uint8_t *image, uint16_t Width, uint16_t Height, uint16_t Rotate, uint16_t Color);        //创建画布控制显示方向
void Paint_SetPixel(uint16_t Xpoint, uint16_t Ypoint, uint16_t Color);
void EPD_Full(uint8_t Color);
void EPD_DrawLine(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t Color);
void EPD_DrawRectangle(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t Color, uint8_t mode); //画矩形
void EPD_DrawCircle(uint16_t X_Center, uint16_t Y_Center, uint16_t Radius, uint16_t Color, uint8_t mode);    //画圆
void EPD_ShowChar(uint16_t x, uint16_t y, uint16_t chr, uint16_t size1, uint16_t color);                     //显示字符
void EPD_ShowString(uint16_t x, uint16_t y, const char *chr, uint16_t size1, uint16_t color);                   //显示字符串
void EPD_ShowNum(uint16_t x, uint16_t y, uint32_t num, uint16_t len, uint16_t size1, uint16_t color);             //显示数字
void EPD_ShowPicture(uint16_t x, uint16_t y, uint16_t sizex, uint16_t sizey, const uint8_t BMP[], uint16_t Color); //显示图片
void EPD_ClearWindows(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);
void EPD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t pre, uint8_t sizey, uint8_t color);
void EPD_ShowWatch(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t pre, uint8_t sizey, uint8_t color); //显示秒表

#endif
