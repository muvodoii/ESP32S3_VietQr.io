/***********************************************************************************
*This program is a demo of drawing
*This demo was made for LCD modules with 8bit port.
*This program requires the the LCDKIWI library.

* File                : touch_pen.ino
* Hardware Environment: ESP32
* Build Environment   : Arduino ESP32

*Set the pins to the correct ones for your development shield or breakout board.
*This demo use the BREAKOUT BOARD only and use these 8bit data lines to the LCD,
*pin usage as follow:
*            LCD_CS  LCD_RS  LCD_WR  LCD_RD  LCD_RST  TP_CS  TP_CLK  TP_MOSI  TP_MISO  TP_PEN   TE
*     ESP32    33      15      4       2       32      22      18      23       19       21     35                                        

*            LCD_D0  LCD_D1  LCD_D2  LCD_D3  LCD_D4  LCD_D5  LCD_D6  LCD_D7  
*     ESP32    12      13      26      25      17      16      27      14

*Remember to set the pins to suit your display module!
*
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**********************************************************************************/

#include <lcd_lib/LCDWIKI_GUI.h> //Core graphics library
#include <lcd_lib/LCDWIKI_KBV.h> //Hardware-specific library
#include <lcd_lib/ft5426.h> //touch library
#include"lvgl/lvgl.h"

#include "ui_files/ui.h"

#define OTT_MAX_TOUCH  5 
#define INT            21
#define CRST           22
#define SCL            18
#define SDA            23

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV my_lcd(SSD1963,33,15,4,2,32); //model,cs,cd,wr,rd,reset


FT5426 my_tp(INT,CRST,SCL,SDA);

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;
static lv_color_t *buf1;
static lv_color_t *buf2;
static lv_obj_t * label1;

void lv_example_btn(void)
{
    label1 = lv_label_create(lv_scr_act());          /*Add a label to the button*/
    lv_label_set_text(label1, "nsdkjnvsdnjvdsvmvdmvmdvmdv");                     /*Set the labels text*/
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, +50 );

}

uint16_t color_mask[] = {0xF800,0xFFE0,0x07E0,0x07FF,0x001F}; //color select

void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_map) {
  // Vẽ ra màn hình LCDWIKI_KBV
  int32_t x1 = area->x1;
  int32_t y1 = area->y1;
  int32_t x2 = area->x2;
  int32_t y2 = area->y2;

  // Gửi dữ liệu màu từ LVGL đến màn hình LCDWIKI_KBV
  my_lcd.Draw_Bit_Map(x1, y1, x2 - x1 + 1, y2 - y1 + 1, (uint16_t *) color_map,1);
  
  lv_disp_flush_ready(disp);
}

void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int16_t x, y;

    my_tp.FT5426_Scan();
    
    if(my_tp.ctp_status&(0x01))
     {
        data->point.x = my_tp.x[0];
        data->point.y = my_tp.y[0];
        data->state = LV_INDEV_STATE_PR;  // Chạm
        printf("touchpad_read lvgl touch \n\r");
    } 
    else 
    {
        data->state = LV_INDEV_STATE_REL;  // Không chạm
    }
}


lv_obj_t * ui_Label1;
void setup(void) 
{
  Serial.begin(115200);
  my_lcd.Set_Rotation(1);  
  my_lcd.Init_LCD();
  my_tp.FT5426_Init(my_lcd.Get_Rotation(),my_lcd.Get_Display_Width(),my_lcd.Get_Display_Height());
 
  screenWidth = my_lcd.Get_Width();
  screenHeight = my_lcd.Get_Height();
  
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  lv_init();
  
  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_drv);


    ui_init();
    lv_example_btn();
    

  // Tạo giao diện LVGL

  
}

uint8_t j=0;
void loop()
{
  //j++;
  lv_task_handler();
  // delay(5);
  // my_lcd.Set_Draw_color(GREEN);
  
  // my_lcd.Fill_Rectangle(100, 50, 200, 600);
 
  
  //  delay(1000);
  // my_lcd.Set_Draw_color(RED);
  // my_lcd.Fill_Rectangle(100, 50, 200, 600);

  // delay(1000);

  // my_tp.FT5426_Scan();
    
  // if(my_tp.ctp_status&(0x01))
  //   {
  //     //remove second touch
  //     if(j>1)
  //     {
  //        j=0;  
  //     }
  //     else
  //     {
  //     Serial.print("\n\rtouch:  ");
  //     Serial.print(my_tp.x[0]);  Serial.print("  ");
  //     Serial.print(my_tp.y[0]);
  //     }
  //   }
}