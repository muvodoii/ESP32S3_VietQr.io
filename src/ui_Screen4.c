#include "ui.h"
#include <stdio.h>
#include <stdlib.h>
extern lv_obj_t *ui_Minute;
extern lv_obj_t *ui_Second;
static lv_timer_t * timer = NULL;
uint8_t current_minute = 0;
uint8_t current_second = 0;
extern void ui_Screen2_screen_init(void);
uint8_t foo = 0;
extern lv_obj_t * ui_Screen2;
extern uint32_t tim1;
extern uint32_t tim2;
extern uint32_t tim3;
extern uint32_t tim4; 
extern lv_timer_t * my_timer;
uint8_t eventSelect = 0;
static void timer_callback(lv_timer_t * timer) {
    if (!foo)
        return;
    if ((!current_minute) && (!current_second)) {
        switch (eventSelect) {
            case 2:
                current_minute = tim1;
                break;
            case 3:
                current_minute = tim2;
                break;
            case 4:
                current_minute = tim3;
                break;
            case 5:
                current_minute = tim4;
                break;
            default:
                current_minute = 0;  
                break;
        }
        current_second = 59;
    }
    current_second--;
    
   if (current_second == 0 && current_minute) {
        current_minute--;
        current_second = 59;
   }
    if (current_minute == 0 && current_second == 0) {
//     
        foo = 0;
      lv_label_set_text(ui_Minute,"00");
      lv_label_set_text(ui_Second,"00");
      lv_disp_load_scr(ui_Screen2);
    }
 
    char minute_str[3];
    char second_str[3];
    sprintf(minute_str, "%02d", current_minute);
    sprintf(second_str, "%02d", current_second);
    lv_label_set_text(ui_Minute, minute_str);  // Update minute label
    lv_label_set_text(ui_Second, second_str);  // Update second label
}


void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
      
    ui_btnfn = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_btnfn, 77);
    lv_obj_set_height(ui_btnfn, 26);
    lv_obj_set_x(ui_btnfn, 104);
    lv_obj_set_y(ui_btnfn, 116);
    lv_obj_set_align(ui_btnfn, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_btnfn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_btnfn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_btnfn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_btnfn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image3 = lv_img_create(ui_Screen4);
    lv_img_set_src(ui_Image3, &ui_img_a1_png);
    lv_obj_set_width(ui_Image3, LV_SIZE_CONTENT);   /// 480
    lv_obj_set_height(ui_Image3, LV_SIZE_CONTENT);    /// 272
    lv_obj_set_align(ui_Image3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Minute = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Minute, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Minute, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Minute, -68);
    lv_obj_set_y(ui_Minute, -27);
    lv_obj_set_align(ui_Minute, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_Minute, lv_color_hex(0xF90808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Minute, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Minute, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Minute, &lv_font_montserrat_44, LV_PART_MAIN | LV_STATE_DEFAULT);
  my_timer =   lv_timer_create(timer_callback, 1000, NULL);
   
    ui_Second = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Second, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Second, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Second, 62);
    lv_obj_set_y(ui_Second, -23);
    lv_obj_set_align(ui_Second, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(ui_Second, lv_color_hex(0xF20A0A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Second, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Second, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Second, &lv_font_montserrat_44, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_btnfn, ui_event_btnfn, LV_EVENT_ALL, NULL);
     lv_label_set_text(ui_Second,"00");
 
}
