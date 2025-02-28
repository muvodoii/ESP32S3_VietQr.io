// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"
extern uint8_t TotalTimeinSec;
void ui_Screen2_screen_init(void)
{

    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Button2_ = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Button2_, 159);
    lv_obj_set_height(ui_Button2_, 28);
    lv_obj_set_x(ui_Button2_, -108);
    lv_obj_set_y(ui_Button2_, -35);
    lv_obj_set_align(ui_Button2_, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button2_, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button2_, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button2_, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button2_, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button3_ = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Button3_, 146);
    lv_obj_set_height(ui_Button3_, 34);
    lv_obj_set_x(ui_Button3_, -107);
    lv_obj_set_y(ui_Button3_, 42);
    lv_obj_set_align(ui_Button3_, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button3_, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button3_, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button3_, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button3_, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button3_, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Button3_, lv_color_hex(0xFBF6F6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button3_, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button3_, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button3_, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_Button3_, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button4_ = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Button4_, 155);
    lv_obj_set_height(ui_Button4_, 33);
    lv_obj_set_x(ui_Button4_, 131);
    lv_obj_set_y(ui_Button4_, -33);
    lv_obj_set_align(ui_Button4_, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button4_, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button4_, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button4_, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button4_, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button5_ = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Button5_, 157);
    lv_obj_set_height(ui_Button5_, 31);
    lv_obj_set_x(ui_Button5_, 135);
    lv_obj_set_y(ui_Button5_, 42);
    lv_obj_set_align(ui_Button5_, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button5_, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button5_, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button5_, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button5_, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button6_ = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Button6_, 68);
    lv_obj_set_height(ui_Button6_, 44);
    lv_obj_set_x(ui_Button6_, 1);
    lv_obj_set_y(ui_Button6_, 105);
    lv_obj_set_align(ui_Button6_, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button6_, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button6_, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button6_, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button6_, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image2 = lv_img_create(ui_Screen2);
    lv_img_set_src(ui_Image2, &ui_img_2_png);
    lv_obj_set_width(ui_Image2, LV_SIZE_CONTENT);   /// 480
    lv_obj_set_height(ui_Image2, LV_SIZE_CONTENT);    /// 272
    lv_obj_set_x(ui_Image2, 1);
    lv_obj_set_y(ui_Image2, 1);
    lv_obj_set_align(ui_Image2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_valueabc = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_valueabc, 91);
    lv_obj_set_height(ui_valueabc, 27);
    lv_obj_set_x(ui_valueabc, -166);
    lv_obj_set_y(ui_valueabc, -26);
    lv_obj_set_align(ui_valueabc, LV_ALIGN_CENTER);
    lv_label_set_text(ui_valueabc, "10000");
    lv_obj_set_style_text_color(ui_valueabc, lv_color_hex(0xF3F702), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_valueabc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_valueabc, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_valueabc, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_valueabc, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_valueabc, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_valueabc, lv_color_hex(0xD10808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_valueabc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_value2 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_value2, 64);
    lv_obj_set_height(ui_value2, 28);
    lv_obj_set_x(ui_value2, 72);
    lv_obj_set_y(ui_value2, -26);
    lv_obj_set_align(ui_value2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_value2, "50000\n");
    lv_obj_set_style_text_color(ui_value2, lv_color_hex(0xF5F208), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_value2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_value2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_value2, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_value4_ = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_value4_, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_value4_, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_value4_, -164);
    lv_obj_set_y(ui_value4_, 46);
    lv_obj_set_align(ui_value4_, LV_ALIGN_CENTER);
    lv_label_set_text(ui_value4_, "100000 ");
    lv_obj_set_style_text_color(ui_value4_, lv_color_hex(0xF6E008), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_value4_, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_value4_, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_value4_, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_value3 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_value3, 91);
    lv_obj_set_height(ui_value3, 27);
    lv_obj_set_x(ui_value3, 72);
    lv_obj_set_y(ui_value3, 50);
    lv_obj_set_align(ui_value3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_value3, "500000");
    lv_obj_set_style_text_color(ui_value3, lv_color_hex(0xEBFB07), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_value3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_value3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_value3, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tim1 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_tim1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tim1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tim1, -97);
    lv_obj_set_y(ui_tim1, -31);
    lv_obj_set_align(ui_tim1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tim1, "3");
    lv_obj_set_style_text_color(ui_tim1, lv_color_hex(0x0007B6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tim1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tim1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tim1, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tim3 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_tim3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tim3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tim3, -94);
    lv_obj_set_y(ui_tim3, 44);
    lv_obj_set_align(ui_tim3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tim3, "10 ");
    lv_obj_set_style_text_color(ui_tim3, lv_color_hex(0x2D009D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tim3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tim3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tim3, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tim2 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_tim2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tim2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tim2, 143);
    lv_obj_set_y(ui_tim2, -32);
    lv_obj_set_align(ui_tim2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tim2, "5");
    lv_obj_set_style_text_color(ui_tim2, lv_color_hex(0x1500AD), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tim2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tim2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tim2, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tim4 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_tim4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tim4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tim4, 141);
    lv_obj_set_y(ui_tim4, 44);
    lv_obj_set_align(ui_tim4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tim4, "20");
    lv_obj_set_style_text_color(ui_tim4, lv_color_hex(0x000BA6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tim4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tim4, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tim4, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button2_, ui_event_Button2_, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button3_, ui_event_Button3_, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button4_, ui_event_Button4_, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button5_, ui_event_Button5_, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button6_, ui_event_Button6_, LV_EVENT_ALL, NULL);
   

}
