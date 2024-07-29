/*******************************************************************************
 * LVGL Widgets
 * This is a widgets demo for LVGL - Light and Versatile Graphics Library
 * import from: https://github.com/lvgl/lv_demos.git
 *
 * Dependent libraries:
 * LVGL: https://github.com/lvgl/lvgl.git

 * Touch libraries:
 * FT6X36: https://github.com/strange-v/FT6X36.git
 * GT911: https://github.com/TAMCTec/gt911-arduino.git
 * XPT2046: https://github.com/PaulStoffregen/XPT2046_Touchscreen.git
 *
 * LVGL Configuration file:
 * Copy your_arduino_path/libraries/lvgl/lv_conf_template.h
 * to your_arduino_path/libraries/lv_conf.h
 * Then find and set:
 * #define LV_COLOR_DEPTH     16
 * #define LV_TICK_CUSTOM     1
 *
 * For SPI display set color swap can be faster, parallel screen don't set!
 * #define LV_COLOR_16_SWAP   1
 *
 * Optional: Show CPU usage and FPS count
 * #define LV_USE_PERF_MONITOR 1
 ******************************************************************************/

#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <ui.h>
#include <stdio.h>
#include <Arduino.h>
#include "RTClib.h"
#include <string.h>
#include <string>

#include <WiFi.h>

#include "virtual_eeprom.h"
// #include "mqttService.h"
// #include "epromService.h"
// #include "ntp.h"
// #include "global.h"
#define TIMEOUT 20000
lv_obj_t * qrCode;
const int coi = 35;
char ssid[SSID_MAX_LENGTH];
char pwd[PWD_MAX_LENGTH];

//SystemConfig systemManager;

//HardwareSerial lcdPort(1);

extern lv_obj_t *ui_Screen3;




extern lv_obj_t *ui_tim1;
extern lv_obj_t *ui_tim2;
extern lv_obj_t *ui_tim3;
extern lv_obj_t *ui_tim4;
extern lv_obj_t *ui_Screen3;



char temp1[50];
char temp2[50];
char temp3[50];
char temp4[50]; 



uint32_t fn;
uint32_t tim1;
uint32_t tim2;
uint32_t tim3;
uint32_t tim4; 

static uint32_t tick1 = 0, tick2 = 0, tick3 = 0;
static uint32_t systick_timr = 0;
extern lv_obj_t *ui_Label1;
extern lv_obj_t *ui_Label2;
extern uint8_t gio, phut, giay;
extern uint8_t setBuzzer;

String engineer;
String DataQR;

RTC_PCF8563 rtc;
// extern lv_obj_t *ui_Screen4;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// void ui_Screen4_screen_init(void); // Khai báo nguyên mẫu hàm ui_Screen4_screen_init()

#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

/* More dev device declaration: https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration */
/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */

#define GFX_BL 44
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);

// Uncomment for ST7262 IPS LCD 800x480
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    480 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 8 /* hsync_back_porch */,
    272 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 8 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

/*******************************************************************************
 * Please config the touch panel in touch.h
 ******************************************************************************/
#include "touch.h"
#include "ui.h"

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

///
// WiFiClient wifiClient;
// PubSubClient mqttClient(wifiClient);
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}


int HandleData (const char *var,uint32_t value  ) {
  Serial.println(var); 
  if (!strncmp(var, "tim1", strlen("tim1"))) {
    tim1 = value;

    Serial.println("User request change tim1 value");
    return eeprom_write_data(VARIABLE1_ADDR, (uint8_t *)&tim1, sizeof(uint32_t));

  } else if (!strncmp(var, "tim2", strlen("tim2"))) {
    tim2 = value;


    Serial.println("User request change tim2 value");
    return eeprom_write_data(VARIABLE2_ADDR, (uint8_t *)&tim2, sizeof(uint32_t));
  } else if (!strncmp(var, "tim3", strlen("tim3"))) {
    tim3 = value;
    Serial.println("User request change tim3 value");
    return eeprom_write_data(VARIABLE3_ADDR, (uint8_t *)&tim3, sizeof(uint32_t));
  } else if (!strncmp(var, "tim4", strlen("tim4"))) {
    tim4 = value;
    Serial.println("User request change tim4 value");
    return eeprom_write_data(VARIABLE4_ADDR, (uint8_t *)&tim4, sizeof(uint32_t));
  } else {
    Serial.println("Command not found!");
    return -1;
  }
}


static int DecodeSerial(const char *cmd) {
  char var[10] = {};
  uint32_t value = 0;
  if (sscanf(cmd, "%s %d", var, &value) == 0)
    return -1;
  return HandleData(var, value);
}
static void handler() {
  engineer = Serial.readString();
  Serial.print("Read uart: ");
  Serial.println(engineer);
  DecodeSerial(&engineer[0]);

}

void handler1() {
    if (Serial.available() > 0) {
        String strData = Serial.readString();
        if (strData.startsWith("QR:")) {
        
            String qrData = strData.substring(3); 

            const char* dataPointer = qrData.c_str(); 
            Serial.print("String from Serial: ");
            Serial.println(dataPointer);

            lv_obj_t * qr = lv_qrcode_create(ui_Screen3, 200, lv_color_hex3(0x000), lv_color_hex3(0xeef));
            lv_qrcode_update(qr, (const uint8_t*)dataPointer, qrData.length());

            
            // lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
        } else {
            Serial.println("Error Command!Let's start with'QR:'");
        }
    }
}
void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(100);


  uint32_t start_tim;
  
  pinMode(coi, OUTPUT);
  eeprom_init_memory();
  String message;
  memset(ssid, 0, sizeof(ssid));
  memset(pwd, 0, sizeof(pwd));
  Serial.print("Variable1: ");
  Serial.println(EEPROM.readUInt(VARIABLE1_ADDR)); 
  Serial.print("Variable2: ");
  Serial.println(EEPROM.readUInt(VARIABLE2_ADDR));
  Serial.print("Variable3: ");
  Serial.println(EEPROM.readUInt(VARIABLE3_ADDR));
  Serial.print("Variable4: ");
  Serial.println(EEPROM.readUInt(VARIABLE4_ADDR));
/////////// WiFi Setup
// reconnect:

//   if (!eeprom_read_wifi(ssid, sizeof(ssid), pwd, sizeof(pwd)))
//   {
//     Serial.print("Enter SSID + <space> + Password: ");

//     while(!Serial.available())
//       ;
//     message = Serial.readString();
//     if (!eeprom_write_wifi(&message[0]))
//     {
//       Serial.println("Store wifi information in eeprom failed!");
//     }
//     else
//     {
//       Serial.print("Store wifi information in eeprom successfully!");
//       Serial.println(EEPROM.readString(EEPROM_BASE_ADDR));
//     }
//   }
//   else
//   {

//     Serial.print("Connect to Wi-Fi SSID: ");
//     Serial.print(ssid);
//     Serial.print(" Password: ");
//     Serial.println(pwd);

//     start_tim = millis();
    
//     //WiFi.begin(ssid, pwd);
//     if (WiFi.status() != WL_CONNECTED)
//     {
//       if ((millis() - start_tim) > TIMEOUT)
//       {
//         Serial.println("SSID or PASSWORD is incorrect!");
//         eeprom_erase_memory(EEPROM_BASE_ADDR, SSID_MAX_LENGTH + PWD_MAX_LENGTH);
//         goto reconnect;
//       }
//    }
//     Serial.println("WiFi connected!");
//     Serial.print("IP address: ");
//     Serial.println(WiFi.localIP());
//   }
  //////
  //// modify variable from Serial Port
 // Serial.onReceive(handler, true);

  touch_init(gfx->width(), gfx->height());

  // Init Display
  gfx->begin();
  gfx->fillScreen(BLACK);

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * 10);
#endif
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * 10);

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
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();

    Serial.println("Setup done");

    if (!rtc.begin())
    {
      Serial.println("Couldn't find RTC");
      Serial.flush();
    }

    if (rtc.lostPower())
    {
      Serial.println("RTC is NOT initialized, let's set the tim!");

      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    rtc.start();
 }
 
}


String foo_;

void loop()
{
//readSerialData();

  if (Serial.available()) {
  //   handler();
    handler1();

   Serial.println("After changing:");
  Serial.print("Variable1: ");
  Serial.println(EEPROM.readUInt(VARIABLE1_ADDR)); 
  Serial.print("Variable2: ");
  Serial.println(EEPROM.readUInt(VARIABLE2_ADDR));
  Serial.print("Variable3: ");
  Serial.println(EEPROM.readUInt(VARIABLE3_ADDR));
  Serial.print("Variable4: ");
  Serial.println(EEPROM.readUInt(VARIABLE4_ADDR));

  sprintf(temp1, "%02d",  tim1);
  lv_label_set_text(ui_tim1, temp1);
  sprintf(temp2, "%02d",  tim2);
  lv_label_set_text(ui_tim2, temp2);
  sprintf(temp3, "%02d",  tim3);
  lv_label_set_text(ui_tim3, temp3);
  sprintf(temp4, "%02d",  tim4);
  lv_label_set_text(ui_tim4, temp4);
     
  }
  systick_timr = millis();
  if (systick_timr - tick1 > 5)
  {
    tick1 = systick_timr;
    lv_timer_handler();
    buzzer_action();
  }
  if (systick_timr - tick2 > 1000)
  {
    tick2 = systick_timr;

    DateTime now = rtc.now();

    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    gio = now.hour();
    phut = now.minute();
    giay = now.second();
  }


}

void buzzer_action(void)
{
  if (setBuzzer == 1)
  {
    digitalWrite(35, HIGH);
    delay(50);
    digitalWrite(35, LOW);
    setBuzzer = 0;
  }
}

