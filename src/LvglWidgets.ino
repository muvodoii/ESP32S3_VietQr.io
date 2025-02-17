#include <Arduino.h>
#include "main.h"
#include "WiFi.h"
#include "time.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <lvgl.h>
#include <vector>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <TFT_eSPI.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <XPT2046_Touchscreen.h>
#include <pubsubClient.h>
#include "sys/time.h"
#include <stdlib.h>
#include <WiFi.h>
#include "./ui_files/ui.h"
#include "ArduinoJson.h"
#include "ClientHandler.h"
#include "Config.h"
#include "LcdInterface.h"
#include "stdio.h"

#include <TinyGSM.h>
#include <TinyGsmClient.h>


#define MQTT_LTE  true
#define MQTT_WIFI  false
#define TIMER_INTERVAL_US  100  // 10000us = 10ms
#define BUZZER_DURATION    100  // 

#define WIFI_MODE  0
#define LTE_MODE   1

extern uint8_t CLIENT_MODE;


static uint32_t moneyPublish = 0;
static uint32_t buzzerTick = 0;
static uint32_t successTick = 0;
static volatile uint16_t NumFulseGenerator = 0;
static volatile uint32_t buzzerCnt =0;

static TCPIPConnection_def TCPIPConnection;

static SemaphoreHandle_t mutex;


static char qrContent[QR_CONTENT_SIZE] = {0};
static uint8_t activeAccountOK = 0;
extern lv_obj_t * ui_Label2;

static uint32_t _orderId;

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);
uint16_t touchScreenMinimumX = 200, touchScreenMaximumX = 3700, touchScreenMinimumY = 240, touchScreenMaximumY = 3800;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * 10];
TaskHandle_t ntConnectTaskHandler;
TaskHandle_t ntTaskMqttClient;

static uint16_t convertFulseToMoney(uint16_t fulse);

char mqtt_buffer[1024];
bool flag = false;
lv_obj_t * qrCode = NULL;

/* define  mqtt client */

WiFiClient espClient;
PubSubClient wifi_client(espClient);


#if (MQTT_LTE)
TinyGsm modem(Serial1);
TinyGsmClient gsmClient(modem);
PubSubClient lte_client(gsmClient);

void ModemHandler_init();
void ModemHandler_connect_gprs();

#endif

PubSubClient mqttClient;

extern int32_t rssi;
extern int8_t ui_rssi_mode;
lv_event_t e;
uint32_t lastReconnectAttempt = 0;

unsigned long start;
static inline String time() {
    return "..." + String((millis() - start) / 1000) + 's';
}

static void log(String info) {
    Serial.println(info);
}

// CONNECT_STATUS_ENUM ConnectStatus = E_POWER_UP;
void getNTPTimer();
void setup_wifi();
void display_QR();
 void buttonClick(void);
void ui_event_Button11(lv_event_t * e);
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void Receive_handler();
const char* LayQR_Content(const char *data);
void setup();
static void timer_callback(lv_timer_t *timer);
void timeout_BankSuccess(uint16_t seconds);
void TrangThaiThanhToan();
void parseMessage(char* json);
void loop();
void buttonConfig_Handler(void);
void timerLVGL_Handler(void);
void lcd_init();
void fulse_generator(uint32_t amount);
static void convertMoneyToFulse(uint32_t amount);
void buzzerSetActive();
void ClientReportCash( void);
void create_spinner();
int32_t getWiFiStrengthAsPercentage(int32_t rssi);
void  test_hash_code(void);

static uint8_t _STATE_CREATE_QR_ACTION  = STATE_IDLE;
static uint8_t _STATE_FULSE_ACTION  = STATE_IDLE;

static volatile int MAX_TIMEOUT  = -1;
static volatile int MAX_TIMEOUT_SUCCESS  = -1;
static volatile boolean   ACTIVE_TIMER         = false;
static volatile boolean   ACTIVE_TIMER_SUCCESS = false;


static int genFulseAction = 0;
static int NumberFulse = 0;
static int startFulse  = 0;
static uint32_t bdsd_moneyValue = 0;

static boolean BLOCK_READING = false;

static volatile boolean buzzerActive =  false;

static volatile uint32_t buzzerDurationCnt = 0;
void screen2_timeout_handler(void);

void screenSucess_timeout_handler(void);

void fulse_read_nonBlocking(void);
void fulse_generate_nonBlocking(void);

static uint32_t TickTimerReadInput = 0;
static uint32_t TickModemHandler = 0;

static bool resetAccountAction = false;
//setting
//
void check_spinner_timeout();
void switch_to_screen1();
static bool spinner_active = false;
static bool qrcode_active = false;
static uint32_t spinner_start_time = 0; // Variable to store when the spinner starts
static uint32_t qrcode_start_time = 0;

void TaskConnectInternet(void *pvParameters);
void TaskMqttClient(void *pvParameters);
void networkConnector(uint32_t interval);
void startMqttClientTask();


hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer()
{
  if (buzzerActive)
  {
    if (buzzerCnt++ < 1000)
      digitalWrite(BUZZER_PIN, HIGH);
    else
    {
      buzzerCnt = 0;
      buzzerActive = false;
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
  
  //fulse_generate_nonBlocking();
}

void check_spinner_timeout() {
    if (spinner_active) {
        // Check if 10 seconds have passed
        if (lv_tick_get() - spinner_start_time >= 10000) {
            switch_to_screen1();
        }
    }

    // if(qrcode_active)
    // {
    //   if (lv_tick_get() - qrcode_start_time >= 10000) {
    //         switch_to_screen1();
    //     }
    // }
}
void switch_to_screen1() {
    if (spinner_active) {    
        lv_scr_load(ui_Screen1); 
        spinner_active = false;
    }
}


//
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  uint16_t *color565 = (uint16_t *)malloc(w * h * sizeof(uint16_t));
  for (uint32_t i = 0; i < w * h; i++) {
    color565[i] = lv_color_to16(color_p[i]);
  }
  tft.pushColors(color565, w * h, true);
  free(color565);

  tft.endWrite();

  lv_disp_flush_ready(disp);
}
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touchscreen.touched())
  {
    TS_Point p = touchscreen.getPoint();

    if (p.x < touchScreenMinimumX)
      touchScreenMinimumX = p.x;
    if (p.x > touchScreenMaximumX)
      touchScreenMaximumX = p.x;
    if (p.y < touchScreenMinimumY)
      touchScreenMinimumY = p.y;
    if (p.y > touchScreenMaximumY)
      touchScreenMaximumY = p.y;

    data->point.x = map(p.x, touchScreenMinimumX, touchScreenMaximumX, 1, SCREEN_WIDTH);
    data->point.y = map(p.y, touchScreenMinimumY, touchScreenMaximumY, 1, SCREEN_HEIGHT);
    data->state = LV_INDEV_STATE_PR;
 }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}


// uart receiver code //
void Receive_handler(){
  return;
}

const char* LayQR_Content(const char *data) {
    const char *key = "\"qrCode\":\"";
    const char *startPos = strstr(data, key);
    if (startPos == NULL) {
        Serial.println(" QRcode is NULL -> error respose from vietQR");
        return NULL;
    }
    startPos += strlen(key);
    const char *endPos = strchr(startPos, '"');
    if (endPos == NULL) {
        Serial.println("Không tìm thấy kết thúc của giá trị 'qrCode'.");
        return NULL;
    }
    int qrLength = endPos - startPos;

    if (qrLength < QR_CONTENT_SIZE) {
        strncpy(qrContent, startPos, qrLength);
        qrContent[qrLength] = '\0';
        return qrContent;
    } else {
        Serial.println("Chiều dài của giá trị 'qrCode' quá lớn.");
        return NULL;
    }
}


/**
 * @brief  setup function 
 * 
 */
void setup()
{ 
  TCPIPConnection.Connectionstate = MODEM_INIT;
  TCPIPConnection.Timeout = 100;
  TCPIPConnection.Tick = 0;

  CLIENT_MODE  = WIFI_MODE;
  ui_rssi_mode = CLIENT_MODE;


  if(CLIENT_MODE == LTE_MODE)
    mqttClient = lte_client;
  else if (CLIENT_MODE == WIFI_MODE)
    mqttClient = wifi_client;
  else;
  

  // init fulse //
  //pinMode(beePin, OUTPUT);          // buzzer is output //
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FULSE_OUT, OUTPUT);
  digitalWrite(FULSE_OUT, HIGH);
  pinMode(FULSE_INPUT, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  lcd_init();
  Serial.begin(115200);  // serial main -> for log //
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // uart for AT Command //

  My_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, TIMER_INTERVAL_US , true);
  timerAlarmEnable(My_timer); // Just Enable

  lv_task_handler();
  // init modem 4G //
  
    if(CLIENT_MODE == WIFI_MODE)
    {
      Serial.print(" wifi mode ...");
   WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
    }

  if(CLIENT_MODE == LTE_MODE)
  {
     Serial.print(" init modem..version 2 ->16/10/2024 \n\r");
     ModemHandler_init();
     ModemHandler_connect_gprs();
  }
    mqttClient.setServer(mqtt_broker, mqtt_port);
    mqttClient.setCallback(mqtt_callback);
    mqttClient.setBufferSize(1024);

    clientHandler.init();
    config.init();
   // load config //
    String setting = config.readSetting();
    printf(" loading setting %s\n\r", setting.c_str());

    Serial.println(" read back file config in the first bootup ");
    // if error --> set default value //


    Serial.println(config.readBankAccount().c_str());
    Serial.println(config.readbankCode().c_str());
    Serial.println(config.readUserBankName().c_str());


    while (!mqttClient.connected()) {
        String client_id = "esp32";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (mqttClient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public  broker connected");

            mqttClient.subscribe(clientHandler.getSyncBoxsTopic().c_str());

        } else {
            Serial.print("failed with state ");
            Serial.print(mqttClient.state());
            delay(2000);
        }
    }

  //lcd_init();
  
  // push the first packet for get sync //
  JSONVar payload;
  payload["macAddr"] = clientHandler.getMacAddress();
  payload["checkSum"] = clientHandler.calculateChecksum();
  Serial.println("checksum");
  Serial.println(JSON.stringify(payload).c_str());

  mqttClient.publish(clientHandler.sync_topic.c_str(), JSON.stringify(payload).c_str());
  Serial.println(" publish sync data in the fissrt time");

  getNTPTimer();
  // startMqttClientTask();
  test_hash_code();
}


typedef struct {
    uint16_t time_left;
} timer_data_t;


static void timer_callback(lv_timer_t *timer)
{
    timer_data_t *data = (timer_data_t *)timer->user_data;
    if (data->time_left > 0) {
        data->time_left--;
    }
    if (data->time_left == 0) {

        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);
        free(data);
        lv_timer_del(timer);
    }
}


void timeout_BankSuccess(uint16_t seconds)
{
    lv_timer_t *timer = lv_timer_create(timer_callback, 1000, NULL);
    timer_data_t *data = (timer_data_t *)malloc(sizeof(timer_data_t));
    if (data == NULL) {
        return;
    }
    data->time_left = seconds;
    timer->user_data = data;
}

 lv_obj_t * qr = NULL;  // qrcode object //
 const char* qrdata_rcv;

/*********************************************************************************************************************
 * @brief  mqtt cllback function
 * 
 * @param topic 
 * @param payload 
 * @param length 
 **********************************************************************************************************************/
 void mqtt_callback(char *topic, byte *payload, unsigned int length)
 {
   Serial.print("Message arrived in topic: ");
   Serial.println(topic);
   Serial.print("Message:");
   char message[length + 1];
   strncpy(message, (char *)payload, length);
   message[length] = '\0';

   Serial.println(message);

   String str = String((char *)payload);
   JSONVar jsondata = JSON.parse(str);

   if (strcmp(topic, (topic_qr).c_str()) == 0)
   {
     
     qrdata_rcv = LayQR_Content(message);
     Serial.println("Data QR Received:");
     Serial.println(qrdata_rcv);
     if (qrdata_rcv != NULL)
     {
       Serial.println("mqtt_callback -> QR is NULL");
     }
   }

   // topic bien dong so du
   if (strcmp(topic, (topic_bdsd).c_str()) == 0)
   {
     uint32_t amount =  (uint32_t)(static_cast<double>(jsondata["amount"]));
     printf(" amount bdsd = %d \n\r", amount);
     bdsd_moneyValue =  amount;
     genFulseAction = 1;
     qrcode_active = false;
     _STATE_FULSE_ACTION = STATE_FULSE_START;

    char money_buff[16];
    sprintf(money_buff, "%d VND", amount);
    lv_label_set_text(ui_moneyPay,(const char*) money_buff);
    
    _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen3_screen_init); 
    // timeout_BankSuccess(10);
   }
 
   // get certificate
   if (String(topic) == clientHandler.getSyncBoxsTopic())
   {

     clientHandler.setBoxId(static_cast<const char *>(jsondata["boxId"]));       // get BOX ID base 64
     clientHandler.setTerminalBox(static_cast<const char *>(jsondata["boxId"])); // get terminalBox after convert base64 to char

     clientHandler.setQrCertificate(static_cast<const char *>(jsondata["qrCertificate"])); // get terminalBox after convert base64 to char
     Serial.print("-->qrCertificate: ");
     Serial.println(clientHandler.getQrCertificate().c_str());

     String topic = qr_topic_prefix + clientHandler.getBoxId();
     Serial.println(topic.c_str()); // sync topic
     if (mqttClient.subscribe(topic.c_str(), 1))
     {
       Serial.print("sub sync Certificate: ");                 // In ra "Arduino"
       Serial.println(topic.c_str());                          // sync topic
       Serial.print("terminal box: ");                         // In ra "Arduino"
       Serial.println(clientHandler.getTerminalBox().c_str()); // box ID: VVB123456

       // store to config //
       config.writeQrCertificate(clientHandler.getQrCertificate().c_str());
       config.writeBoxId(clientHandler.getBoxId().c_str());

       // update all topic publish//
       topic_cash = topic_cash_prefix + clientHandler.getTerminalBox();
       topic_request = topic_request_prefix + clientHandler.getTerminalBox();
       topic_request = topic_request_prefix + clientHandler.getTerminalBox();
       topic_qr = topic_qr_prefix + clientHandler.getTerminalBox();
       topic_status = topic_status_prefix + clientHandler.getTerminalBox();
       topic_bdsd = topic_bdsd_prefix + clientHandler.getTerminalBox();
       // clientHandler.getBoxId().c_str();

       if (mqttClient.subscribe(topic_qr.c_str(), 1))
         printf(" sub topic_qr_prefix ok: %s \n\r", topic_qr.c_str());
       if (mqttClient.subscribe(topic_status.c_str(), 1))
         printf(" sub topic_status_prefix ok: %s \n\r", topic_status.c_str());
       if (mqttClient.subscribe(topic_bdsd.c_str(), 1))
         printf(" sub topic_bdsd_prefix ok: %s  \n\r", topic_bdsd.c_str());

        test_hash_code();

       lcd2_updateBoxId(clientHandler.getTerminalBox().c_str());

       return;
     }
   }

   // sync TerminalBox and BoxID
   if (jsondata.hasOwnProperty("notificationType"))
   {
     String bankAccount = static_cast<const char *>(jsondata["bankAccount"]);
     String bankCode = static_cast<const char *>(jsondata["bankCode"]);
     String userBankName = static_cast<const char *>(jsondata["userBankName"]);

     config.writeBankAccount(bankAccount.c_str());
     config.writebankCode(bankCode.c_str());
     config.writeUserBankName(userBankName.c_str());
     
    Serial.println(" read back file config");
     // store to config //
    resetAccountAction = false;
    
    // update Back Account + Bank user name //
    String bankAcc = config.readbankCode() + ": " + config.readBankAccount();
    lv_label_set_text(ui_bankCode, bankAcc.c_str());
    lv_label_set_text(ui_bankAccount, config.readUserBankName().c_str());

    Serial.println(config.readBankAccount().c_str());
    Serial.println(config.readbankCode().c_str());
    Serial.println(config.readUserBankName().c_str());

    activeAccountOK = 1;

  }
    Serial.println("---------------------------------------------------------");
 }

void ClientRequestQR(int money, const char* bankAccount, const char* bankCode, const char* userBankName )
{
  sprintf(mqtt_buffer, "{\"amount\":%d,\"content\":\"PaymentForOrder\",\"bankAccount\":\"%s\",\"bankCode\":\"%s\",\"userBankName\":\"%s\",\"transType\":\"C\",\"orderId\":\"VPB%s\",\"terminalCode\":\"%s\",\"serviceCode\":\"SVC001\",\"additionalData\":[{\"additionalData1\":\"%s\"}]}", \
  money, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str(), clientHandler.createRandomString().c_str(), clientHandler.getTerminalBox().c_str(), clientHandler.createRandomString().c_str());
  
       
       boolean status =  mqttClient.publish(topic_request.c_str(), mqtt_buffer, 1);
       if(status == true)
       {
          _orderId++;
          printf("topic request pub: %s \n\r", topic_request.c_str());
          printf("data: %s \n\r", mqtt_buffer);
       }
       else
          printf("Pub Failure \n\r");
}

void ClientReportCash( int cashValue)
{
  sprintf(mqtt_buffer, "{\"amount\":%lu,\"content\":\"PaymentForOrder\",\"bankAccount\":\"%s\",\"bankCode\":\"%s\",\"userBankName\":\
       \"%s\",\"transType\":\"C\",\"orderId\":\"VPB%s\",\"terminalCode\":\"%s\",\"serviceCode\":\"SVC001\",\"additionalData\":\
       [{\"additionalData1\":\"tienmat\"}]}", cashValue*1000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str(), clientHandler.createRandomString().c_str(), clientHandler.getTerminalBox().c_str());
  
       boolean status = mqttClient.publish(topic_cash.c_str(), mqtt_buffer, 1);

       if (status == true)
       {
         printf("topic cash pub : %s \n\r", topic_cash.c_str());
         printf("data: %s \n\r", mqtt_buffer);
       }
       else
         printf("Pub Failure \n\r");
}

void display_QR()
{
  if (qrdata_rcv != NULL)
  {
    qr = lv_qrcode_create(ui_Screen2, max(QR_WIDTH, QR_HEIGHT), lv_color_hex(0x000000), lv_color_hex(0xffffff));
    lv_qrcode_update(qr, (const uint8_t *)qrdata_rcv, strlen(qrdata_rcv));
    lv_obj_set_size(qr, QR_WIDTH, QR_HEIGHT);
    lv_obj_set_pos(qr, (240 - QR_WIDTH) / 2, (320 - QR_HEIGHT) / 2);
    lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
    // timeout_BankSuccess(30);
    qrcode_active = true;
    qrcode_start_time = lv_tick_get();
    printf(" create timeout for QRcode \n\r");
    screen2_setTimeout(60);
  }
  qrdata_rcv = NULL;
}

static unsigned long  buttonTimer = 0;
void loop()
{

  switch (_STATE_CREATE_QR_ACTION)
  {
  case STATE_CLEAR_QRCODE:
     if (qr != NULL)
    {
      //lv_obj_del(qr);
      lv_obj_add_flag(qr, LV_OBJ_FLAG_HIDDEN);
      lv_qrcode_delete(qr);
      qr = NULL;
    }
    _STATE_CREATE_QR_ACTION = STATE_CREATE_SPINNER;
    /* code */
    break;
  
  case STATE_CREATE_SPINNER:
     create_spinner();
    _STATE_CREATE_QR_ACTION = STATE_CREATE_MESSAGE_REQUEST;
    /* code */
    break;
  case STATE_CREATE_MESSAGE_REQUEST:
      ClientRequestQR(moneyPublish, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
     _STATE_CREATE_QR_ACTION = STATE_CREATE_QRCODE;
    /* code */
    break;
  case STATE_CREATE_QRCODE:
    _STATE_CREATE_QR_ACTION = STATE_IDLE;
    /* code */
    break;
  
  default:
  _STATE_CREATE_QR_ACTION = STATE_IDLE;
    break;
  }
  
  if (!mqttClient.connected())
  {
    
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 3000L)
    {
      log("=== MQTT NOT CONNECTED ===");
      lastReconnectAttempt = t;
      String client_id = "esp32_LTE_";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
      if (mqttClient.connect(client_id.c_str(), mqtt_username, mqtt_password))
      {
        Serial.println("Public  broker connected");
        mqttClient.subscribe(clientHandler.getSyncBoxsTopic().c_str());

        JSONVar payload;
        payload["macAddr"] = clientHandler.getMacAddress();
        payload["checkSum"] = clientHandler.calculateChecksum();
        Serial.println("checksum");
        Serial.println(JSON.stringify(payload).c_str());

        mqttClient.publish(clientHandler.sync_topic.c_str(), JSON.stringify(payload).c_str());
        Serial.println("--> re publish for lost connect --> re-syncbox");
      }
      else
      {
        Serial.print("failed with state ");
        Serial.print(mqttClient.state());
        delay(2000);
      }
    }
  }
  else
  {
    mqttClient.loop();
  }

  Receive_handler();
  lv_task_handler();
  //delay(5);
  display_QR();
  buttonConfig_Handler();
  timerLVGL_Handler();
  screen2_timeout_handler();
  screenSucess_timeout_handler();
  fulse_read_nonBlocking();
  

  //check_spinner_timeout();

  static uint32_t signalTick = 0;
  if (millis() - signalTick > 5000)
  {
    signalTick = millis();
    if (CLIENT_MODE == LTE_MODE)
    {
      int csq = modem.getSignalQuality();
      rssi = csq * 100 / 31;
    }
    if (CLIENT_MODE == WIFI_MODE)
    {
      int8_t wifi_signal = WiFi.RSSI();
      // Convert RSSI to percentage
      int percent = map(wifi_signal, -100, -30, 0, 100);
      percent = constrain(percent, 0, 100); // Ensure it stays within 0-100%
      rssi = (int32_t)(percent);
    }
  }

  static uint32_t testTick = 0;
  if(millis() - testTick > 10)
  {
    testTick = millis();
    if (genFulseAction == 2)
    {
      // printf("main gen fulse control is 2 \n\r");
      fulse_generator(bdsd_moneyValue);
      bdsd_moneyValue = 0;
      genFulseAction = 0;
      printf("-----> finish \n\r");
      timeout_BankSuccess(10);
    }
    if (genFulseAction == 1 && bdsd_moneyValue > 0)
    {
      genFulseAction = 2;
      // printf(" change mode fulse 1--> 2-->\n\r");
    }
  }
  //fulse_generate_nonBlocking();
}

/**
 * @brief button reset config handler
 * 
 */
void buttonConfig_Handler(void)
{
  static uint8_t buttonCounter = 0;
  if (millis() - buttonTimer > 100)
  {
    buttonTimer = millis();
    int buttonState = digitalRead(BUTTON_PIN);
    // Kiểm tra trạng thái
    delayMicroseconds(100);
    if (buttonState == LOW)
    {
      buttonCounter++;
      Serial.println("Button Pressed"); // In ra khi nút được nhấn
      if(buttonCounter > 30)
      {   
          buttonCounter = 0;
          Serial.println("Goto reset account device \n\r"); // In ra khi nút được nhấn
          // while (1)
          // {
          //   /* code */
          // }
          _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen2_screen_init);
           qr = lv_qrcode_create(ui_Screen2, max(QR_WIDTH, QR_HEIGHT), lv_color_hex(0x000000), lv_color_hex(0xeeeeee));
           lv_qrcode_update(qr, (const uint8_t *)clientHandler.getQrCertificate().c_str(), strlen(clientHandler.getQrCertificate().c_str()));
           lv_obj_set_size(qr, QR_WIDTH, QR_HEIGHT);
           lv_obj_set_pos(qr, (240 - QR_WIDTH) / 2, (320 - QR_HEIGHT) / 2);
           lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
           resetAccountAction = true;
      }
    }
  }
}

void lcd_init()
{
  tft.init();
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(0);
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  ui_init();
  Serial.println(F("Setup done!"));
  // rssi = WiFi.RSSI();
  // Serial.print("RSSI: ");
  // Serial.println(rssi);
  delay(1000);
}
static uint8_t TIMEOUT_SPINNER = 30;
static uint8_t TIMEOUT_QR  = 60;

static uint8_t timeout_screen2 = 0;

void create_spinner()
{
  ui_Spinner2 = lv_spinner_create(ui_Screen2, 1000, 90);
  lv_obj_set_width(ui_Spinner2, 60);
  lv_obj_set_height(ui_Spinner2, 60);
  lv_obj_set_x(ui_Spinner2, 0);
  lv_obj_set_y(ui_Spinner2, 0);
  lv_obj_set_align(ui_Spinner2, LV_ALIGN_CENTER);
  lv_obj_clear_flag(ui_Spinner2, LV_OBJ_FLAG_CLICKABLE); // Flags

  screen2_setTimeout(30);

}

/* implement event click to select monay value */
void ui_event_ClickMoney_10k(lv_event_t * e)
{
   lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
      buzzerSetActive();
      //Serial.println(" ui_event_ClickMoney1");
      //ClientRequestQR(10000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
      //create_spinner();
      moneyPublish = 10000;
      _STATE_CREATE_QR_ACTION = STATE_CLEAR_QRCODE;
    }
}

void ui_event_ClickMoney_20k(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
      buzzerSetActive();
      //Serial.println(" ui_event_ClickMoney2");
      // ClientRequestQR(20000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
      // create_spinner();
      moneyPublish =  20000;
      _STATE_CREATE_QR_ACTION = STATE_CLEAR_QRCODE;
      
    }
}
void ui_event_ClickMoney_50k(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
      buzzerSetActive();
      // Serial.println(" ui_event_ClickMoney3");
      // ClientRequestQR(50000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
      // create_spinner();
      moneyPublish =  50000;
      _STATE_CREATE_QR_ACTION = STATE_CLEAR_QRCODE;
      
    }
}
void ui_event_ClickMoney_100k(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    buzzerSetActive();
    //Serial.println(" ui_event_ClickMoney4");
    // ClientRequestQR(100000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
    // create_spinner();
    moneyPublish = 100000;
    _STATE_CREATE_QR_ACTION = STATE_CLEAR_QRCODE;
    
  }
}
void ui_event_ClickMoney_200k(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    buzzerSetActive();
    //Serial.println(" ui_event_ClickMoney5");
    // ClientRequestQR(200000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
    // create_spinner();
    moneyPublish = 200000;
    _STATE_CREATE_QR_ACTION = STATE_CLEAR_QRCODE;
    
  }
}
void ui_event_ClickMoney_500k(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    buzzerSetActive();
    //Serial.println(" ui_event_ClickMoney6");
    // ClientRequestQR(500000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
    // create_spinner();
    moneyPublish =  500000;
    _STATE_CREATE_QR_ACTION = STATE_CLEAR_QRCODE;
  }
}

void ui_event_ClearQrCode(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    screen2_setTimeout(0);
    buzzerSetActive();
    printf(" clear Qr code \n\r");
    //lv_qrcode_delete(qr);  --> cash//
    if (qr != NULL)
    {
      //lv_obj_del(qr);
      lv_obj_add_flag(qr, LV_OBJ_FLAG_HIDDEN);
      lv_qrcode_delete(qr);
      qr = NULL;
    }
  }
}

// password is ok //
void submit_password_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) {
        const char *input_password = lv_textarea_get_text(password_input);
        const char *keypass = clientHandler.getkeyPass().c_str();
        if (strcmp(input_password,"11235813") == 0 || strcmp(input_password, keypass)== 0) {
          // lv_obj_del(ui_Screen5);
          lv_textarea_set_text(password_input, "");
          _ui_screen_change(&ui_Screen6, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen6_screen_init);

          // add old config  to lcd //
          char buffer[8];
          itoa(config.Ton, buffer, 10);
          lv_textarea_set_text(ui_TextAreaTon, buffer);

          itoa(config.Toff, buffer, 10);
          lv_textarea_set_text(ui_TextAreaToff, buffer);

          itoa(config.fulse_10K, buffer, 10);
          lv_textarea_set_text(ui_TextArea10k, buffer);

          itoa(config.fulse_20K, buffer, 10);
          lv_textarea_set_text(ui_TextArea20k, buffer);

          itoa(config.fulse_50K, buffer, 10);
          lv_textarea_set_text(ui_TextArea50k, buffer);

          itoa(config.fulse_100K, buffer, 10);
          lv_textarea_set_text(ui_TextArea100k, buffer);

          itoa(config.fulse_200K, buffer, 10);
          lv_textarea_set_text(ui_TextArea200k, buffer);

          itoa(config.fulse_500K, buffer, 10);
          lv_textarea_set_text(ui_TextArea500k, buffer);

          Serial.println("Correct password");

        } else {
            lv_label_set_text(password_label, "Wrong Password!");
        }
    }
}
void ui_event_selectCancel(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen5_screen_init);
    }
}
static bool timer_created = false;
 void change_screen(lv_timer_t * timer)
{
    _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);
    timer_created = false; 
 lv_timer_del(timer);
}

void ui_event_seclectSave(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    
    if(event_code == LV_EVENT_RELEASED) {
      timer_created = true;
        const char *Toff = lv_textarea_get_text(ui_TextAreaToff);
        const char *Ton = lv_textarea_get_text(ui_TextAreaTon);
        const char *Area_Text10K = lv_textarea_get_text(ui_TextArea10k);
        const char *Area_Text20K = lv_textarea_get_text(ui_TextArea20k);
        const char *Area_Text50K = lv_textarea_get_text(ui_TextArea50k);
        
        const char *Area_Text100K = lv_textarea_get_text(ui_TextArea100k);
        const char *Area_Text200K = lv_textarea_get_text(ui_TextArea200k);
        const char *Area_Text500K = lv_textarea_get_text(ui_TextArea500k);

        printf("Ton: %s\n", Ton);
        printf("Toff: %s\n", Toff);
        printf("10k: %s\n", Area_Text10K);
        printf("20k: %s\n", Area_Text20K);
        printf("50k: %s\n", Area_Text50K);
        printf("100k: %s\n", Area_Text100K);
        printf("200k: %s\n", Area_Text200K);
        printf("500k: %s\n", Area_Text500K);
        lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0xFF0000), 0);
        lv_label_set_text(ui_Label2, "Thanh Cong");
 
        lv_timer_create(change_screen, 3000, NULL);

        // save data to memory //
        // save to memory //
        String dataContent ="";
        StaticJsonDocument<256> settingJson;
        // JSONVar payload;
        settingJson["Ton"] = atoi(Ton);
        settingJson["Toff"] = atoi(Toff);
        settingJson["10k"] = atoi(Area_Text10K);
        settingJson["20k"] = atoi(Area_Text20K);
        settingJson["50k"] = atoi(Area_Text50K);
        settingJson["100k"] = atoi(Area_Text100K);
        settingJson["200k"] = atoi(Area_Text200K);
        settingJson["500k"] = atoi(Area_Text500K);

        serializeJson(settingJson, dataContent);
        printf("json, %s \n", dataContent.c_str());

        config.writeSetting(dataContent.c_str());
    }
}

void ui_event_ResetAccount(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    if(event_code == LV_EVENT_RELEASED) {
       printf(" reset account event \n\r");
       //CERT-VVB-VlZCMTIxNTExODgxM0JGMEM1QTYwVmlldFFSQm94QWNjZXNzS2V5//
      //  if (qrdata_rcv != NULL)
      //  {
         qr = lv_qrcode_create(ui_Screen4, max(QR_WIDTH, QR_HEIGHT), lv_color_hex(0x000000), lv_color_hex(0xffffff));
         lv_qrcode_update(qr, (const uint8_t *)clientHandler.getQrCertificate().c_str(), strlen(clientHandler.getQrCertificate().c_str()));
         lv_obj_set_size(qr, QR_WIDTH, QR_HEIGHT);
         lv_obj_set_pos(qr, (240 - QR_WIDTH) / 2, (320 - QR_HEIGHT) / 2);
         lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
         // timeout_BankSuccess(30);
       //}
      //qrdata_rcv = NULL;
    }
}

void timerLVGL_Handler(void)
{
  static uint32_t cnt = 0;
  static uint32_t timerCounter = 0;
  if (millis() - timerCounter > 1000)
  {
    timerCounter = millis();
    if (activeAccountOK == 1)
    {
      cnt++;
      if (cnt == 5)
      {
        activeAccountOK = 0;
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);
      }
    }
    else
      cnt = 0;
  }
}

static void convertMoneyToFulse(uint32_t amount)
{
  switch (amount)
   {
   case 10000:
    NumFulseGenerator = config.fulse_10K;
    break;

  case 20000:
    NumFulseGenerator = config.fulse_20K;
    break;
  case 50000:
    NumFulseGenerator = config.fulse_50K;
    break;
  case 100000:
    NumFulseGenerator = config.fulse_100K;
    break;
  case 200000:
    NumFulseGenerator = config.fulse_200K;
    break;
  case 500000:
    NumFulseGenerator = config.fulse_500K;
    break;
  default:
    NumFulseGenerator = 0;
    break;
   }

   if(NumFulseGenerator)
   {
      printf("Convert money: %d to fulse:%d\r\n",amount, NumFulseGenerator);
   }
}

void fulse_generator(u32_t amount)
{
  int fulse = 0;
  uint32_t fulseLow = config.Toff * 1000; //  milisecond
  uint32_t fulseHigh = config.Ton *1000;  // milisecond
  //printf( "ton = %d toff =%d \n\r", fulseHigh, fulseLow);
   switch (amount)
   {
   case 10000:
    fulse = config.fulse_10K;
    break;

  case 20000:
    fulse = config.fulse_20K;
    break;
  case 50000:
    fulse = config.fulse_50K;
    break;
  case 100000:
    fulse = config.fulse_100K;
    break;
  case 200000:
    fulse = config.fulse_200K;
    break;
  case 500000:
    fulse = config.fulse_500K;
    break;
  default:
    fulse = 0;
    break;
   }

   if(fulse)
   {
      NumFulseGenerator = fulse;
      printf("number fulse= %d\r\n", fulse);
   }

  BLOCK_READING = true;
  for(int  i = 0; i < fulse ; i++)
  {
    digitalWrite(FULSE_OUT, LOW);
    delayMicroseconds(fulseLow);
    digitalWrite(FULSE_OUT, HIGH);
    delayMicroseconds(fulseHigh);
  }
  BLOCK_READING = false;
}

// /**
//  * @brief screen2_timeout_handler
//  */

void screen2_timeout_handler(void)
{
  if (millis() - buzzerTick > 1000)
  {
    buzzerTick = millis();

    if (ACTIVE_TIMER == false)
    {
      //printf(" timer ACTIVE false -> pause \n\r");
      return;
    }
    char text_timeout[8];
    if (MAX_TIMEOUT == 0)
    {
      _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);

      MAX_TIMEOUT = -1;
      ACTIVE_TIMER = false;
      return;
    }

    if (MAX_TIMEOUT > 0)
    {
      MAX_TIMEOUT--;
      sprintf(text_timeout, "%ds", MAX_TIMEOUT);
      lv_label_set_text(ui_LabelTimeout, text_timeout);
    }
  }
}

void screenSucess_timeout_handler(void)
{
   if (millis() - successTick > 1000)
  {
    successTick = millis();

    if (ACTIVE_TIMER_SUCCESS == false)
    {
      return;
    }
    char text_timeout[8];
    if (MAX_TIMEOUT == 0)
    {
      _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);

      MAX_TIMEOUT = -1;
      ACTIVE_TIMER = false;
      return;
    }

    if (MAX_TIMEOUT > 0)
    {
      MAX_TIMEOUT--;
      // sprintf(text_timeout, "%ds", MAX_TIMEOUT);
      // lv_label_set_text(ui_LabelTimeout, text_timeout);
    }
  }
}

static volatile uint32_t fulseTick = 0;
static volatile uint32_t fulseGenCounter = 0;
static volatile uint16_t NumberClock = 0;

void fulse_generate_nonBlocking(void)
{
  // if (millis() - fulseTick > 1)
  // {
  //   fulseTick = millis();
     switch (_STATE_FULSE_ACTION)
     {
     case STATE_FULSE_START: // start gen fulse -> convert value //
     BLOCK_READING = true;
     convertMoneyToFulse(bdsd_moneyValue);
     NumberClock = NumFulseGenerator;
     fulseGenCounter = 0;
      _STATE_FULSE_ACTION = STATE_FULSE_LOW;
      break;

    case STATE_FULSE_LOW: // start gen fulse -> convert value //
     fulseGenCounter++;
      digitalWrite(FULSE_OUT, LOW);
      _STATE_FULSE_ACTION = STATE_FULSE_WAIT_LOW_OK;
      break;

    case STATE_FULSE_WAIT_LOW_OK: // start gen fulse -> convert value //
     fulseGenCounter++;
     if(fulseGenCounter >= 50) 
        _STATE_FULSE_ACTION = STATE_FULSE_HIGH;
      break;

    case STATE_FULSE_HIGH: // start gen fulse -> convert value //
     fulseGenCounter++;
     digitalWrite(FULSE_OUT, HIGH);
      _STATE_FULSE_ACTION = STATE_FULSE_WAIT_HIGH_OK;
      break;

    case STATE_FULSE_WAIT_HIGH_OK: // start gen fulse -> convert value //
     fulseGenCounter++;
     if(fulseGenCounter >= 200) 
     {
         digitalWrite(FULSE_OUT, HIGH);
     NumberClock--;
     if(NumberClock)
     
      _STATE_FULSE_ACTION = STATE_FULSE_LOW;
     else
       _STATE_FULSE_ACTION = STATE_FULSE_FINISH;

      fulseGenCounter = 0; 
     }
      break;

    case STATE_FULSE_FINISH: // start gen fulse -> convert value //
       
       BLOCK_READING = false;
      // printf(" genfulse is done !!!! \n\r");
      _STATE_FULSE_ACTION = STATE_IDLE;
      break;
     
     default:
       _STATE_FULSE_ACTION = STATE_IDLE;
      break;
     }
  
}

static uint8_t lastState  = HIGH;
static uint8_t fulseCounter = 0;
static uint16_t fulseTimeout = 0;

void fulse_read_nonBlocking(void)
{
  if(BLOCK_READING)
     return;
  if (millis() - TickTimerReadInput > 10)
  {
    TickTimerReadInput = millis();
    int buttonState = digitalRead(FULSE_INPUT);
    // Kiểm tra trạng thái
    //delayMicroseconds(100);
    if (buttonState == LOW && lastState == HIGH)
    {
        fulseTimeout = 0;
        fulseCounter++;
        lastState = LOW;
    }
    if (buttonState == HIGH && lastState == LOW)
    {
       lastState = HIGH;
    }
    if(buttonState == HIGH)
    {
       fulseTimeout++; 
       if(fulseTimeout > 25)
       {
          if(fulseCounter != 0)
          {
             printf(" timeout --> fulse = %d \n\r", fulseCounter);
             // publish data to server //
            uint16_t money =  convertFulseToMoney(fulseCounter);
            ClientReportCash(money);
          }
          fulseCounter = 0;
       }
    }
  }
}

/** modem fuction*/
void ModemHandler_init()
{
  pinMode(PPP_MODEM_RESET, OUTPUT);    // accept for all Module 4G combine RST and PWR//
  digitalWrite(PPP_MODEM_RESET, LOW);
  delay(250);
  digitalWrite(PPP_MODEM_RESET, HIGH);
  delay(100);
  modem.restart();
  
  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

   int simStatus = modem.getSimStatus();
  if(simStatus == SIM_READY)
    Serial.println("\n\r SIMCARD IS READY --> Goto Connect gprs !!!");
  else
    Serial.println("\n\r SIMCARD IS ERROR --> Please connect simcard !!!");
}

void ModemHandler_connect_gprs()
{
  log("Waiting for network...." + time());
  if (!modem.waitForNetwork())
  {
    log("fail" + time());
    delay(10000);
    return;
  }
  if (modem.isNetworkConnected())
  {
    log("Network connected" + time());
  }
  log("GPRS connect..." + time());
  if (!modem.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str()))
  {
    log("fail");
    delay(10000);
    return;
  }
  if (modem.isGprsConnected())
  {
    log("GPRS connected");
  }
}

void buzzerSetActive()
{
  buzzerActive =  true;

}

static uint16_t convertFulseToMoney(uint16_t fulse)
{
 
  int fulse10K, fulse20K, fulse50K, fulse100K, fulse200K, fulse500K;
  fulse10K = config.fulse_10K;
  fulse20K = config.fulse_20K;
  fulse50K = config.fulse_50K;
  fulse100K = config.fulse_100K;
  fulse200K = config.fulse_200K;
  fulse500K = config.fulse_500K;

  if(fulse == config.fulse_10K)
     return 10;
  else if(fulse == config.fulse_20K)
   return 20;
  
   else if(fulse == config.fulse_50K)
   return 50;

    else if(fulse == config.fulse_100K)
   return 100;

    else if(fulse == config.fulse_200K)
   return 200;

    else if(fulse == config.fulse_500K)
   return 500;
 else
  return 0;
}

void screen2_setTimeout(int value)
{   
    if(value == 0)
    {
       MAX_TIMEOUT = 0;
       ACTIVE_TIMER =  true;
       //printf(" reset timeout = 0 \n\r");
    }
    if (value > 0)
    {
        char text_timeout[8];
        ACTIVE_TIMER = false;
        MAX_TIMEOUT = value;
        //printf(" set ui2 screen = %d -->tick = %d \n\r", MAX_TIMEOUT, lv_tick_get());
        sprintf(text_timeout,"%ds",MAX_TIMEOUT);
        lv_label_set_text(ui_LabelTimeout,text_timeout);
        ACTIVE_TIMER = true;
    }
}

void screenSuccess_setTimeout(int value)
{  
  return;
}


int32_t getWiFiStrengthAsPercentage(int32_t rssi) {
    int8_t quality;
    
    if (rssi <= -100) {
        quality = 0;
    } else if (rssi >= -50) {
        quality = 100;
    } else {
        quality = 2 * (rssi + 100);
    }
    
    return quality;
}

void  test_hash_code(void)
{
  String key = clientHandler.getkeyPass();
  Serial.println("key pass is:" + key);
}
/*
#define MODEM_INIT                                      0
#define MODEM_GET_INFOR                                 1
#define MODEM_NETWORK_CONNECTING                        2
#define MODEM_NETWORK_CONNECTED                         3
#define MODEM_GPRS_CONNECTING                           4
#define MODEM_GPRS_CONNECTED                            5
*/
void networkConnector(uint32_t interval)
{ 
  static uint32_t TimerTick = 0;
  static bool action = false;
  uint8_t CONNECT_STATE =  TCPIPConnection.Connectionstate;
  String modemInfo;
 

  if(millis()- TimerTick > interval)
  {
    TimerTick = millis();
    action = true;
  }
   /* task handler network connector */

  if (action == true )
  {
    action = false;
    TCPIPConnection.Tick++;
    switch (CONNECT_STATE)
    {
    case MODEM_INIT: // reset modem //
    {
      digitalWrite(PPP_MODEM_RESET, LOW);
      delay(250);
      digitalWrite(PPP_MODEM_RESET, HIGH);
      delay(100);
      modem.restart();
      TCPIPConnection.Connectionstate = MODEM_GET_INFOR;
      TCPIPConnection.Tick = 0;
      break;
    }
    case MODEM_GET_INFOR:
    {
      modemInfo = modem.getModemInfo();
      Serial.print("Modem Info: ");
      Serial.println(modemInfo);

      int simStatus = modem.getSimStatus();
      if (simStatus == SIM_READY)
        Serial.println("\n\r SIMCARD IS READY --> Goto Connect gprs !!!");
      else
        Serial.println("\n\r SIMCARD IS ERROR --> Please connect simcard !!!");

      int index = modemInfo.indexOf("IMEI");
      if(index > 0 && simStatus == SIM_READY)
      {
         TCPIPConnection.Connectionstate = MODEM_NETWORK_CONNECTING;
      }
      else{

      }
      break;
    }
    default:
       
      break;
    }
  }
}

void startMqttClientTask()
{
  
}

// =========================== define function ==================

void TaskConnectInternet(void *pvParameters)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    vTaskDelay(200);

    printf("TaskConnectInternet \n\r");
    xSemaphoreGive(mutex);
}

void TaskMqttClient(void *pvParameters)
{
  vTaskDelay(500);
  xSemaphoreTake(mutex, portMAX_DELAY);

   printf("TaskMqttClient \n\r");
 
  xSemaphoreGive(mutex);

}

void getNTPTimer()
{  
  if(CLIENT_MODE == LTE_MODE)
  {
  Serial.println("Asking modem to sync with NTP");
  modem.NTPServerSync("pool.ntp.org", 28);

  int   ntp_year     = 0;
  int   ntp_month    = 0;
  int   ntp_day      = 0;
  int   ntp_hour     = 0;
  int   ntp_min      = 0;
  int   ntp_sec      = 0;
  float ntp_timezone = 0;
  for (int8_t i = 5; i; i--) {
    Serial.println("Requesting current network time");
    if (modem.getNetworkTime(&ntp_year, &ntp_month, &ntp_day, &ntp_hour,
                             &ntp_min, &ntp_sec, &ntp_timezone)) {
      break;
    } else {
      Serial.println("Couldn't get network time, retrying in 15s.");
      delay(15000L);
    }
  }
  Serial.println("Retrieving time again as a string");
  String time = modem.getGSMDateTime(DATE_FULL);
  log("Current Network Boot Time:" + time);
  }

  else
  {
    struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");
  Serial.println("Time variables");
  }
}

void ui_event_SaveWiFi(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    const char *wifi_ssid = lv_textarea_get_text(ui_SSID);
    const char *wifi_pwd = lv_textarea_get_text(ui_PWD);

    String dataContent = "";
    StaticJsonDocument<256> settingJson;
    // JSONVar payload;
    settingJson["SSID"] = wifi_ssid;
    settingJson["PWD"]  = wifi_pwd;

    serializeJson(settingJson, dataContent);
    printf("json, %s \n", dataContent.c_str());
    //config.writeSetting(dataContent.c_str());
    //(get_SSID, get_PWD);
  }
}
