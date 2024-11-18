#ifndef __MAIN_H__
#define __MAIN_H__

#include "Arduino.h"

const char* ssid = "FFT-VT";
const char* password = "11235813";

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define FONT_SIZE 2
#define QR_CONTENT_SIZE 512

#define  QR_WIDTH   172
#define  QR_HEIGHT  172

// define PIN //
#define BUTTON_PIN   0  // Định nghĩa chân kết nối nút nhấn
#define FULSE_OUT    21  // xuất xung
#define FULSE_INPUT  35  // đọc xung

#define BUZZER_PIN   22  // còi beep //

String Terminal_ID = "";
String topic_request = "";
String topic_cash = "";
String topic_qr = "";
String topic_status ="";
String topic_bdsd = "";


String topic_cash_prefix = "vietqr/cash/";
String topic_request_prefix = "vietqr/request/";
String topic_qr_prefix = "vietqr/response/";
String topic_status_prefix ="vietqr/response-status/";
String topic_bdsd_prefix = "vietqr/bdsd/";

char topic_handlerbox_pub[128];
char topic_handlerbox_sub[128];

String qr_topic_prefix = "vietqr/boxId/";
String sync_topic_prefix = "/vqr/handle-box";


//const char* msg_get = "{\"amount\":5000,\"content\":\"PaymentForOrder\",\"bankAccount\":\"9690194300493\",\"bankCode\":\"MB\",\"userBankName\":\"TrieuNgocXuan\",\"transType\":\"C\",\"orderId\":\"ORD12345XYZ\",\"terminalCode\":\"VVB121511\",\"serviceCode\":\"SVC001\",\"additionalData\":[{\"additionalData1\":\"xuantest\"}]}";
//const char *mqtt_broker = "112.78.1.209";
const char *mqtt_broker = "api.vietqr.org";
const char *mqtt_username = "vietqrprodAdmin123";
const char *mqtt_password = "vietqrbns123";
const int   mqtt_port = 1883;

#define RXD1              16
#define TXD1              17
#define PPP_MODEM_RESET   4

const int beePin = 27; // buzzer pin //

uint8_t CLIENT_MODE;

String apn     = "";
String gprsUser = "";
String gprsPass= "";

// state machine connect //
enum CONNECT_STATUS_ENUM
{
    E_POWER_UP,
    E_INIT_INTERNET,
    E_CONNECTING, 
    E_CONENNECTED,
};

void screen2_setTimeout(int value);

#define STATE_CLEAR_QRCODE    1
#define STATE_CREATE_SPINNER  2
#define STATE_CREATE_MESSAGE_REQUEST  3
#define STATE_CREATE_QRCODE   4
#define STATE_IDLE            0

//state machine for gen fulse //
#define STATE_FULSE_START             1

#define STATE_FULSE_LOW               2
#define STATE_FULSE_WAIT_LOW_OK       3

#define STATE_FULSE_HIGH              4
#define STATE_FULSE_WAIT_HIGH_OK      5
#define STATE_FULSE_FINISH            6
#define STATE_IDLE                    0


/* TCP connection state */
#define MODEM_INIT                                      0
#define MODEM_GET_INFOR                                 1
#define MODEM_NETWORK_CONNECTING                        2
#define MODEM_NETWORK_CONNECTED                         3
#define MODEM_GPRS_CONNECTING                           4
#define MODEM_GPRS_CONNECTED                            5

typedef struct
{
    volatile uint8_t Connectionstate;
    volatile uint32_t Timeout;
    volatile uint32_t Tick;
} TCPIPConnection_def;

#endif
