#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <OneButton.h>
#include <DFRobot_MAX17043.h>
#include <ESP32MX1508.h>
#include "freertos/timers.h"



//#include <Arduino_FreeRTOS.h>
//#include <ADS1115-Driver.h>



#define buttonStopReset 0
#define buttonStart 4
#define buttonEqOn 16
#define buttonEqOff 17
#define buttonPumpOn 5
#define buttonPumpOff 18
#define buttonLift 23 //35
#define buttonLower 19 // 34
#define buttonStopResetLed 22
#define buttonStartLed 27
#define buttonEqOnLed 26
#define buttonPumpOnLed 25
#define buttonLiftLed 35 // 23
#define buttonLowerLed 34 //19
#define potPin 39
#define potBright 36
#define motorFarwardPin 13
#define motorBackwardPin 14
#define I2C_SCL 33
#define I2C_SDA 32

#define RES 12  // Resolution in bits:  8 (0-255),  12 (0-4095), or 16 (0-65535)
#define FREQ 5000
#define CH1 0  // 16 Channels (0-15) are availible
#define CH2 1  // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)             // PWM Frequency in Hz



// sync Timers
// https://github.com/hermanschmit/espnow_sync

int minutes = 1;
int currentMin = minutes;
float v;
byte drawonce = 0;
int remotePotValue;
int getRemotePotValue = 0;
int getRemotePotValueOld = 0;
int localPotValue = 0;


float gs_rad;       //stores angle from where to start in radinats
float ge_rad;       //stores angle where to stop in radinats
byte cx = 112;      //x center
byte cy = 14;       //y center
byte radius = 14;   //radius
byte radius2 = 11;  //radius
byte percent = 50;  //needle percent
int pot = 0;
double soc = 0;  // Variable to keep track of LiPo state-of-charge (SOC)
bool alert;
int rssi_display;

byte flash = 0;  //Flag for display flashing - toggle once per update interval


int State = 0;
unsigned long Timestamp_Button_Pressed;

unsigned long upDownDisplayTimer;
unsigned int arrowDisplayMoveSpeed = 500;
long TimeWaitBeforeReading_pot;
long TimeWaitBeforeReading_bat;
long TimerClock;
long runMillis = 999;
unsigned long ClockMillis;
int M;
int S;
int reset_timer = 0;
int state_pause = 0;
unsigned int readSamples = 100;
uint32_t sum = 0;


unsigned long buttonPressEndTime1 = 0;
unsigned long buttonPressEndTime2 = 0;
unsigned long buttonPressStartTime1 = 0;
unsigned long buttonPressStartTime2 = 0;
unsigned long buttonPressDuration = 0;
unsigned long buttonPressPeriod = 0;
unsigned long buttonPressCounter = 0;
unsigned int liftSet = 0;
unsigned long pressStartTime;
unsigned long button_time = 0;
unsigned long last_button_time = 0;
unsigned long currentMillis;
bool isnNotSet = true;
bool doNotCount1 = false;
bool doNotCount2 = false;
bool runOnce1 = false;
bool getTimer = true;
bool getTimerForBtnLiftStart = true;
bool getTimerForBtnLiftEnd = true;
bool getTimerForBtnLowerStart = false;
bool getTimerForBtnLowerEnd = false;
const TickType_t xFrequency = 5000;
int arr[] = { 40, 50, 60, 70, 80, 90 };
byte timerRunning1;
byte timerRunning2;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers


// need to chahge for the external ESP address.
uint8_t RxMACaddress[] = { 0x54, 0x43, 0xB2, 0xA9, 0x1E, 0x90 };  // first ESP
//uint8_t RxMACaddress[] = {0x3C, 0x61, 0x05, 0x65, 0x03, 0x44}; // Second ESP


DFRobot_MAX17043 gauge;
MX1508 motor(motorFarwardPin, motorBackwardPin, CH1, CH2);
U8G2_ST7567_ENH_DG128064I_F_SW_I2C u8g2(U8G2_R2, 2, 15, U8X8_PIN_NONE);
OneButton button1(buttonStopReset, true);
OneButton button2(buttonEqOn, true);
OneButton button3(buttonEqOff, true);
OneButton button4(buttonPumpOn, true);
OneButton button5(buttonPumpOff, true);
OneButton button6(buttonStart, true);
//OneButton button7(buttonLower, false);
//OneButton button8(buttonStart, false);



/*** Create the tasks ***/

TaskHandle_t Task_interruptsLift;
TaskHandle_t Task_interruptLower;
TaskHandle_t Task_interruptsRun;
TaskHandle_t Task_ProgramRun;
TaskHandle_t Task_Timer;
TaskHandle_t Task_arrowDown;
TaskHandle_t Task_arrowUp;



void IRAM_ATTR isr() {
}


 /*** this Code relate to Wifi Signal ***/

typedef struct {
  unsigned frame_ctrl : 16;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

typedef struct TxStruct {
  int potVal;
  //int ledpinState; int ledFadeState;
} TxStruct;
TxStruct sentData;

typedef struct RxStruct {
  float temp;
  int humidity;
  //int ledpinState; int ledFadeState;
} RxStruct;
RxStruct receivedData;




void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  int rssi = ppkt->rx_ctrl.rssi;
  rssi_display = rssi;
  Serial.println(rssi_display);
}


 /***********************************************************************************************************************/

// this code relate to send data from one ESP32 to another //
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
   Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//------------------------------------------------------------
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Bytes received: ");
  Serial.println(len);
}

/***********************************************************************************************************************/

/*** this section relate to change brighness of the display - at the moment not connected to the pot ***/
void setScreenBrightness() {
  int val = map(analogRead(potBright), 0, 4095, 220, 255);
  u8g2.setContrast(val);
}

void readPot() {
  int k;
  for (k = 0; k < readSamples; k = k + 1) {
    sum += analogRead(potPin);
    delayMicroseconds(1000);
  }
  pot = sum / readSamples;
  sum = 0;
}

/***********************************************************************************************************************/


/*** this code related to move the speed pot in the second ESP32  (the one that connected to the machine, not implemented in this remote system) ***/
void motorRun() {
  readPot();
  if ((getRemotePotValue >= 0) && (getRemotePotValue != getRemotePotValueOld)) {
    getRemotePotValueOld = getRemotePotValue;
    localPotValue = getRemotePotValue;
  } else {
    localPotValue = pot;
  }

  if ((localPotValue - pot < 10) && (localPotValue - pot > 0)) {
    if (pot < 4095) {
      motor.motorGo(150);
    }
  }

  else if (localPotValue > pot) {
    if (pot < 4095) {
      motor.motorGo(150);
    }
  }

  else if ((pot - localPotValue < 10) && (pot - localPotValue > 0)) {
    if (pot > 0) {
      motor.motorRev(150);
    }
  }

  else if (localPotValue < pot) {
    if (pot > 0) {
      motor.motorRev(255);
    }
  }

  else if (localPotValue == pot) {
    motor.motorBrake();
  }
}
/***********************************************************************************************************************/

/*** Drawing the Wifi signal on the screen ***/
void DrawRSSI() {
  int x = 1;
  if ((rssi_display <= -30) && (rssi_display <= -40)) {
    for (int j = 27; j < 38; j++) {
      int number = j;
      u8g2.drawLine(27 - x, 12, 27 - x, 1);
      u8g2.drawLine(28 - x, 12, 28 - x, 1);
      u8g2.drawLine(30 - x, 12, 30 - x, 4);
      u8g2.drawLine(31 - x, 12, 31 - x, 4);
      u8g2.drawLine(33 - x, 12, 33 - x, 6);
      u8g2.drawLine(34 - x, 12, 34 - x, 6);
      u8g2.drawLine(36 - x, 12, 36 - x, 8);
      u8g2.drawLine(37 - x, 12, 37 - x, 8);
    }
    // }
  } else if ((rssi_display <= -41) && (rssi_display <= -50)) {
    u8g2.drawLine(30 - x, 12, 30 - x, 4);
    u8g2.drawLine(31 - x, 12, 31 - x, 4);
    u8g2.drawLine(33 - x, 12, 33 - x, 6);
    u8g2.drawLine(34 - x, 12, 34 - x, 6);
    u8g2.drawLine(36 - x, 12, 36 - x, 8);
    u8g2.drawLine(37 - x, 12, 37 - x, 8);
  } else if ((rssi_display <= -51) && (rssi_display <= -60)) {
    u8g2.drawLine(33 - x, 12, 33 - x, 6);
    u8g2.drawLine(34 - x, 12, 34 - x, 6);
    u8g2.drawLine(36 - x, 12, 36 - x, 8);
    u8g2.drawLine(37 - x, 12, 37 - x, 8);
  } else if ((rssi_display <= -61) && (rssi_display <= -75)) {
    u8g2.drawLine(36 - x, 12, 36 - x, 8);
    u8g2.drawLine(37 - x, 12, 37 - x, 8);
  } else if ((rssi_display <= -76) && (rssi_display <= -95)) {
  }
}

/***********************************************************************************************************************/

/*** drwinng the speed gauge together with the speed pot & Wifi signal ***/

void Drawgauge(int x, byte y, byte r, byte p, int v, int minVal, int maxVal) {

  if ((millis() - TimeWaitBeforeReading_pot > 50) && (digitalRead(buttonLower == LOW)) && (digitalRead(buttonLift == LOW))) {
    readPot();
    DrawRSSI();
    TimeWaitBeforeReading_pot = millis();
  }
  u8g2.drawCircle(x, y, r, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(x, y, radius2, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawLine(cx - r, cy, cx - radius2, cy);
  u8g2.drawLine(cx + r, cy, cx + radius2, cy);
  u8g2.setDrawColor(1);
  u8g2.setCursor(x + 7, y + 15);
  float val = map(pot, 0, 4095, 0, 180);
  int val1 = map(pot, 0, 4095, 0, 100);
  val = val * 3.14 / 180 - 1.572;
  int xp = x + (sin(val) * radius);
  int yp = y - (cos(val) * radius);
  u8g2.drawLine(x, y, xp, yp);
  u8g2.drawLine(x - 1, y - 1, xp, yp);
  u8g2.drawLine(x + 1, y - 1, xp, yp);
  u8g2.drawLine(x - 1, y, xp, yp);
  u8g2.drawLine(x + 1, y, xp, yp);
  u8g2.setFont(u8g2_font_timB08_tr);
  u8g2.drawStr(40, 12, "Speed:");
  if (val1 == 0) {
    u8g2.setCursor(82, 12);
  }
  if (val1 > 0 && val1 < 10) {
    u8g2.setCursor(82, 12);
  }
  if (val1 >= 10 && val1 < 100) {
    u8g2.setCursor(78, 12);
  }
  if (val1 == 100) {
    u8g2.setCursor(72, 12);
  }
  u8g2.print(val1);
  u8g2.drawStr(88, 12, "%");
}
/***********************************************************************************************************************/

/*** Drawing the battery guage ***/

void battery() {

  if (millis() - TimeWaitBeforeReading_bat > 100) {
    v = gauge.readVoltage();
    soc = gauge.readPercentage();
  }

  TimeWaitBeforeReading_bat = millis();
  upDownDisplayTimer = millis();

  // }
  if (v >= 3000) {
    u8g2.drawFrame(1, 2, 19, 11);
  } else if (flash) {
    u8g2.drawFrame(1, 2, 19, 11);
  }
  if (v <= 3300) {
    u8g2.drawBox(20, 5, 2, 5);
  } else if (flash) {
    u8g2.drawBox(20, 5, 2, 5);
  }
  if (v > 3300) {
    u8g2.drawBox(20, 5, 3, 5);
  } else if (flash) {
    u8g2.drawBox(20, 5, 3, 5);
  }
  if (v > 4100) { u8g2.drawBox(2, 4, 18, 7); }
  if (v > 3950 && v <= 4100) { u8g2.drawBox(2, 4, 15, 7); }
  if (v > 3800 && v <= 3950) { u8g2.drawBox(2, 4, 11, 7); }
  if (v > 3650 && v <= 3800) { u8g2.drawBox(2, 4, 7, 7); }
  if (v > 3500 && v <= 3650) { u8g2.drawBox(2, 4, 3, 7); }
  if (v > 3350 && v <= 3500) { u8g2.drawBox(2, 4, 2, 7); }
}
/***********************************************************************************************************************/

/*** for timming of the switches (long press / short press) + the lib for the buttons ***/
void isr();

void IRAM_ATTR Buttons() {
  // include all buttons here to be checked
  button1.tick();  // just call tick() to check the state.
  button2.tick();
  button3.tick();
  button4.tick();
  button5.tick();
  button6.tick();
}

/***********************************************************************************************************************/

/***  The buttons functions ***/
void buttonTROff() {
  digitalWrite(buttonStopResetLed, HIGH);
  digitalWrite(buttonStartLed, LOW);
  if (State == 1) {
    State = 0;
    state_pause = 1;
    Serial.println("Stop");
  }
}
void resetTimer() {
  digitalWrite(buttonStopResetLed, HIGH);
  digitalWrite(buttonStartLed, LOW);
  Serial.println("ResetTimer()");
  currentMin = minutes;
  M = 0;
  S = 0;
  State = 0;
  reset_timer = 1;
  pressStartTime = millis() - 2000;  // as set in setPressTicks()
}


void buttonTROn() {
  digitalWrite(buttonStartLed, HIGH);
  digitalWrite(buttonStopResetLed, LOW);
  // if (millis() - Timestamp_Button_Pressed>100){
  //   Timestamp_Button_Pressed=millis();
  if (State == 0 || State == 2) {
    State = 1;
    reset_timer = 0;
    state_pause = 0;
  }
}
void Equipment_on() {
  digitalWrite(buttonEqOnLed, HIGH);
  //digitalWrite(buttonEqOffLed, LOW);
}

void resetLift() {
  digitalWrite(buttonEqOnLed, HIGH);
  //digitalWrite(buttonEqOffLed, LOW);
  buttonPressDuration = 0;
  pressStartTime = millis() - 2000;
}

void Equipment_off() {
  //digitalWrite(buttonEqOffLed, HIGH);
  digitalWrite(buttonEqOnLed, LOW);
}

void Punp_on() {
  digitalWrite(buttonPumpOnLed, HIGH);
  //digitalWrite(buttonPumpOffLed, LOW);
}

void Punp_off() {
  //digitalWrite(buttonPumpOffLed, HIGH);
  digitalWrite(buttonPumpOnLed, LOW);
}



/*** some flags ***/
 
bool runOnce5 = false;
bool runOnce6 = false;
int notifyToArrowDownTask = 0;
int notifyToArrowUpTask = 0;
bool getSum = false;
bool calculateSum = false;
bool getSum1 = false;
bool calculateSum1 = false;
unsigned long buttonPressDurationlast = buttonPressDuration;
unsigned long buttonPressDurationSum;

void Lift() {

  if (digitalRead(buttonLift) == HIGH) {
    if (notifyToArrowUpTask == 0) {
      notifyToArrowUpTask = 1;
      //runOnce5 = false;
    }
  }

  if (digitalRead(buttonLift) == LOW) {
    digitalWrite(buttonLiftLed, LOW);
    if (notifyToArrowUpTask == 1) {
      notifyToArrowUpTask = 0;
      timerRunning1 = 0;
      getSum1 = true;
    }
  }

  if (digitalRead(buttonLift) == HIGH && liftSet == 0) {
    digitalWrite(buttonLiftLed, HIGH);
    //  if (runOnce5 == false) {
    getTimerForBtnLiftEnd = true;
    getSum1 = false;
    calculateSum1 = true;
    //   }
  }

  if (timerRunning1 == 0 && getTimerForBtnLiftEnd == true) {
    getTimerForBtnLiftStart = true;
    doNotCount1 = false;
    timerRunning1 = 1;
  }

  if ((digitalRead(buttonLift) == HIGH) && (liftSet == 1) && (doNotCount1 == false)) {
    if (getSum1 == true) {
      buttonPressDuration = buttonPressEndTime1 - buttonPressStartTime1;
    }
  }
  runOnce1 = true;
  // Serial.println(buttonPressDuration);
  // Serial.println(buttonPressCounter/1000);
  //Serial.println(buttonPressCounter);
  //(buttonPressDurationSum > buttonPressCounter  && &)

  if ((liftSet == 1) && (digitalRead(buttonLift) == HIGH) && (isnNotSet == false) && (runOnce1 == true)) {
    //   bool getTimerForBtnLiftStart = true;
    digitalWrite(buttonLiftLed, HIGH);
  }
  // if (buttonPressCounter > buttonPressDurationSum) {
  //   digitalWrite(buttonLiftLed, LOW);
  //   runOnce1 = false;
  // }
}

void Lower() {
  if ((digitalRead(buttonLower) == HIGH)) {
    digitalWrite(buttonLowerLed, HIGH);
    if (notifyToArrowDownTask == 0) {
      notifyToArrowDownTask = 1;
    }

    if (isnNotSet != false) {
      isnNotSet = false;
    }
    if (liftSet != 1) {
      liftSet = 1;
    }
    if (runOnce1 != true) {
      runOnce1 = true;
    }
  }
  if (digitalRead(buttonLower) == LOW) {
    digitalWrite(buttonLowerLed, LOW);
    if (notifyToArrowDownTask == 1) {
      notifyToArrowDownTask = 0;
      timerRunning2 = 0;
      getSum = true;
    }
  }
  if ((digitalRead(buttonLower) == HIGH)) {
    {
      getTimerForBtnLowerEnd = true;
      getSum = false;
      calculateSum = true;
    }
    if (timerRunning2 == 0 && getTimerForBtnLowerEnd == true) {
      getTimerForBtnLowerStart = true;
      doNotCount2 = false;
      timerRunning2 = 1;
    }
  }

  if (getSum == true) {
    buttonPressDuration = buttonPressEndTime2 - buttonPressStartTime2;

    if (calculateSum == true) {
      if (buttonPressDuration - buttonPressDurationlast <= 30000) {
        buttonPressDurationlast = buttonPressDuration;
      } else if (buttonPressDuration - buttonPressDurationlast > 30000) {
        buttonPressDuration = buttonPressDurationlast;
      }
    }
  }
  if (buttonPressDurationlast > 0 && digitalRead(buttonLower) == LOW) {
    if (timerRunning2 == 0) {
      buttonPressDurationSum += buttonPressDurationlast;
      buttonPressDurationlast = 0;
      calculateSum = false;
    }
  }
}

/***********************************************************************************************************************/

/*** timer that stat the clock function ***/

void runMillisTimer() {
  for (int j = 0; j < currentMin; j++) {
    if ((millis() - TimerClock > runMillis) && (State == 1)) {
      TimerClock = millis();
      S++;

      if (S > 59) {
        M++;
        currentMin = currentMin - 1;
        S = 0;
      }
    }
  }
  // int oldState = State;
  if (M == minutes && State == 1) {

    State = 0;
  }
}

String timerRunSet() {


  String Result;

  // strcat (Result, ResultOld);
  // for (int j = 0; j < currentMin; j++) {
  //   if ((millis() - TimerClock > runMillis) && (State == 1)) {
  //     // if (State == 1) {
  //     //  vTaskDelay(runMillis / portTICK_PERIOD_MS);
  //     TimerClock = millis();
  //     S++;

  //     if (S > 59) {
  //       M++;
  //       currentMin = currentMin - 1;
  //       S = 0;
  //     }
  //   }
  if (M < 10) Result = (String) "0" + M + ":";
  else Result = (String)M + ":";
  if (S < 10) Result = (String)Result + "0" + S;
  else Result = (String)Result + S;
  // }
  if (M == minutes) {
    currentMin = minutes;
    State = 0;
    M = 0;
    S = 0;
  }

  if ((State == 0) && (reset_timer == 0) && (state_pause == 1)) {
    if (M < 10) Result = (String) "0" + M + ":";
    else Result = (String)M + ":";
    if (S < 10) Result = (String)Result + "0" + S;
    else Result = (String)Result + S;
    Serial.println((String)Result);
  }


  return Result;
}

/***********************************************************************************************************************/
// }
//}

/***  setup to load the configuration + drivers + lib at start up  ***/

void setup(void) {
  Serial.begin(115200);


  u8g2.setI2CAddress(0x3F * 2);
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.begin();
  u8g2.setFontMode(1);
  u8g2.setContrast(200);
  u8g2.clearBuffer();  // clear the internal memory

  if (gauge.begin() != 0) {
    Serial.println("gauge begin faild!");
  } else {
    delay(2);
    Serial.println("gauge begin successful!");
    gauge.setInterrupt(20);  //use this to modify alert threshold as 1% - 32% (integer)
  }

  //u8g2.setDisplayRotation(U8G2_R2);
  pinMode(buttonStopReset, INPUT_PULLUP);
  pinMode(buttonStart, INPUT_PULLUP);
  pinMode(buttonEqOn, INPUT_PULLUP);
  pinMode(buttonEqOff, INPUT_PULLUP);
  pinMode(buttonPumpOn, INPUT_PULLUP);
  pinMode(buttonPumpOff, INPUT_PULLUP);
  pinMode(buttonLift, INPUT);
  pinMode(buttonLower, INPUT);

  pinMode(buttonStopResetLed, OUTPUT);
  pinMode(buttonStartLed, OUTPUT);
  pinMode(buttonEqOnLed, OUTPUT);
  // pinMode(buttonEqOffLed, OUTPUT);
  pinMode(buttonPumpOnLed, OUTPUT);
  //pinMode(buttonPumpOffLed, OUTPUT);
  pinMode(buttonLiftLed, OUTPUT);
  pinMode(buttonLowerLed, OUTPUT);
  pinMode(motorFarwardPin, OUTPUT);
  pinMode(motorBackwardPin, OUTPUT);
  pinMode(potBright, INPUT);
  pinMode(potPin, INPUT);
  adcAttachPin(potPin);
  analogSetPinAttenuation(potPin, ADC_6db);
  analogReadResolution(12);
  adcAttachPin(potBright);
  analogSetPinAttenuation(potBright, ADC_6db);

  attachInterrupt(digitalPinToInterrupt(buttonStopReset), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonStart), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonEqOn), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonEqOff), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPumpOn), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPumpOff), Buttons, FALLING);

  button1.attachClick(buttonTROff);
  button1.attachLongPressStart(resetTimer);
  button1.setPressTicks(2000);
  button1.setClickTicks(60);

  button2.attachClick(Equipment_on);
  button2.attachLongPressStart(resetLift);
  button2.setPressTicks(2000);
  button2.setClickTicks(60);

  button3.attachClick(Equipment_off);

  button4.attachClick(Punp_on);

  button5.attachClick(Punp_off);

  button6.attachClick(buttonTROn);
  delay(500);
  //     WiFi.mode(WIFI_STA);
  //   if (esp_now_init() != ESP_OK)
  //   {
  //     Serial.println("Error initializing ESP-NOW");
  //     return;
  //   }
  //   esp_now_register_send_cb(OnDataSent);
  //   esp_now_peer_info_t peerInfo;
  //   memcpy(peerInfo.peer_addr, RxMACaddress, 6);
  //   peerInfo.channel = 0;
  //   peerInfo.encrypt = false;
  //   //----------------------------------------------------------
  //   if(esp_now_add_peer(&peerInfo) != ESP_OK)
  //   {
  //     Serial.println("Failed to add peer");
  //     return;
  //   }
  //  // ----------------------------------------------------------
  //    esp_now_register_recv_cb(OnDataRecv);

  //   esp_wifi_set_promiscuous(true);
  //   esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);




  /*** this is part of the setup to load the FreeRTOS functions and select the size and which core it should run on ***/


  // xTaskCreatePinnedToCore(DrawUpArrow, "Task_arrowUp", 2048, NULL, 2, &Task_arrowUp, 0);
  //vTaskSuspend(Task_arrowUp);
  //xTaskCreatePinnedToCore(DrawDownArrow, "Task_arrowDown", 2048, NULL, 2, &Task_arrowDown, 0);
  //vTaskSuspend(Task_arrowDown);


  xTaskCreatePinnedToCore(interruptsLift, "Task_interruptsLift", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(interruptsLower, "Task_interruptLower", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(interruptsRun, "Task_interruptsRun", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ProgramRun, "Task_ProgramRun", 10000, NULL, 1, NULL, 0);


  //Drawgauge(cx, cy, radius, percent, pot, 0, 100);
  //battery();
  Timestamp_Button_Pressed = millis();
  currentMillis = millis();

/*** I run the state 1 as debug, it starts the clock when the power is applied, will not be in the final code. ***/
  State = 1;
}

/***********************************************************************************************************************/

/*** setting how the operations will work ***/
void RunTimeProg() {
  String timerRun = timerRunSet();
  setScreenBrightness();




  // if (millis() - Timestamp_Button_Pressed > 1000) {
  // Serial.println(esp_timer_get_time());
  //   Serial.println("GPIO=");
  //   Serial.print("buttonStopResetLed=");
  //   Serial.println(digitalRead(buttonStopResetLed));
  //   Serial.print(",buttonStartLed=");
  //   Serial.println(digitalRead(buttonStartLed));
  //   Serial.print(",buttonEqOnLed=");
  //   Serial.println(digitalRead(buttonEqOnLed));
  //   Serial.print(",buttonLift=");
  //   Serial.println(digitalRead(buttonLift));
  //   Serial.print(",buttonLower=");
  //   Serial.println(digitalRead(buttonLower));
  //   //Serial.print(digitalRead(buttonEqOffLed));
  //   Serial.print(",buttonPumpOnLed=");
  //   Serial.println(digitalRead(buttonPumpOnLed));
  //   //Serial.print(digitalRead(buttonPumpOffLed));
  //   Serial.print(",buttonLiftLed=");
  //   Serial.println(digitalRead(buttonLiftLed));
  //   Serial.print(",buttonLowerLed=");
  //   Serial.println(digitalRead(buttonLowerLed));
  //   Serial.print(",motorFarwardPin=");
  //   Serial.println(digitalRead(motorFarwardPin));
  //  // Serial.print("  Voltage=");
  //  // Serial.print(gauge.readVoltage());
  //   // Serial.print(" POTPOT=");
  //   // Serial.print(analogRead(potPin));
  //   // Serial.print(analogRead(potBright));
  //   Serial.print(",motorBackwardPin=");
  //   Serial.println(digitalRead(motorBackwardPin));
  //   Serial.print("RSSI = ");
  //   Serial.println(rssi_display);
  //   Serial.print(",");
  //  // readPot();
  // Timestamp_Button_Pressed = millis();
  //}
  if ((digitalRead(buttonLower) != HIGH) && (digitalRead(buttonLift) != HIGH)) {

      if (State == 0) {
        u8g2.clearBuffer();
        //   String timerRun = timerRunSet();
        battery();
        Drawgauge(cx, cy, radius, percent, pot, 0, 100);
        // if(digitalRead(buttonStopResetLed == LOW)){
        //   digitalWrite(buttonStopResetLed, HIGH);
        // }

        if (reset_timer == 1) {
          u8g2.setFont(u8g2_font_inr30_mf);
          u8g2.drawStr(2, 56, "RESET");
          u8g2.sendBuffer();
          delay(500);
          u8g2.clearBuffer();
          state_pause = 0;
        } else {
          u8g2.setFont(u8g2_font_inr30_mf);
          // u8g2.drawStr(1, 58, timerRun.c_str());
        }
        if (state_pause == 1) {
          u8g2.setFont(u8g2_font_inr24_mf);
          u8g2.drawStr(12, 42, timerRun.c_str());
          u8g2.setFont(u8g2_font_luBIS12_te);
          u8g2.drawStr(34, 56, "PAUSE");
        }
        if ((millis() - TimerClock > 500) && (state_pause != 1)) {
          //  u8g2.drawStr(12, 44, timerRun.c_str());
          //   battery();
          //   Drawgauge(cx,cy,radius,percent,pot,0,100);
          u8g2.setFont(u8g2_font_inr30_mf);
          u8g2.drawStr(2, 56, timerRun.c_str());

          reset_timer = 0;
        }

        u8g2.sendBuffer();
      }


      if ((millis() - TimerClock > 0) && (State == 1)) {
        //   //   String timerRun = timerRunSet();
        //   //Serial.println(timerRun);
        u8g2.clearBuffer();
        battery();
        Drawgauge(cx, cy, radius, percent, pot, 0, 100);
        u8g2.setFont(u8g2_font_inr30_mf);
        u8g2.drawStr(2, 56, timerRun.c_str());
        //   u8g2.sendBuffer();
      }
      if (State == 2) {
        u8g2.clearBuffer();
        battery();
        Drawgauge(cx, cy, radius, percent, pot, 0, 100);
        u8g2.setFont(u8g2_font_inr30_mf);
        u8g2.sendBuffer();
      }
      if (State == 3) {
        u8g2.clearBuffer();
        // battery();
        u8g2.setFont(u8g2_font_timB08_tr);
        u8g2.setCursor(0, 20);
        u8g2.print(soc);
        u8g2.setCursor(40, 20);
        u8g2.print(v);
        u8g2.sendBuffer();
      }
      // if (State == 4) {
      //   DrawDownArrow();
      //   DrawUpArrow();
      // }
    }
}

void interruptsProg() {
  button1.tick();
  button2.tick();
  button3.tick();
  button4.tick();
  button5.tick();
  button6.tick();
}

void ProgramRun(void *pvParameters) {

  for (;;) {  // A Task shall never return or exit.
    RunTimeProg();
  }
}

bool runOnce3 = false;
bool runOnce4 = false;

void interruptsLift(void *pvParameters) {
  //attachInterrupt(digitalPinToInterrupt(buttonLift), isr, RISING);

  for (;;) {  // A Task shall never return or exit.
    Lift();
    if (notifyToArrowUpTask == 0) {
      vTaskSuspend(Task_arrowUp);
    }
    if (notifyToArrowUpTask == 1) {
      vTaskResume(Task_arrowUp);
    }
  }
}

void interruptsLower(void *pvParameters) {
  // attachInterrupt(digitalPinToInterrupt(buttonLower), isr, RISING);

  for (;;) {  // A Task shall never return or exit.
    Lower();  //    digitalRead(buttonLowerLed);
              //  Serial.println(notifyToArrowDownTask);
    if (notifyToArrowDownTask == 0) {
      vTaskSuspend(Task_arrowDown);
    }
    if (notifyToArrowDownTask == 1) {
      vTaskResume(Task_arrowDown);
    }
   //  Serial.print(xPortGetTickCount());
  }
}



void interruptsRun(void *pvParameters) {

  for (;;) {  // A Task shall never return or exit.
    interruptsProg();
  }
}

/***********************************************************************************************************************/

/*** loop the loop ***/

void loop(void) {
  runMillisTimer();


  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (digitalRead(buttonLift) == HIGH && getTimerForBtnLiftStart == true) {
      buttonPressStartTime1 = millis();
      getTimerForBtnLiftStart = false;
      lastDebounceTime = millis();
    }

    else if (digitalRead(buttonLift) == LOW && getTimerForBtnLiftEnd == true) {
      buttonPressEndTime1 = millis();
      getTimerForBtnLiftEnd = false;
      lastDebounceTime = millis();
    }
  }


  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (digitalRead(buttonLower) == HIGH && getTimerForBtnLowerStart == true) {
      buttonPressStartTime2 = millis();
      getTimerForBtnLowerStart = false;
      lastDebounceTime = millis();
    }

    else if (digitalRead(buttonLower) == LOW && getTimerForBtnLowerEnd == true) {
      buttonPressEndTime2 = millis();
      getTimerForBtnLowerEnd = false;
      lastDebounceTime = millis();
    }
  }

   if (millis() - upDownDisplayTimer > 300) {
  //   Serial.print("B= ");
  //   Serial.println(buttonPressEndTime1);
  //   Serial.print("A= ");
  //   Serial.println(buttonPressStartTime1);
  //   Serial.print("buttonPressCounter ");
  //   Serial.println(buttonPressCounter);
  //   // Serial.print("buttonPressDurationSum");
  //   // Serial.println(buttonPressDurationSum);
   //   
  //   upDownDisplayTimer = millis();
   }
}
/***********************************************************************************************************************/


/*** this is how FreeRTOS is used to schedule tasks + this should draw arrows on the display when pressing the buttons (up/down) 
 it works and draw the arrow but after it does it function the key is released the screen doesnt go back to the clock count.. so at the moment the task set not to run ***/

void DrawUpArrow(void *pvParameters) {

  for (;;) {  // A Task shall never return or exit.
    for (int i = 5; i < sizeof arr / sizeof arr[0]; i--) {
      int number = arr[i];
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_open_iconic_arrow_8x_t);
      u8g2.drawGlyph(32, number, 83);
      u8g2.sendBuffer();
      vTaskDelay(300);
    }
  }
}
void DrawDownArrow(void *pvParameters) {
  for (;;) {
    for (int i = 0; i < sizeof arr / sizeof arr[0]; i++) {
      int number = arr[i];
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_open_iconic_arrow_8x_t);
      u8g2.drawGlyph(32, number, 80);
      u8g2.sendBuffer();
      vTaskDelay(300);
    }
  }
}


// DOR ORGANIZATION == False
/********************** main ******************************/



