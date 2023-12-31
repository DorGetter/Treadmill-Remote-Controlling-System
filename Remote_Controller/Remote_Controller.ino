/*
TODO:
SPEED BUTTONS CONFIGURATION!
*/

#include <U8g2lib.h>
#include <DFRobot_MAX17043.h>
#include <Wire.h>
#include <OneButton.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi_types.h>
#include <esp_wifi.h>
#include <wifiscan.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#define I2C_SCL 33
#define I2C_SDA 32
#define MAX17043_I2C_ADDRESS 0x36


U8G2_ST7567_ENH_DG128064I_F_SW_I2C u8g2(U8G2_R2, 2, 15, U8X8_PIN_NONE);
uint8_t broadcastAddress[] ={0x3C, 0x61, 0x05, 0x65, 0x03, 0x44};

/*
###################################################################################################
###################################### SCRIPT VARIABLES ###########################################
###################################################################################################
*/
/*variables STATUS*/

int EQUIPMENT_ON = false;
int TREADMILL_ON = false;
bool PUMP_ON = false;
int SPEED = 0;
int SECONDS=0;
bool RUNNING_TIMER=false;
bool RESET = false;
bool DONE = false;
int DIMMER = 0;

bool CHARGING = false;
int BATTERY_PER = 0; 

/*
###################################################################################################
###################################### DISPLAY DRAWERS ############################################
###################################################################################################
*/

/******Battery******/
DFRobot_MAX17043 Battery;
double soc = 0;  // Variable to keep track of LiPo state-of-charge (SOC)
long TimeWaitBeforeReading_bat = 0;
long BatteryReadingInterval = 1000; // Time interval for reading battery voltage
float battery_voltage=4100;
float battery_voltage_charge = 0;
int RSSI_SIGNAL=-100;
unsigned long LAST_PATCKET_RSST_TIME = 0;


void DrawBatteryGauge() {
  // Read battery voltage every BatteryReadingInterval ms
  if (millis() - TimeWaitBeforeReading_bat > BatteryReadingInterval) {
    battery_voltage = Battery.readVoltage();
    battery_voltage_charge = Battery.readVoltage();
    // soc = Battery.readPercentage();
    TimeWaitBeforeReading_bat = millis();
  }

  if (battery_voltage_charge > 4180){CHARGING=true;}
  else {CHARGING=false;}

  if (battery_voltage > 4150){ battery_voltage = 4150;}; // highest value of battery
  if (battery_voltage < 3000){ battery_voltage = 3000; } // lowest value of battery

  BATTERY_PER = map(battery_voltage, 3000, 4150, 0, 100);

  int x = 1;
  int y = 4;
  int width = 19;
  int height = 11;
  
  int batteryWidth = width - 4;
  int batteryHeight = height - 4;
  int batteryX = x + 2;
  int batteryY = y + 2;
  int batteryPercentageHeight = (batteryHeight * BATTERY_PER) / 100;
  
  // Draw the battery frame
  u8g2.drawFrame(x, y, width, height);
  u8g2.drawFrame(x+width, y+(height/2)-2, 3, 6);

  // Draw the battery fill
  u8g2.drawBox(batteryX, batteryY, (batteryWidth * BATTERY_PER) / 100, batteryHeight);
}
/******RSSI******/
void DrawRSSIFrames(){
    int i = 0;
    u8g2.drawFrame(58,  11, 3,  4 );
    u8g2.drawFrame(55,  8,  3,  7 );
    u8g2.drawFrame(52,  5,  3,  10);
    u8g2.drawFrame(49,  2,  3,  13);
  
}
void DrawRSSIGauge(int rssi_signal) { 
  int number_bars;
  // int number_bars = map(rssi_display, -90, -10, 0, 4);  // map input value to number of bars
  if        (rssi_signal >= -40)                            {number_bars=4;}
  else if ( (rssi_signal <  -40) && (rssi_signal >= -55))   {number_bars=3;}
  else if ( (rssi_signal <  -55) && (rssi_signal >= -67))   {number_bars=2;}
  else if ( (rssi_signal <  -67) && (rssi_signal >= -71))   {number_bars=1;}
  else if ( (rssi_signal <  -70))                           {number_bars=0;}
  int bar_width = 2;
  int bar_margin = 1;
  int x = 59;
  // Serial.println(rssi_signal);
  if (number_bars==0){ u8g2.setFont(u8g2_font_6x10_mf); u8g2.setCursor(57,6); u8g2.print("x"); }
  
  for (int i = 0; i < number_bars; i++) { 
    u8g2.drawLine(x, 13, x, 13 - (i+1)*3);
    u8g2.drawLine(x-1, 13, x-1, 13 - (i+1)*3);
    x -= (bar_width + bar_margin);
  }
}
/******Speed******/
void DrawSpeedGauge(int speed) {
  int x = 110; // Updated X-coordinate of the center of the gauge
  int y = 15; // Y-coordinate of the center of the gauge
  int r = 15; // Outer radius of the gauge
  int p = 3; // Width of the gauge's pointer
  int minVal = 0; // Minimum value of the gauge
  int maxVal = 100; // Maximum value of the gauge
  
  u8g2.drawCircle(x, y, r, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(x, y, r - p, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawLine(x - r, y, x - r + p, y);
  u8g2.drawLine(x + r - p, y, x + r, y);
  u8g2.setDrawColor(1);
  
  float val = map(speed, 0, 100, 0, 180);
  val = val * 3.14 / 180 - 1.572;
  int xp = x + (sin(val) * (r - p));
  int yp = y - (cos(val) * (r - p));
  u8g2.drawLine(x, y, xp, yp);
  u8g2.drawLine(x - 1, y - 1, xp, yp);
  u8g2.drawLine(x + 1, y - 1, xp, yp);
  u8g2.drawLine(x - 1, y, xp, yp);
  u8g2.drawLine(x + 1, y, xp, yp);
  
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(101,25);
  u8g2.print(speed);
  u8g2.drawStr(113, 25, "%");

}
/******Stopper******/
void DrawStopper(int seconds) {
  int minutes = seconds / 60;
  seconds = seconds % 60;
  u8g2.setFont(u8g2_font_inr24_mf);
  u8g2.setCursor(2, 56);
  if (minutes < 10) {
    u8g2.print("0");
  }
  u8g2.print(minutes);
  u8g2.print(":");
  if (seconds < 10) {
    u8g2.print("0");
  }
  u8g2.print(seconds);
}
/*RESET message*/
void DrawResetTimmer(bool reset) {
  if (reset){
    u8g2.setFont(u8g2_font_inr24_mf);
    u8g2.setCursor(2, 56);
    u8g2.print("RESET"); 
    }
}
/******Up&Down Arrow******/
void DrawArrows(int arrow_value, int arrow_position) {
  if (arrow_value == 1){
    if      ((arrow_position<=33)     &&  (arrow_position>=0))        {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 55, 83);}
    if ((arrow_position<=66)    &&  (arrow_position>33))        {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 50, 83);}   
    if ((arrow_position<=100)    &&  (arrow_position>66))       {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 45, 83);}                              
  }
  else if (arrow_value == -1){
    if      ((arrow_position<=100)    &&  (arrow_position>66))       {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 45, 80);}
    if ((arrow_position<=66)    &&  (arrow_position>33))        {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 50, 80);}   
    if ((arrow_position<=33)     &&  (arrow_position>=0))        {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 55, 80);}  
  }
}
void DrawDone(bool done){
  if (done){
    u8g2.setFont(u8g2_font_inr24_mf);
    u8g2.setCursor(2, 56);
    u8g2.print("END!"); 
  }
}
void DrawEquepmentOn(bool equipmentOn){
  if (equipmentOn){
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(2, 25);
    u8g2.print("ON");
  } 
  else {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(2, 25);
    u8g2.print("OFF");
  }
}
void DrawChargingSymbol(bool charge){
  if (charge){
    u8g2.setFont(u8g2_font_unifont_t_77);
    u8g2.drawGlyph(28, 13, 0x26a1);  // Unicode snowman
  }
  else{
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(25,13);
    u8g2.print(BATTERY_PER);
    u8g2.setCursor(40,13);
    u8g2.print("%");
  }
}
void DrawPumpSymbol(bool pump_working){
  if (PUMP_ON){
    u8g2.setFont(u8g2_font_unifont_t_86);
    u8g2.drawGlyph(68, 16, 0x2b59);  // Unicode snowman
  }
}

/*
###################################################################################################
###################################### MESSAGE STRUCTURES #########################################
###################################################################################################
*/

// Structure to receive data. Must match the sender structure
typedef struct struct_message {
    char messageName[32]; // can be either ['signal', 'statusUpdate',] 
    char operation[32]; // [button< name of button >Pressed, 
    float currentValue; // value of current button. (if hes numerical-->potenziometer value)
    bool currentLevel; //  value of current button. (if hes Binary--> buttonOff||buttonOn) 
} struct_message;

// Create a struct_message called myData
struct_message myData;



/*
###################################################################################################
###################################### Buttons Config #############################################
###################################################################################################
*/

// PINS: Buttons:
#define pin_buttonStopReset 0
#define pin_buttonStart 4
#define pin_buttonEqOn 16
#define pin_buttonEqOff 17
#define pin_buttonPumpOn 5
#define pin_buttonPumpOff 18
#define pin_buttonLift 35 
#define pin_buttonLower 34
#define pin_buttonSpeedUp 22 
#define pin_buttonSpeedDown 21

// PINS: ButtonsLeds:
#define pin_buttonStopResetLed 12
#define pin_buttonStartLed 27
#define pin_buttonEqOnLed 26
#define pin_buttonPumpOnLed 25
#define pin_dimmScreen 36
#define pin_buttonPumpOffLed 13
#define pin_buttonEqOffLed 14

OneButton button_1_buttonStopReset  (pin_buttonStopReset,   true);
OneButton button_2_buttonEqOn       (pin_buttonEqOn,        true);
OneButton button_3_buttonEqOff      (pin_buttonEqOff,       true);
OneButton button_4_buttonPumpOn     (pin_buttonPumpOn,      true);
OneButton button_5_buttonPumpOff    (pin_buttonPumpOff,     true);
OneButton button_6_buttonStart      (pin_buttonStart,       true);
OneButton button_7_buttonLift       (pin_buttonLift,        false);
OneButton button_8_buttonLower      (pin_buttonLower,       false);
OneButton button_9_buttonSpeedUp    (pin_buttonSpeedUp,     true);
OneButton button_10_buttonSpeedDown (pin_buttonSpeedDown,   true);

int ARROW_VALUE = 3;
int COUNTER_ARROW_POSITION = 0;

void sender_helper(String message_name, String operation, float currentValue, bool currentLevel){
  // In charge of sending triggers to the premise system. 
  // Set values to send
  strcpy(myData.messageName, message_name.c_str());
  strcpy(myData.operation, operation.c_str());
  myData.currentValue = currentValue;
  myData.currentLevel = currentLevel;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}

void button_Equipment_on() {
  Serial.println("Equipment on pressed!");
  sender_helper("status", "EquipmentOn", 0, true);
}
void button_Equipment_off() {
  Serial.println("Equipment off pressed!");
  sender_helper("status", "EquipmentOn", 0, false);
}
void button_TROn() {
  if (EQUIPMENT_ON) {
    Serial.println("button treadmill On pressed");
    sender_helper("status", "TreadmillOn", 0, true);
  }
}
void button_TROnLongPress() {
  if (EQUIPMENT_ON) {
    Serial.println("buttonTROnLongPress pressed");
    sender_helper("status", "TreadmillOn", 0, true);
  }
}
void button_TROff() {
  if (EQUIPMENT_ON) {
      sender_helper("status", "TreadmillStop", 0, true);

  }
}
void button_TROffLongPress() {
  // Serial.println("button treadmill off Long Press pressed");
  if (EQUIPMENT_ON) {
    sender_helper("status", "TreadmillReset", 0, true);
    RESET = true;
  }
}
void button_TROffLongPressStop(){
  // Serial.println("buttonTROffLongPressStop pressed");
  if (EQUIPMENT_ON) {
    delay(2000);
    RESET = false;
  }
}
void button_Pump_on() {
  Serial.println("Pump_on pressed!");
  if (EQUIPMENT_ON) {
    sender_helper("status", "PumpOn", 0, true);
  }
}
void button_Pump_off() {
  Serial.println("pump_off pressed!");
  if (EQUIPMENT_ON) {
    sender_helper("status", "PumpOn", 0, false);
  }
}
void button_liftUpLongPress(){
  if (EQUIPMENT_ON){
    COUNTER_ARROW_POSITION = 0;
    ARROW_VALUE =1;
    COUNTER_ARROW_POSITION +=1;
    sender_helper("status", "LiftUP", 0, true); // TODO: change to LiftUP
  }
}
void button_lowerDownLongPress(){
  if (EQUIPMENT_ON){
    COUNTER_ARROW_POSITION = 100;
    ARROW_VALUE=-1;
    COUNTER_ARROW_POSITION -=1;
    sender_helper("status", "LiftRev", 0, true); // TODO: change to LiftRev
  }
}
void button_liftUpRelease(){
  // if (EQUIPMENT_ON){
  ARROW_VALUE=3;
  sender_helper("status", "LiftStop", 0, true); // TODO: change to LiftStop
  sender_helper("status", "LiftStop", 0, true); // TODO: change to LiftStop
  // }
}
void button_lowerDownRelease(){
  // if (EQUIPMENT_ON){
  ARROW_VALUE=3;
  sender_helper("status", "LiftStop", 0, true); // TODO: change to LiftStop
  sender_helper("status", "LiftStop", 0, true); // TODO: change to LiftStop
  // }
}
// ----------------------------------------------------- <<< added:
void button_SpeedUpLongPress(){
  Serial.println("pressed!");

  if (EQUIPMENT_ON & TREADMILL_ON & !DONE) {
    ARROW_VALUE =1;
    COUNTER_ARROW_POSITION +=1;
    sender_helper("status", "SpeedUP", 0, true); // TODO: change to LiftUP
  }
}
void button_SpeedDownLongPress(){
  Serial.println("pressed! Down!");

  if (EQUIPMENT_ON & TREADMILL_ON & !DONE){
    ARROW_VALUE=-1;
    COUNTER_ARROW_POSITION -=1;
    sender_helper("status", "SpeedRev", 0, true); // TODO: change to LiftRev
  }
}
void button_SpeedUpRelease(){
  Serial.println("release speed!");
  if (EQUIPMENT_ON & TREADMILL_ON & !DONE) {
  ARROW_VALUE=3;
  sender_helper("status", "SpeedStop", 0, true); // TODO: change to LiftStop
  }
}
void button_SpeedDownRelease(){
  Serial.println("release speed!");
  if (EQUIPMENT_ON & TREADMILL_ON & !DONE) {
  ARROW_VALUE=3;
  sender_helper("status", "SpeedStop", 0, true); // TODO: change to LiftStop
  }
}




/*
###################################################################################################
###################################### FUNCTIONALITY- STATUS  #####################################
###################################################################################################
*/


void EquipmentOn_status(bool val){
  // Serial.print("EquipmentOn_status:    "); Serial.print(val);
  if (EQUIPMENT_ON == val){return;}
  else{
    EQUIPMENT_ON = val;
    if (val){
      digitalWrite(pin_buttonEqOnLed, HIGH);
      digitalWrite(pin_buttonEqOffLed, LOW);
    }
    else {
      digitalWrite(pin_buttonEqOffLed, HIGH);
      digitalWrite(pin_buttonPumpOffLed, HIGH); 
      digitalWrite(pin_buttonStopResetLed, HIGH);

      digitalWrite(pin_buttonEqOnLed, LOW);
      digitalWrite(pin_buttonStartLed, LOW); 
      digitalWrite(pin_buttonPumpOnLed, LOW); 
    }
  }
}
void TreadmillOn_status(bool val){
  // Serial.print("  TreadmillOn_status:    "); Serial.print(val);
  if (EQUIPMENT_ON) {
    if (TREADMILL_ON == val){return;}
    else {
      TREADMILL_ON = val;
      if (val){
      digitalWrite(pin_buttonStartLed,      HIGH);
      digitalWrite(pin_buttonStopResetLed,  LOW);
      }
      else{
        digitalWrite(pin_buttonStartLed,      LOW);
        digitalWrite(pin_buttonStopResetLed,  HIGH);
      }
    }
  }else{
    digitalWrite(pin_buttonStartLed,      LOW);
    digitalWrite(pin_buttonStopResetLed,  HIGH);
  }
}
void PunpOn_status(bool val){
  // Serial.print(" PunpOn_status:    "); Serial.println(val);
  if (EQUIPMENT_ON) {
    if (PUMP_ON == val){}
    else {
      PUMP_ON = val;
      if (val){
        digitalWrite(pin_buttonPumpOnLed, HIGH);
        digitalWrite(pin_buttonPumpOffLed, LOW);
      }
      else{
        digitalWrite(pin_buttonPumpOnLed, LOW);
        digitalWrite(pin_buttonPumpOffLed, HIGH);
      }
    }
  }else{
      digitalWrite(pin_buttonPumpOnLed, LOW);
      digitalWrite(pin_buttonPumpOffLed, HIGH);
  }
}


/*
###################################################################################################
########################################## HANDLERS ###############################################
###################################################################################################
*/


/*** this section relate to change brighness of the display - at the moment not connected to the pot ***/
void ScreenBrightnessHandler() {
  while(true){
    DIMMER =  map(analogRead(pin_dimmScreen), 0, 4095, 150, 255);
    delay(75);
  }
}

void interruptsProg() {
  button_1_buttonStopReset.tick();    button_2_buttonEqOn.tick();       button_3_buttonEqOff.tick();
  button_4_buttonPumpOn.tick();       button_5_buttonPumpOff.tick();    button_6_buttonStart.tick();
  button_7_buttonLift.tick();         button_8_buttonLower.tick();      button_9_buttonSpeedUp.tick();
  button_10_buttonSpeedDown.tick();   
  }


void interruptsRun(void *pvParameters) {
  bool running = true;
  while (running) {
    interruptsProg();
    delay(30);
  }
}

/////////////////////////////////////// Comunication Handlers 
typedef struct {
  unsigned frame_ctrl:16;
  unsigned duration_id:16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl:16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

// RSSI connection signal.
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
    if (type != WIFI_PKT_MGMT)
        return;

    static const uint8_t ACTION_SUBTYPE = 0xD0;
    static const uint8_t ESPRESSIF_OUI[] = {0x18, 0xFE, 0x34};

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    if (ACTION_SUBTYPE == (hdr->frame_ctrl & 0xFF)) {
        const uint8_t *payload = ipkt->payload;
        const uint8_t *end = payload + ppkt->rx_ctrl.sig_len;

        while (payload < end) {
            uint8_t element_id = payload[0];
            uint8_t element_length = payload[1];

            if (element_id == 0xDD && element_length >= 3 && memcmp(payload + 2, ESPRESSIF_OUI, 3) == 0) {
                RSSI_SIGNAL = ppkt->rx_ctrl.rssi;
                LAST_PATCKET_RSST_TIME = millis();
                break;
            }

            payload += element_length + 2;
        }
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}
unsigned long lastStatusReceivedTime = 0;
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // Check if myData.a is equal to "signal Testing"
  if (strcmp(myData.messageName, "signal") == 0) {
        SPEED = myData.currentValue;
        lastStatusReceivedTime = millis(); // Update the timestamp of the last status received
  } 
  else if (strcmp(myData.messageName, "timer") == 0)          {SECONDS = myData.currentValue;}
  else if (strcmp(myData.messageName, "status") == 0){
    if      (strcmp(myData.operation, "Equipment_on")   == 0)    {EquipmentOn_status(myData.currentLevel);}
    else if (strcmp(myData.operation, "Treadmill_On")   == 0)    {TreadmillOn_status(myData.currentLevel);}
    else if (strcmp(myData.operation, "Pump_On")        == 0)    {PunpOn_status(myData.currentLevel);}
    else if (strcmp(myData.operation, "Time_Out")   == 0)        {DONE= myData.currentLevel;}

  }
}  

void LogicsHandler(){
  while(true){
    /*Configuration of variables end cases */
    if (SPEED > 100) { SPEED = 100; } else if (SPEED <= 0)  { SPEED = 0;}
    if (COUNTER_ARROW_POSITION > 100) { COUNTER_ARROW_POSITION = 0;} else if (COUNTER_ARROW_POSITION < 0)  { COUNTER_ARROW_POSITION = 100;}
    if (millis() - LAST_PATCKET_RSST_TIME > 1000) { RSSI_SIGNAL = -100;  LAST_PATCKET_RSST_TIME = millis();  } 
    if (millis() - lastStatusReceivedTime > 500) { Serial.println("not getting status!"); }
    delay(75);
  }
}

// int  lowest_memory_usage = 9999999;
// int  highest_memory_usage = 0;
// void MemoryCheck(){
//   while(true){
//     Serial.print("lowest memory usage: "); Serial.print(lowest_memory_usage);
//     Serial.print("   highest memory usage: "); Serial.println(highest_memory_usage);
//     delay(2000);
//   }
// }


/*
###################################################################################################
########################################## SETUP ##################################################
###################################################################################################
*/
void setup(void) {
  Serial.begin(115200);

  
  ////////////////// Display configurations ------------------------------------------------------------------
  u8g2.setI2CAddress(0x3F * 2);
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.begin();

  ////////////////// Battery Configurations ------------------------------------------------------------------
  Wire.begin(32,33);
  if (Battery.begin() != 0){ Serial.println("Battery Gauge Failed!!!!"); }
  else { Serial.println("Battery Gauge begin succesfully!"); Battery.setInterrupt(20); }

  ////////////////// Communication Configurations ------------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); return; }
  esp_now_peer_info_t peerInfo;
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 9);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if(esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); return; }
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);


  ////////////////// DIMMER configurations ------------------------------------------------------------------
  pinMode(pin_dimmScreen, INPUT);
  
  ////////////////// Buttons configurations ------------------------------------------------------------------
  pinMode(pin_buttonStopReset, INPUT_PULLUP);
  pinMode(pin_buttonStart, INPUT_PULLUP);
  pinMode(pin_buttonEqOn, INPUT_PULLUP);
  pinMode(pin_buttonEqOff, INPUT_PULLUP);
  pinMode(pin_buttonPumpOn, INPUT_PULLUP);
  pinMode(pin_buttonPumpOff, INPUT_PULLUP);
  pinMode(pin_buttonLift, INPUT);
  pinMode(pin_buttonLower, INPUT);
  pinMode(pin_buttonSpeedUp, INPUT_PULLUP);
  pinMode(pin_buttonSpeedDown, INPUT_PULLUP);

  pinMode(pin_buttonStopResetLed, OUTPUT);
  pinMode(pin_buttonStartLed, OUTPUT);
  pinMode(pin_buttonEqOnLed, OUTPUT);
  pinMode(pin_buttonEqOffLed, OUTPUT);
  pinMode(pin_buttonPumpOnLed, OUTPUT);
  pinMode(pin_buttonPumpOffLed, OUTPUT);

  button_1_buttonStopReset.attachClick(button_TROff);
  button_1_buttonStopReset.attachLongPressStart(button_TROffLongPress);
  button_1_buttonStopReset.setPressTicks(2000);
  button_1_buttonStopReset.setClickTicks(60);
  button_1_buttonStopReset.attachLongPressStop(button_TROffLongPressStop);

  button_2_buttonEqOn.attachClick(button_Equipment_on);
  button_3_buttonEqOff.attachClick(button_Equipment_off);
  button_4_buttonPumpOn.attachClick(button_Pump_on);
  button_5_buttonPumpOff.attachClick(button_Pump_off);

  button_6_buttonStart.attachClick(button_TROn);
  button_6_buttonStart.setPressTicks(300);	
  button_6_buttonStart.attachDuringLongPress(button_TROnLongPress);

  button_7_buttonLift.attachClick(button_liftUpLongPress);
  button_7_buttonLift.attachDuringLongPress(button_liftUpLongPress);  
  button_7_buttonLift.setPressTicks(0);	
  button_7_buttonLift.attachLongPressStop(button_liftUpRelease);

  button_8_buttonLower.attachClick(button_lowerDownLongPress);
  button_8_buttonLower.attachDuringLongPress(button_lowerDownLongPress);
  button_8_buttonLower.setPressTicks(0);	
  button_8_buttonLower.attachLongPressStop(button_lowerDownRelease);


  button_9_buttonSpeedUp.attachClick(button_SpeedUpLongPress);
  button_9_buttonSpeedUp.attachDuringLongPress(button_SpeedUpLongPress);  
  button_9_buttonSpeedUp.setPressTicks(0);	
  button_9_buttonSpeedUp.attachLongPressStop(button_SpeedUpRelease);

  button_10_buttonSpeedDown.attachClick(button_SpeedDownLongPress);
  button_10_buttonSpeedDown.attachDuringLongPress(button_SpeedDownLongPress);
  button_10_buttonSpeedDown.setPressTicks(0);	
  button_10_buttonSpeedDown.attachLongPressStop(button_SpeedDownRelease);

  ////////////////// Processes configurations ------------------------------------------------------------------
  xTaskCreatePinnedToCore((TaskFunction_t)&interruptsRun, "interruptsRun", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore((TaskFunction_t)&ScreenBrightnessHandler, "screenDimmer", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore((TaskFunction_t)&LogicsHandler, "Logics", 4096, NULL, 1, NULL, 1);
  // xTaskCreatePinnedToCore((TaskFunction_t)&MemoryCheck, "MemoryCheck", 1024, NULL, 1, NULL, 1);

  // initialize leds as off.!
  digitalWrite(pin_buttonEqOffLed, HIGH);
  digitalWrite(pin_buttonPumpOffLed, HIGH); 
  digitalWrite(pin_buttonStopResetLed, HIGH);
}

 
/*
###################################################################################################
########################################## LOOP ###################################################
###################################################################################################
*/

void loop(void) {  
  u8g2.clearBuffer();  // Clear the display buffer
 
  u8g2.setContrast(DIMMER);
  DrawSpeedGauge(SPEED);
  DrawBatteryGauge();
  DrawRSSIFrames();
  DrawRSSIGauge(RSSI_SIGNAL); 
  DrawArrows(ARROW_VALUE, COUNTER_ARROW_POSITION);
  DrawChargingSymbol(CHARGING) ;  
  DrawPumpSymbol(PUMP_ON);
  DrawEquepmentOn(EQUIPMENT_ON);
  if (DONE){
    DrawDone(DONE) ;    
  }
  else if (RESET){
    DrawResetTimmer(RESET);
  }
  else {
  DrawStopper(SECONDS);
  }
  
  u8g2.sendBuffer();  // Send the buffer to the display
  delay(200);  // Delay for 1 second before clearing the display
  u8g2.clearBuffer();  // Clear the display buffer again

  // int current_memory = ESP.getFreeHeap();
  // if (current_memory < lowest_memory_usage)   {lowest_memory_usage = current_memory;}
  // if (current_memory > highest_memory_usage)  {highest_memory_usage = current_memory;}
}