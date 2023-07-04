/*
TODO:
define pin_buttonPumpOffLed, pin_buttonEqOffLed
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


U8G2_ST7567_ENH_DG128064I_F_SW_I2C u8g2(U8G2_R2, 2, 15, U8X8_PIN_NONE);
 uint8_t broadcastAddress[] ={0x3C, 0x61, 0x05, 0x65, 0x03, 0x44};

// uint8_t broadcastAddress[] = { 0x54, 0x43, 0xB2, 0xA9, 0x1E, 0x90 };  // first ESP
// uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x77, 0x7A, 0xFC}; // This MAC adress Remote System

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
int RSSI_SIGNAL=-100;
unsigned long LAST_PATCKET_RSST_TIME = 0;


void DrawBatteryGauge() {
  // Read battery voltage every BatteryReadingInterval ms
  if (millis() - TimeWaitBeforeReading_bat > BatteryReadingInterval) {
    battery_voltage = Battery.readVoltage();
    soc = Battery.readPercentage();
    //  Serial.println("battery voltage:    ");
    //  Serial.println("");
    //  Serial.print(battery_voltage);
    //  Serial.println("battery percentage:   ");
    //  Serial.println("");
    //  Serial.print(soc);
    TimeWaitBeforeReading_bat = millis();
  }

  if (battery_voltage > 4100){ battery_voltage = 4100; } // highest value of battery
  if (battery_voltage < 3000){ battery_voltage = 3000; } // lowest value of battery

  int battery_percentage = map(battery_voltage, 3000, 4100, 0, 100);

  int x = 1;
  int y = 4;
  int width = 19;
  int height = 11;
  
  int batteryWidth = width - 4;
  int batteryHeight = height - 4;
  int batteryX = x + 2;
  int batteryY = y + 2;
  int batteryPercentageHeight = (batteryHeight * battery_percentage) / 100;
  
  // Draw the battery frame
  u8g2.drawFrame(x, y, width, height);
  u8g2.drawFrame(x+width, y+(height/2)-2, 3, 6);

  // Draw the battery fill
  u8g2.drawBox(batteryX, batteryY, (batteryWidth * battery_percentage) / 100, batteryHeight);
}

/******RSSI******/
void DrawRSSIFrames(){
    int i = 0;
    u8g2.drawFrame(35,  11, 3,  4 );
    u8g2.drawFrame(32,  8,  3,  7 );
    u8g2.drawFrame(29,  5,  3,  10);
    u8g2.drawFrame(26,  2,  3,  13);
  
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
  int x = 36;
  // Serial.println(rssi_signal);
  if (number_bars==0){ u8g2.setFont(u8g2_font_6x10_mf); u8g2.setCursor(34,6); u8g2.print("x"); }
  
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

  int speed_text_x = x-r-(6*5)-(6*4);
  int speed_val_x = x-r-(6*3);
  u8g2.drawStr(speed_text_x, y, "Speed:");
  u8g2.setCursor(speed_val_x,y);
  u8g2.print(speed);
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
    if      ((arrow_position<=10)   &&  (arrow_position>=0))      {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 55, 83);}
    else if ((arrow_position<=20)   &&  (arrow_position>11))      {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 50, 83);}   
    else if ((arrow_position<=30)  &&  (arrow_position>21))      {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 45, 83);}                              
  }
  else if (arrow_value == -1){
    if      ((arrow_position<=30)  &&  (arrow_position>21))      {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 45, 80);}
    else if ((arrow_position<=20)   &&  (arrow_position>11))      {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 50, 80);}   
    else if ((arrow_position<=10)   &&  (arrow_position>=0))      {u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);  u8g2.drawGlyph(109, 55, 80);}  
  }
}

void DrawDone(bool done){
  if (done){
    u8g2.setFont(u8g2_font_inr24_mf);
    u8g2.setCursor(2, 56);
    u8g2.print("END !"); 
    // Serial.println("DONE DRAW");
  }
}

void DrawEquepmentOn(bool equipmentOn){
  if (equipmentOn){
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(2, 25);
    u8g2.print("ON");
  } 
}



/*
###################################################################################################
###################################### FUNCTIONALITY  #############################################
###################################################################################################
*/

void sendOnChange(String what_change, String value_change){
    Serial.println("Equipment_on pressed!");
}

/*
###################################################################################################
###################################### MESSAGE STRUCTURES #########################################
###################################################################################################
*/

// Structure example to receive data
// Must match the sender structure
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

// PINS: ButtonsLeds:
#define pin_buttonStopResetLed 12
#define pin_buttonStartLed 27
#define pin_buttonEqOnLed 26
#define pin_buttonPumpOnLed 25
#define pin_dimmScreen 36
#define pin_buttonPumpOffLed 13
#define pin_buttonEqOffLed 14

OneButton button_1_buttonStopReset  (pin_buttonStopReset, true);
OneButton button_2_buttonEqOn       (pin_buttonEqOn,      true);
OneButton button_3_buttonEqOff      (pin_buttonEqOff,     true);
OneButton button_4_buttonPumpOn     (pin_buttonPumpOn,    true);
OneButton button_5_buttonPumpOff    (pin_buttonPumpOff,   true);
OneButton button_6_buttonStart      (pin_buttonStart,     true);
OneButton button_7_buttonLift       (pin_buttonLift,      false);
OneButton button_8_buttonLower      (pin_buttonLower,     false);




/*** for timming of the switches (long press / short press) + the lib for the buttons ***/
void IRAM_ATTR isr() {}
void isr();

void IRAM_ATTR Buttons() {
  // include all buttons here to be checked
  button_1_buttonStopReset.tick();  // just call tick() to check the state.
  button_2_buttonEqOn.tick();
  button_3_buttonEqOff.tick();
  button_4_buttonPumpOn.tick();
  button_5_buttonPumpOff.tick();
  button_6_buttonStart.tick();
  button_7_buttonLift.tick();
  button_8_buttonLower.tick();

}

/*variables buttons*/

int EQuipmentOn = false;
int ARROW_VALUE = 3;
int SPEED = 0;
int COUNTER_ARROW_POSITION = 0;
int SECONDS=0;
int RUNNING_TIMER=false;
int RESET = false;
int DONE = false;
int DIMMER = 0;


void sender_helper(String message_name, String operation, float currentValue, bool currentLevel){
 // Set values to send
    strcpy(myData.messageName, message_name.c_str());
  strcpy(myData.operation, operation.c_str());
  myData.currentValue = currentValue;
  myData.currentLevel = currentLevel;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}

void Equipment_on() {
  digitalWrite(pin_buttonEqOnLed, HIGH);
  digitalWrite(pin_buttonEqOffLed, LOW);

  EQuipmentOn =true;
  Serial.println("Equipment_on pressed!");

  // Set values to send
  strcpy(myData.messageName, "status");   strcpy(myData.operation, "Equipment_on");
  myData.currentValue = 0.0;              myData.currentLevel = EQuipmentOn;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
}

void Equipment_off() {
  digitalWrite(pin_buttonEqOffLed, HIGH);
  digitalWrite(pin_buttonEqOnLed, LOW);

  // shut all down
  EQuipmentOn = false;
  RUNNING_TIMER =false;
  SECONDS=0;
  // all buttons off:
  digitalWrite(pin_buttonPumpOffLed, HIGH); 
  digitalWrite(pin_buttonStopResetLed, HIGH);

  digitalWrite(pin_buttonStartLed, LOW); 
  digitalWrite(pin_buttonEqOnLed, LOW); 
  digitalWrite(pin_buttonPumpOnLed, LOW); 
  digitalWrite(pin_dimmScreen, LOW); 

  // Set values to send
  strcpy(myData.messageName, "status");   strcpy(myData.operation, "Equipment_on");
  myData.currentValue = 0.0;              myData.currentLevel = EQuipmentOn;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
    Serial.println("Equipment_off pressed!");

}

void buttonTROff() {
  if (EQuipmentOn) {
  digitalWrite(pin_buttonStopResetLed, HIGH);
  digitalWrite(pin_buttonStartLed,  LOW);
  Serial.println("buttonTROff pressed");
  RUNNING_TIMER=false;

      // Set values to send
    strcpy(myData.messageName, "status");   strcpy(myData.operation, "TraidmilOn");
    myData.currentValue =SECONDS;              myData.currentLevel = RUNNING_TIMER;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
  }
}

void buttonTROffLongPress() {
  if (EQuipmentOn) {
    digitalWrite(pin_buttonStopResetLed, HIGH);
    digitalWrite(pin_buttonStartLed,  LOW);
    RESET  = true;
    RUNNING_TIMER=false;
    delay(2000);
    Serial.println("buttonTROffLongPress pressed");
  }
}

void buttonTROffLongPressStop(){
  if (EQuipmentOn) {
  digitalWrite(pin_buttonStopResetLed, LOW);
  SECONDS = 0;
  RESET  = false;
  DONE = false;
  sender_helper("status", "TraidmilOn", SECONDS, RUNNING_TIMER);
  Serial.println("buttonTROffLongPressStop pressed");

  }
}

void buttonTROn() {
  if (EQuipmentOn) {
    digitalWrite(pin_buttonStartLed,      HIGH);
    digitalWrite(pin_buttonStopResetLed,  LOW);
    RUNNING_TIMER=true;

      // Set values to send
    strcpy(myData.messageName, "status");   strcpy(myData.operation, "TraidmilOn");
    myData.currentValue =SECONDS;              myData.currentLevel = RUNNING_TIMER;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    Serial.println("buttonTROn pressed");

  }
}

void buttonTROnLongPress() {
  if (EQuipmentOn) {
    Serial.println("buttonTROnLongPress pressed");
  }
}

void Punp_on() {
  if (EQuipmentOn) {
  digitalWrite(pin_buttonPumpOnLed, HIGH);
  digitalWrite(pin_buttonPumpOffLed, LOW);
  Serial.println("Pump_on pressed!");

  }
}

void Pump_off() {
  if (EQuipmentOn) {
  digitalWrite(pin_buttonPumpOffLed, HIGH);
  digitalWrite(pin_buttonPumpOnLed, LOW);

  Serial.println("pump_off pressed!");
  }
}

void liftUpLongPress(){
  if (EQuipmentOn) {
  ARROW_VALUE =1;
  COUNTER_ARROW_POSITION +=1;
  sender_helper("status", "SpeedUP", 0, true);
  }
}

void lowerDownLongPress(){
  if (EQuipmentOn) {
  ARROW_VALUE=-1;
  COUNTER_ARROW_POSITION -=1;
  sender_helper("status", "SpeedRev", 0, true);
  }
}

void liftUpRelease(){
  if (EQuipmentOn) {
  Serial.print("releaseeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee!!!!!____>>>>>");
  ARROW_VALUE=3;
  sender_helper("status", "SpeedStop", 0, true);
  }
}

void lowerDownRelease(){
  if (EQuipmentOn) {
  ARROW_VALUE=3;
    sender_helper("status", "SpeedStop", 0, true);
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
    delay(50);
  }
}

void StopperHandler(void* pvParameter) {
  while (true) {
    if (RUNNING_TIMER){ SECONDS+=1; delay(1000);}
    else {delay(100);} // Update every 100ms
  }
}

void interruptsProg() {
  button_1_buttonStopReset.tick();   button_2_buttonEqOn.tick();   button_3_buttonEqOff.tick();
  button_4_buttonPumpOn.tick();   button_5_buttonPumpOff.tick();   button_6_buttonStart.tick();
  button_7_buttonLift.tick();   button_8_buttonLower.tick();   delay(100);
  }


void interruptsRun(void *pvParameters) {
  bool running = true;
  while (running) {
    interruptsProg();
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
                // Serial.println(RSSI_SIGNAL);
                break;
            }

            payload += element_length + 2;
        }
    }
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
   Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

unsigned long lastStatusReceivedTime = 0;
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
   memcpy(&myData, incomingData, sizeof(myData));
    Serial.println("getting data!");

  // Check if myData.a is equal to "signal Testing"
  if (strcmp(myData.messageName, "signal") == 0) {
        Serial.print("----------->");
        Serial.print(myData.currentLevel);
        lastStatusReceivedTime = millis(); // Update the timestamp of the last status received
        SPEED = myData.currentValue ;
  } 
  if (strcmp(myData.messageName, "status") == 0){
    Serial.print("operation: ");
    Serial.println(myData.operation);
    if      (strcmp(myData.operation, "Equipment_on") == 0) {Equipment_on();}
    else if (strcmp(myData.operation, "Equipment_off") == 0){Equipment_off();}
    else if (strcmp(myData.operation, "TraidmilOn") == 0) {buttonTROn();}
    else if (strcmp(myData.operation, "TraidmilOff") == 0) {buttonTROff();}
    else if (strcmp(myData.operation, "Pump_on") == 0) {Punp_on();}
    else if (strcmp(myData.operation, "Pump_off") == 0) {Pump_off();}
    else if (strcmp(myData.operation, "liftUp") == 0) {liftUpLongPress();liftUpRelease();}
    else if (strcmp(myData.operation, "liftDown") == 0) {lowerDownLongPress(); lowerDownRelease();}

  }
  // Check if the last status was not received within the timeout
}  


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

  pinMode(pin_buttonStopResetLed, OUTPUT);
  pinMode(pin_buttonStartLed, OUTPUT);
  pinMode(pin_buttonEqOnLed, OUTPUT);
  pinMode(pin_buttonEqOffLed, OUTPUT);
  pinMode(pin_buttonPumpOnLed, OUTPUT);
  pinMode(pin_buttonPumpOffLed, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pin_buttonStopReset), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_buttonStart), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_buttonEqOn), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_buttonEqOff), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_buttonPumpOn), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_buttonPumpOff), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_buttonLift), Buttons, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_buttonLower), Buttons, FALLING);

  button_1_buttonStopReset.attachClick(buttonTROff);
  button_1_buttonStopReset.attachLongPressStart(buttonTROffLongPress);
  button_1_buttonStopReset.setPressTicks(2000);
  button_1_buttonStopReset.setClickTicks(60);
  button_1_buttonStopReset.attachLongPressStop(buttonTROffLongPressStop);

  button_2_buttonEqOn.attachClick(Equipment_on);
  button_3_buttonEqOff.attachClick(Equipment_off);
  button_4_buttonPumpOn.attachClick(Punp_on);
  button_5_buttonPumpOff.attachClick(Pump_off);

  button_6_buttonStart.attachClick(buttonTROn);
  button_6_buttonStart.setPressTicks(300);	
  button_6_buttonStart.attachDuringLongPress(buttonTROnLongPress);

  button_7_buttonLift.attachClick(liftUpLongPress);
  button_7_buttonLift.attachDuringLongPress(liftUpLongPress);  
  button_7_buttonLift.setPressTicks(0);	
  button_7_buttonLift.attachLongPressStop(liftUpRelease);

  button_8_buttonLower.attachClick(lowerDownLongPress);
  button_8_buttonLower.attachDuringLongPress(lowerDownLongPress);
  button_8_buttonLower.setPressTicks(0);	
  button_8_buttonLower.attachLongPressStop(lowerDownRelease);

  ////////////////// Processes configurations ------------------------------------------------------------------
  xTaskCreatePinnedToCore((TaskFunction_t)&StopperHandler, "StopperHandler", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore((TaskFunction_t)&interruptsRun, "interruptsRun", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore((TaskFunction_t)&ScreenBrightnessHandler, "screenDimmer", 2048, NULL, 1, NULL, 1);
  
 }

 
/*
###################################################################################################
########################################## LOOP ###################################################
###################################################################################################
*/
void loop(void) {
  /*Configuration of variables end cases */
  if (SPEED > 100) { SPEED = 100; } else if (SPEED <= 0)  { SPEED = 0;}
  if (COUNTER_ARROW_POSITION > 30) { COUNTER_ARROW_POSITION = 0;} else if (COUNTER_ARROW_POSITION < 0)  { COUNTER_ARROW_POSITION = 30;}
  if (SECONDS >= 1800) {DONE = true; SECONDS=0; RUNNING_TIMER=false; Serial.println("over time done!");}
  if (millis() - LAST_PATCKET_RSST_TIME > 1000) { RSSI_SIGNAL = -100;  LAST_PATCKET_RSST_TIME = millis();  } 
  if (millis() - lastStatusReceivedTime > 500) { Serial.println("not getting status!"); }

  /*Drawing pages*/
  u8g2.firstPage();
    do {
      u8g2.setContrast(DIMMER);
      DrawSpeedGauge(SPEED);
      DrawStopper(SECONDS);
      DrawBatteryGauge();
      DrawRSSIFrames();
      DrawRSSIGauge(RSSI_SIGNAL); 
      DrawResetTimmer(RESET);
      DrawArrows(ARROW_VALUE, COUNTER_ARROW_POSITION);
      DrawDone(DONE) ;    
      DrawEquepmentOn(EQuipmentOn);
    } while (u8g2.nextPage());
}