

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t RxMACaddress[] = { 0x54, 0x43, 0xB2, 0xA9, 0x1E, 0x90 };  // first ESP
uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x77, 0x7A, 0xFC}; // Second ESP

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32MX1508.h> // Motor Driver lib



/*
###################################################################################################
###################################### Pins Config #############################################
###################################################################################################
*/


/////////////////////////// INPUT FROM THE CONTROLLER ///////////////////
#define pin_Eq_On_Led_IN 12
#define pin_TreadMill_On_Led_IN 14
#define pin_Pump_On_Led_IN 27

/////////////////////////// INPUT FROM UNIT /////////////////////////////
#define pin_Lift_Treadmill_IN 26
#define pin_Lower_Treadmill_IN 25


/////////////////////////// OUTPUT TO THE CONTROLLER ///////////////////
#define pin_Eq_On_OUT 13
#define pin_Eq_Off_OUT 15

#define pin_TreadMill_On_OUT 2
#define pin_TreadMill_Off_OUT 4

#define pin_Pump_On_OUT 16
#define pin_Pump_Off_OUT 17

#define pin_Lift_Treadmill_OUT 5
#define pin_Lower_Treadmill_OUT 18
 
/////////////////////////// Motor /////////////////////////////////////
#define pin_Motor_L 22
#define pin_Motor_R 19
#define CH1 0                   // 16 Channels (0-15) are availible
#define CH2 1  

#define pin_potenziometer 32

MX1508 motor(pin_Motor_L, pin_Motor_R, CH1, CH2);








/****************************************************************************************************
*****************************************************************************************************
*********************************** STATUS VARIALBLES ***********************************************
******************************************************************************************************/
int EQuipmentOn = false;
int SPEED = 0;
int PUMP_ON = 0;
int SECONDS = 0;
int RUNNING_TIMER = false;
float POT_STATUS = 0;


bool EQUIPMENT_ON_LED = false;
bool TREADMILL_ON_LED = false;
bool PUMP_ON_LED = false;
bool LIFT_UP_BUTTON = false;
bool LIFT_DOWN_BUTTON = false;



esp_now_peer_info_t peerInfo;

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

// Handlers variables//


/****************************************************************************************************
*****************************************************************************************************
*********************************** HANDLERS ********************************************************
******************************************************************************************************/
void EQUIPMENT_ON_listner(){
    // Set values to send
  strcpy(myData.messageName, "status");

  if (EQUIPMENT_ON_LED) {strcpy(myData.operation, "Equipment_on");} else {strcpy(myData.operation, "Equipment_off");}
  myData.currentValue = 0.0;
  myData.currentLevel = EQUIPMENT_ON_LED;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
  // Serial.println("Sent with success");
  }

}

void TREADMILL_ON_listner(){
    // Set values to send
  strcpy(myData.messageName, "status");

  if (EQUIPMENT_ON_LED) {strcpy(myData.operation, "TraidmilOn");} else {strcpy(myData.operation, "TraidmilOff");}
  myData.currentValue = 0.0;
  myData.currentLevel = TREADMILL_ON_LED;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
  // Serial.println("Sent with success");
  }

}

void PUMP_ON_listner(){
    // Set values to send
  strcpy(myData.messageName, "status");

  if (EQUIPMENT_ON_LED) {strcpy(myData.operation, "Pump_on");} else {strcpy(myData.operation, "Pump_off");}
  myData.currentValue = 0.0;
  myData.currentLevel = TREADMILL_ON_LED;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
  // Serial.println("Sent with success");
  }
}

void LIFT_Llistner(){
  ///
  if (LIFT_DOWN_BUTTON) {
      // Set values to send
    strcpy(myData.messageName, "status");
    strcpy(myData.operation, "liftDown");
    myData.currentValue = 0.0;
    myData.currentLevel = LIFT_DOWN_BUTTON;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
    // Serial.println("Sent with success");
    }
  }
  /// 
   else if (LIFT_UP_BUTTON) {
      // Set values to send
    strcpy(myData.messageName, "status");
    strcpy(myData.operation, "liftUp");
    myData.currentValue = 0.0;
    myData.currentLevel = LIFT_UP_BUTTON;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
    // Serial.println("Sent with success");
    }
  }

}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
   memcpy(&myData, incomingData, sizeof(myData));
  // Check if myData.a is equal to "signal Testing"
  if (strcmp(myData.messageName, "signal") == 0) {} // Do nothing.
  if (strcmp(myData.messageName, "status") == 0){
    // Serial.println(myData.operation);
    if      (strcmp(myData.operation, "Equipment_on") == 0) {EQuipmentOn=myData.currentLevel;}
    else if (strcmp(myData.operation, "TraidmilOn") == 0) {RUNNING_TIMER=myData.currentLevel; SECONDS=myData.currentValue;}
    else if (strcmp(myData.operation, "TraidmilOn") == 0) {RUNNING_TIMER=myData.currentLevel; SECONDS=myData.currentValue;}
    else if (strcmp(myData.operation, "SpeedUP") == 0) {motor.motorGo(255); }
    else if (strcmp(myData.operation, "SpeedRev") == 0) {motor.motorRev(255);}
    else if (strcmp(myData.operation, "SpeedStop") == 0) { motor.motorBrake();}

    
  }   
}

/****************************************************************************************************
*****************************************************************************************************
*********************************** THREADS *********************************************************
******************************************************************************************************/
int __mapPotenziometerValue__(int input) {
  int inputMin = 0;     // Minimum input value
  int inputMax = 1570;  // Maximum input value
  int outputMin = 0;    // Minimum output value
  int outputMax = 100;  // Maximum output value

  // Perform the mapping calculation
  return ((input - inputMin) * (outputMax - outputMin)) / (inputMax - inputMin) + outputMin;
}
void GetAnalogValue(){
  while(true){
    SPEED = __mapPotenziometerValue__(analogRead(pin_potenziometer));
    delay(200);

  }
}



void GetLedAndButtonsStatus(){
  while(true){
  EQUIPMENT_ON_LED  = digitalRead(pin_Eq_On_Led_IN);
  TREADMILL_ON_LED  = digitalRead(pin_TreadMill_On_Led_IN);
  PUMP_ON_LED       = digitalRead(pin_Pump_On_Led_IN);

  Serial.print(" equ on:"); Serial.print(EQUIPMENT_ON_LED); 
  Serial.print("     tre on:"); Serial.print(TREADMILL_ON_LED); 
  Serial.print("     pu on:"); Serial.print(PUMP_ON_LED); 
  Serial.println();

  LIFT_UP_BUTTON    = digitalRead(pin_Lift_Treadmill_IN);
  LIFT_DOWN_BUTTON  = digitalRead(pin_Lower_Treadmill_IN);



  // EQUIPMENT_ON_listner();
  // TREADMILL_ON_listner();
  // PUMP_ON_listner();
  // LIFT_listner();
  delay(150);
  }
}

void SendConnectionSignal(){
  while(true){
  // Serial.println("Sending Signal");
    // Set values to send
    strcpy(myData.messageName, "signal");
    strcpy(myData.operation, "");
    myData.currentValue = SPEED;
    myData.currentLevel = true;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    delay(400);
  }
}

void StopperHandler(void* pvParameter) {
  while (true) {
    if (RUNNING_TIMER){ SECONDS+=1; delay(1000);}
    else {delay(100);} // Update every 100ms
  }
}



/****************************************************************************************************
*****************************************************************************************************
*********************************** SETUP ***********************************************************
******************************************************************************************************/
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


  ////////////////// Switches configurations ------------------------------------------------------------------
  /////// INPUTS ////////
  pinMode(pin_Eq_On_Led_IN ,        INPUT);
  pinMode(pin_TreadMill_On_Led_IN , INPUT);
  pinMode(pin_Pump_On_Led_IN ,      INPUT);

  pinMode(pin_Lift_Treadmill_IN ,   INPUT);
  pinMode(pin_Lower_Treadmill_IN ,  INPUT);

  /////// OUTPUTS ////////
  pinMode(pin_Eq_On_OUT ,           OUTPUT);
  pinMode(pin_Eq_Off_OUT ,          OUTPUT);

  pinMode(pin_TreadMill_On_OUT ,    OUTPUT);
  pinMode(pin_TreadMill_Off_OUT  ,  OUTPUT);
  
  pinMode(pin_Pump_On_OUT  ,        OUTPUT);
  pinMode(pin_Pump_Off_OUT  ,       OUTPUT);
  
  pinMode(pin_Lift_Treadmill_OUT  , OUTPUT);
  pinMode(pin_Lower_Treadmill_OUT , OUTPUT);

 
  //Potenziometer motor
  pinMode(pin_Motor_L, OUTPUT);
  pinMode(pin_Motor_R, OUTPUT);
 
  // AnalogRead Pot;
  pinMode(pin_potenziometer, INPUT);
  analogSetPinAttenuation(pin_potenziometer, ADC_6db);
  analogReadResolution(16);

  xTaskCreatePinnedToCore((TaskFunction_t)&GetAnalogValue, "GetAnalogValue", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore((TaskFunction_t)&GetLedAndButtonsStatus, "GetLedAndButtonsStatus", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore((TaskFunction_t)&SendConnectionSignal, "SendConnectionSignal", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore((TaskFunction_t)&StopperHandler, "StopperHandler", 4096, NULL, 1, NULL, 0);

}


void loop() {
  // digitalWrite( pin_Eq_On_OUT, HIGH  ); Serial.print("pin_Eq_On_OUT on: "); delay(2000); digitalWrite( pin_Eq_On_OUT, LOW  );   Serial.println("pin_Eq_On_OUT off: "); delay(8000);
  // digitalWrite( pin_Eq_Off_OUT, HIGH  ); Serial.print("pin_Eq_Off_OUT on"); delay(2000); digitalWrite( pin_Eq_Off_OUT, LOW  );  Serial. println(" pin_Eq_Off_OUT off");delay(8000);


  // digitalWrite( pin_TreadMill_On_OUT, HIGH  ); Serial.print("pin_TreadMill_On_OUT on: "); delay(2000); digitalWrite( pin_TreadMill_On_OUT, LOW  );   Serial.println(" pin_TreadMill_On_OUT off: "); delay(8000);
  // digitalWrite( pin_TreadMill_Off_OUT, HIGH  ); Serial.print("pin_TreadMill_Off_OUT on"); delay(2000); digitalWrite( pin_TreadMill_Off_OUT, LOW  );  Serial. println(" pin_TreadMill_Off_OUT off");delay(8000);

  // digitalWrite( pin_Pump_On_OUT, HIGH  ); Serial.print("pin_Eq_On_OUT on: "); delay(2000); digitalWrite( pin_Pump_On_OUT, LOW  );   Serial.println("pin_Pump_On_OUT off: "); delay(8000);
  // digitalWrite( pin_Pump_Off_OUT, HIGH  ); Serial.print("pin_Pump_Off_OUT on"); delay(2000); digitalWrite( pin_Pump_Off_OUT, LOW  );  Serial. println(" pin_Pump_Off_OUT off");delay(8000);


  //   digitalWrite( pin_Eq_On_OUT, HIGH  );  delay(2000); digitalWrite( pin_Eq_On_OUT, LOW  );   delay(8000);
  // digitalWrite( pin_Eq_Off_OUT, HIGH  ); delay(2000); digitalWrite( pin_Eq_Off_OUT, LOW  ); delay(8000);


  // digitalWrite( pin_TreadMill_On_OUT, HIGH  );  delay(2000); digitalWrite( pin_TreadMill_On_OUT, LOW  );   delay(8000);
  // digitalWrite( pin_TreadMill_Off_OUT, HIGH  ); delay(2000); digitalWrite( pin_TreadMill_Off_OUT, LOW  ); delay(8000);

  // digitalWrite( pin_Pump_On_OUT, HIGH  ); delay(2000); digitalWrite( pin_Pump_On_OUT, LOW  );    delay(8000);
  // digitalWrite( pin_Pump_Off_OUT, HIGH  );  delay(2000); digitalWrite( pin_Pump_Off_OUT, LOW  ); delay(8000);
  
}


void print_status(){

  // int x= digitalRead(pin_Eq_On_Led); Serial.print("equipmentOn value: "); Serial.print(x);
  // Serial.println("********************************************************************");
  // Serial.print("Equipment=>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"); Serial.print(EQUIPMENT_ON_LED);
  // int minutes = SECONDS / 60;
  // int seconds = SECONDS % 60;
  // Serial.print("  TIMER="); Serial.print(minutes); Serial.print(":");Serial.print(seconds); Serial.print("  RUNNING_TIMER="); Serial.print(RUNNING_TIMER); Serial.println();
  // Serial.println("********************************************************************");
  // delay(1000);
}