

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
*********************************** STATUS VARIALBLES ***********************************************
******************************************************************************************************/
// int EQuipmentOn = false;
int SPEED = 0;
int SECONDS = 0;
int RUNNING_TIMER = false;
float POT_STATUS = 0;
bool DONE = true;

bool EQUIPMENT_ON_LED = false;
bool TREADMILL_ON_LED = false;
bool PUMP_ON_LED = false;
bool LIFT_UP_BUTTON = false;
bool LIFT_DOWN_BUTTON = false;






/****************************************************************************************************
*****************************************************************************************************
*********************************** TRIGGERS ********************************************************
******************************************************************************************************/

void equipmentOnTrigger(bool val){
  Serial.print("equipment trigger: "); Serial.println(val);
    if (val == EQUIPMENT_ON_LED){return;} 
    else if (val) {digitalWrite( pin_Eq_On_OUT,         HIGH);  delay(100); digitalWrite( pin_Eq_On_OUT,          LOW);delay(100);}
    else          {digitalWrite( pin_Eq_Off_OUT,        HIGH);  delay(100); digitalWrite( pin_Eq_Off_OUT,         LOW);delay(100);
                   digitalWrite( pin_TreadMill_Off_OUT, HIGH);  delay(100); digitalWrite( pin_TreadMill_Off_OUT,  LOW);delay(100);
                   digitalWrite( pin_Pump_Off_OUT,      HIGH);  delay(100); digitalWrite( pin_Pump_Off_OUT,       LOW);delay(100);
                   RUNNING_TIMER = false;  SECONDS = 0;
                   }
}

void treadmillOnTrigger(bool val){
  Serial.print("treadmill trigger: "); Serial.println(val);
  if (val == TREADMILL_ON_LED){return;} 
  else if (EQUIPMENT_ON_LED){
    
    if (val)        {digitalWrite( pin_TreadMill_On_OUT, HIGH   );  delay(200); digitalWrite( pin_TreadMill_On_OUT,  LOW); RUNNING_TIMER = true;}

    else            {digitalWrite( pin_TreadMill_Off_OUT, HIGH  );  delay(200); digitalWrite( pin_TreadMill_Off_OUT, LOW); RUNNING_TIMER = false;}
  }
}

void treadmillStopTrigger(){
  
  Serial.print("treadmillStop trigger: ");
  digitalWrite( pin_TreadMill_Off_OUT, HIGH  );  delay(200); digitalWrite( pin_TreadMill_Off_OUT, LOW); RUNNING_TIMER = false;
  RUNNING_TIMER = false;
}

void treadmillResetTrigger(){
    Serial.print("treadmillReset trigger: "); 
    SECONDS = 0; 
}

void pumpOnTrigger(bool val){
  if (val)    {digitalWrite( pin_Pump_On_OUT, HIGH   );   delay(200); digitalWrite( pin_Pump_On_OUT,  LOW);}
  else        {digitalWrite( pin_Pump_Off_OUT, HIGH  );   delay(200); digitalWrite( pin_Pump_Off_OUT, LOW);}
  
}



/****************************************************************************************************
*****************************************************************************************************
*********************************** SEND & RECIEAVE *************************************************
******************************************************************************************************/

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
   memcpy(&myData, incomingData, sizeof(myData));
  // Check if myData.a is equal to "signal Testing"
  if (strcmp(myData.messageName, "signal") == 0) {} // Do nothing.
  if (strcmp(myData.messageName, "status") == 0){
    // Serial.println(myData.operation);
    if      (strcmp(myData.operation, "EquipmentOn") == 0)    {equipmentOnTrigger(myData.currentLevel);}
    else if (strcmp(myData.operation, "TreadmillOn") == 0)   {treadmillOnTrigger(myData.currentLevel);}
    else if (strcmp(myData.operation, "TreadmillStop") == 0) {treadmillStopTrigger();  motor.motorBrake();}
    else if (strcmp(myData.operation, "TreadmillReset") == 0) {treadmillResetTrigger(); motor.motorBrake();}
    else if (strcmp(myData.operation, "PumpOn") == 0)         {pumpOnTrigger(myData.currentLevel);}

    // else if (strcmp(myData.operation, "LiftUP") == 0)    {motor.motorGo(255);  }
    // else if (strcmp(myData.operation, "LiftRev") == 0)   {motor.motorRev(255); }
    // else if (strcmp(myData.operation, "LiftStop") == 0)  {motor.motorBrake();  }

    else if (strcmp(myData.operation, "SpeedUP") == 0)    {motor.motorGo(255);  }
    else if (strcmp(myData.operation, "SpeedRev") == 0)   {motor.motorRev(255); }
    else if (strcmp(myData.operation, "SpeedStop") == 0)  {motor.motorBrake();  }

    
  }   
}

/****************************************************************************************************
*****************************************************************************************************
*********************************** THREADS *********************************************************
******************************************************************************************************/

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


void GetLedAndButtonsStatus(){
  while(true){
    EQUIPMENT_ON_LED  = digitalRead(pin_Eq_On_Led_IN);          delay(15);
    TREADMILL_ON_LED  = digitalRead(pin_TreadMill_On_Led_IN);   delay(15);
    PUMP_ON_LED       = digitalRead(pin_Pump_On_Led_IN);        delay(15);

    // LIFT_UP_BUTTON    = digitalRead(pin_Lift_Treadmill_IN);     delay(20);
    // LIFT_DOWN_BUTTON  = digitalRead(pin_Lower_Treadmill_IN);    delay(20);
  }
}

void SendConnectionSignalAndStatus(){
  while(true){
    sender_helper("timer",  "",             SECONDS,          true);              delay(10);

    sender_helper("status", "Equipment_on",   0,              EQUIPMENT_ON_LED);  delay(10);
    sender_helper("status", "Treadmill_On",   0,              TREADMILL_ON_LED);  delay(10);
    sender_helper("status", "Pump_On",        0,              PUMP_ON_LED);       delay(10);
  }
}

void SendSignalAndSpeed(){
  while(true){
    sender_helper("signal", "",             SPEED,                true);             
    delay(20);
  }
}


void StopperHandler(void* pvParameter) {
  while (true) {
    if (RUNNING_TIMER){ SECONDS+=1; delay(1000);}
    else {delay(100);} // Update every 100ms
  }
}


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

  xTaskCreatePinnedToCore((TaskFunction_t)&GetAnalogValue,                "GetAnalogValue",                 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore((TaskFunction_t)&GetLedAndButtonsStatus,        "GetLedAndButtonsStatus",         4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore((TaskFunction_t)&SendConnectionSignalAndStatus, "SendConnectionSignalAndStatus",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore((TaskFunction_t)&SendSignalAndSpeed,            "SendSignalAndSpeed",             4096, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore((TaskFunction_t)&StopperHandler,                "StopperHandler",                 4096, NULL, 1, NULL, 0);
}



void loop() {

  if (SECONDS >= 1800)          {DONE = true;   sender_helper("status", "Time_Out",0,DONE); equipmentOnTrigger(false);}
  if (EQUIPMENT_ON_LED && DONE) {DONE = false;  sender_helper("status", "Time_Out",0,DONE);}

  Serial.print("eq: ");Serial.print(EQUIPMENT_ON_LED);Serial.print("   ter: ");Serial.print(TREADMILL_ON_LED);
  Serial.print("   pu: ");Serial.print(PUMP_ON_LED); ;Serial.print("   seconds: ");Serial.print(SECONDS);   
  Serial.print("  DONE  "); Serial.println(DONE);

  delay(500);
  
}