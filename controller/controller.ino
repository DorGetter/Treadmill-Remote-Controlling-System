/////////////////////////// OUTPUT FROM THE CONTROLLER ///////////////////
#define pin_Eq_On_OUT 12            // 1
#define pin_TreadMill_OUT 14     // 2 
#define pin_Pump_OUT 27          // 3


/////////////////////////// INPUT TO THE CONTROLLER ///////////////////
#define pin_Eq_On_Led 13            // 1 
#define pin_Eq_Off_Led 15           // 2

#define pin_TreadMill_On_Led 2      // 3
#define pin_TreadMill_Off_Led 4      // 4

#define pin_Pump_On_Led 16            // 5
#define pin_Pump_Off_Led 17         // 6

#define pin_Lift_Treadmill_Led 5   // 7
#define pin_Lower_Treadmill_Led 18   // 8

/////////////////////////// LEDS //////////////////////////////////////
#define pin_Eq_On_TEST_Led  25
#define pin_TreadMill_On_TEST_Led 33 
#define pin_Pump_On_TEST_Led 32



bool EQUPMENT_STATE = false;
bool TREADMILL_STATE = false;
bool PUMP_STATE = false;



void GetInputSignals(){
  while(true){
    bool EQUIPMENT_ON     = digitalRead(pin_Eq_On_Led);
    bool EQUIPMENT_OFF    = digitalRead(pin_Eq_Off_Led);
    bool TREADMILL_ON     = digitalRead(pin_TreadMill_On_Led);
    bool TREADMIL_OFF     = digitalRead(pin_TreadMill_Off_Led);
    bool PUNP_ON          = digitalRead(pin_Pump_On_Led);
    bool PUMP_OFF         = digitalRead(pin_Pump_Off_Led);
    bool TREADMILL_UP     = digitalRead(pin_Lift_Treadmill_Led);
    bool TREADMILL_LOWER  = digitalRead(pin_Lower_Treadmill_Led);


    Serial.print(" equ on:"); Serial.print(EQUIPMENT_ON); Serial.print(" equ off:"); Serial.print(EQUIPMENT_OFF);
    if (EQUIPMENT_ON & !EQUIPMENT_OFF)        { EQUPMENT_STATE = true;  }
    else if (!EQUIPMENT_ON & EQUIPMENT_OFF)   { EQUPMENT_STATE = false; }

    Serial.print("     tre on:"); Serial.print(TREADMILL_ON); Serial.print(" tre off:"); Serial.print(TREADMIL_OFF);
    if (TREADMILL_ON & !TREADMIL_OFF)        { TREADMILL_STATE = true;  }
    else if (!TREADMILL_ON & TREADMIL_OFF)   { TREADMILL_STATE = false; }


    Serial.print("     pu on:"); Serial.print(PUNP_ON); Serial.print(" pu off:"); Serial.print(PUMP_OFF);
    if (PUNP_ON & !PUMP_OFF)                { PUMP_STATE = true;  }
    else if (!PUNP_ON & PUMP_OFF)           { PUMP_STATE = false; }



    Serial.print("     UP on:"); Serial.print(TREADMILL_UP); Serial.print(" LOW off:"); Serial.print(TREADMILL_LOWER);

    Serial.println();
    delay(40);
  } 
}


void setup() {
  Serial.begin(115200);
/////////////////////////// OUTPUT FROM THE CONTROLLER ///////////////////
  pinMode(pin_Eq_On_OUT,              OUTPUT);
  pinMode(pin_TreadMill_OUT,          OUTPUT);
  pinMode(pin_Pump_OUT,               OUTPUT);

/////////////////////////// INPUT TO THE CONTROLLER ///////////////////
  pinMode(pin_Eq_On_Led,              PULLDOWN);
  pinMode(pin_Eq_Off_Led,             PULLDOWN);
  pinMode(pin_TreadMill_On_Led,       PULLDOWN);
  pinMode(pin_TreadMill_Off_Led,      PULLDOWN);
  pinMode(pin_Pump_On_Led,            PULLDOWN);
  pinMode(pin_Pump_Off_Led,           PULLDOWN);
  pinMode(pin_Lift_Treadmill_Led,     PULLDOWN);
  pinMode(pin_Lower_Treadmill_Led,    PULLDOWN);
/////////////////////////// LEDS //////////////////////////////////////

  pinMode(pin_Eq_On_TEST_Led,         OUTPUT);
  pinMode(pin_TreadMill_On_TEST_Led,  OUTPUT);
  pinMode(pin_Pump_On_TEST_Led,       OUTPUT);

  xTaskCreatePinnedToCore((TaskFunction_t)&GetInputSignals, "GetInputSignals", 1024, NULL, 1, NULL, 0);
}

void loop() {

  
  if (EQUPMENT_STATE)   {digitalWrite(pin_Eq_On_TEST_Led,HIGH);         digitalWrite(pin_Eq_On_OUT, HIGH); delay(100); digitalWrite(pin_Eq_On_OUT, LOW); }          else {digitalWrite(pin_Eq_On_TEST_Led,LOW);}
  if (TREADMILL_STATE)  {digitalWrite(pin_TreadMill_On_TEST_Led,HIGH);  digitalWrite(pin_TreadMill_OUT, HIGH); delay(100); digitalWrite(pin_TreadMill_OUT, LOW);}   else {digitalWrite(pin_TreadMill_On_TEST_Led,LOW);}
  if (PUMP_STATE)       {digitalWrite(pin_Pump_On_TEST_Led,HIGH);       digitalWrite(pin_Pump_OUT, HIGH); delay(100); digitalWrite(pin_Pump_OUT, LOW);}             else {digitalWrite(pin_Pump_On_TEST_Led,LOW);}


}
