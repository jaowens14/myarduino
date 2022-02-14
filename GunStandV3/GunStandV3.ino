/* How things work

   There is an IR Transmiter/Receiver Pair.
   When the Button is pressed on the trinket, it triggers the white LED.

   There are 3 Alarm Units.
   When they are moved they use ESPnow to communicate and trigger the Red LED.

   There is a menu that is navigated by a rotary encoder
   There is a LCD screen to display that menu

*/

/**
 * Changes made by Jj on 12/22/2021
 *  - Fixed Bug with the alarm triggering in a loop (Forgot to reset a variable to 0)
 *  - Fixed issue with the LCD Screen (connection issue)
 *  - Added a timer check to the remote control check so then it doesn't take 3 reads, only one
 *  - Made sure up to 3 alarms work at the same time
 **/


//=================INCLUDES=================

#include <IRremote.h>
#include <Encoder.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>

//=================PINS=================

#define RedLed 32
#define WhiteLed 33
#define MotionSensor 35 // p
#define IrReceive 15
#define EncoderPin2 16
#define EncoderPin1 17
#define EncoderButton 25

#define DECODE_NEC

//=================ESP_NOW_COMMUNICATION=================
esp_now_peer_info_t peerInfo1;
uint8_t alarmNodeAddress1[] = {0x98, 0xCD, 0xAC, 0x62, 0xFD, 0x54}; // This needs to match the MAC Address of the alarmNode


//=================OBJECTS=================

TFT_eSPI tft = TFT_eSPI();
Encoder myEncoder(EncoderPin1, EncoderPin2);

//=================GLOBAL=================

volatile bool LedColor = 0; //0 = red, false = white ******
volatile bool NearLedColor = 0; // 0 = red, 1 = white
volatile bool LedMode = 1; //true = dimmer, false = strobe ****
volatile bool NearLedMode = 1; // This will always be dimmer. A near trigger and strobe would be a bad idea
volatile bool LedOutput = 0; //true = light on, false light off
volatile bool MotionSensorIn = 0; //input from the Near Motion sensor
volatile bool MotionSensorInTemp = 0;
volatile bool RemoteSensorIn = 0; //input from the IR Remote
bool DoorSensorIn; //input from esp_now protocol
volatile int LedTimer = 0; // variable containing duration that counts down
volatile bool NearTrigger = false; // This will let the system decide if it is a near trigger or door trigger
volatile int StrobeState;
volatile int Duration = 10; // amount of time in seconds to display led, a setting **
volatile int Brightness = 5; // amount of light given out by leds ****
volatile int count = 0;
volatile int RemoteControlDebounce = 0; // Time to wait to read the next value from the Remote Control
volatile int RemoteControlDebounceDuration = 2;

int NearMotionState = 0; // initialized at 0 as the default state
int DoorAlarmState = 0;
int RemoteControlState = 0;
int SmartMenuState = 0;

long encoderOldPosition = 0;
long encoderNewPosition = 0;
// ================Channels for Led Modes =================
const int RedCh = 0;
const int WhiteCh = 1;
const int Resolution = 8;
const int Freq = 5000;

const long StrobeFreq = 10; // in 1/10ths of a Hertz. So 10 is 1.0 Hertz, 11 is 1.1 Hertz


//=================IRAM TIMER FUNCTIONS=================

hw_timer_t * MainTimer = NULL; // pointer to variable type hw_timer_t https://diyprojects.io/esp32-timers-alarms-interrupts-arduino-code/


void IRAM_ATTR onTime() { // this the where the work of the timer is done
  if (LedTimer > 0) { // happens every second
    LedTimer--;         // reduces from duration down to 0
    StrobeState = 1;
  }
  if (RemoteControlDebounce > 0) {
    RemoteControlDebounce--;
  }
}



//=================SETUP=================
void setup() {
  Serial.begin(115200);
  //pinMode(RedLed, OUTPUT);
  pinMode(MotionSensor, INPUT);
  pinMode(EncoderButton, INPUT_PULLUP);

  // configuration of Led Channels
  ledcSetup(RedCh, Freq, Resolution);
  ledcSetup(WhiteCh, Freq, Resolution);
 
  ledcAttachPin(RedLed, RedCh); // 0 attaching leds to pwm channels,
  ledcAttachPin(WhiteLed, WhiteCh); // 1
  // 0 = strobe, 1 = dimmer



  //=================TIMER=================
  MainTimer = timerBegin(0, 80, true);  // these arguments are timerBegin ( TimerIndex or TimerNumber, Prescalar, flag for counting up or dow  timerAttachInterrupt(timer, &onTime, true); //
  timerAttachInterrupt(MainTimer, &onTime, true); //
  timerAlarmWrite(MainTimer, 1000000, true); // ticker ticks every 1/10th second
  timerAlarmEnable(MainTimer);

  //=================ESP Now Setup =============
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_peer_info_t peerInfo1;
  memcpy(peerInfo1.peer_addr, alarmNodeAddress1, 6);
  peerInfo1.channel = 0;
  peerInfo1.encrypt = false;
  // Add peer
  if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
    // Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  //==========================================

  //===============SCREEN SETUP===========================
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  delay(1000);
  tft.drawString("Hello World", 20, 20);
  delay(3000);
  tft.fillScreen(TFT_BLACK);
 

  //===============IR RECEIVER===========================
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
}

//=================MAIN LOOP=================
void loop() {
  NearMotion();
  DoorAlarm();
  SmartLedMachine();
  RemoteControl();
  SmartMenu();
}

//=================MACHINES=================

// ======== Function that handles Near Motion Input ===============
void NearMotion() {
  switch (NearMotionState) {
    case 0:
      tft.drawString("                            ", 40, 40);
      // Standby, check the status of Near Motion IR
      //MotionSensorInTemp = digitalRead(MotionSensor);
      //delay(100);
      MotionSensorIn = digitalRead(MotionSensor);
      // if low, debounce, then change states
     
      if (MotionSensorIn == LOW) { // The Sensor is Normally HIGH
        //delay(50); // debounce delay
        // Make the LEDs white and dimmer
        NearTrigger = true;
        RemoteControlState = 0;
        NearMotionState = 1; // update the state to active
        LedTimer = Duration; // set timer to duration
      }
      // Serial.println("nearMotion 0");
      break;
    case 1:
      // Active, check the timer to see if we need to switch back to stand by
      if (LedTimer <= 0) {
        NearMotionState = 0; // timer is up! back to standby
        NearTrigger = false;
        // Move back the LED settings
      }
      tft.drawString("Motion Sensor", 40, 40);
      // Serial.println("nearMotion 1");
      break;
  }
}

// ======== Function that handles Door Alarm Input ===============
void DoorAlarm() {
  switch (DoorAlarmState) {
    case 0:
      // Standby, check the status of Door Alarm
      tft.drawString("                          ", 40, 40); // clear Door Alarm Message
      if (DoorSensorIn == 1) {
        // The Door Sensor has been triggered
        RemoteControlState = 0;
        DoorAlarmState = 1; // update the state to active
        LedTimer = Duration; // set timer to duration
      }
      // Serial.println("Door Alarm 0");
      break;
    case 1:
      // Active, check if the timer is up
      if (LedTimer <= 0) {
        DoorAlarmState = 0; // timer is up! back to standby
        DoorSensorIn = 0;
      }
      // While active write to display, Door Alarm
      tft.drawString("Door Alarm", 40, 40);
      //tft.drawString(String(LedTimer), 60, 60);
      //tft.drawString("  ", 60, 60);
      // Serial.println("Door Alarm 1");
      break;
  }
}

// ======== Function that handles IR Receiver INPUT ===============
// ======== Used as a kill switch for all processes ===============
void RemoteControl() {
  switch (RemoteControlState) {
    case 0:
      // Standby, check if the remote has been triggered
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        if (IrReceiver.decodedIRData.command == 0x34 && RemoteControlDebounce <= 0){
          // command received, do stuff
          // set state to off
          RemoteControlState = 1;
          RemoteControlDebounce = RemoteControlDebounceDuration;
          LedTimer = 0;
          Duration = 10;
          delay(100);
        }
      }
      // Serial.println("remote Control 0");
      break;
    case 1:
      // active, check if the timer is up
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        if (IrReceiver.decodedIRData.command == 0x34 && RemoteControlDebounce <= 0){
          // command received, do stuff
          // set state to on
          RemoteControlState = 0;
          RemoteControlDebounce = RemoteControlDebounceDuration;
          LedTimer = 0;
          Duration = 61;
          delay(100);
        }
      }
      // Serial.println("Remote Control 1");
      break;
  }
}


// ======== Function that handles MENU LCD OUTPUT ==========
void SmartMenu() {
  switch (SmartMenuState) {
    case 0: // Standby, do nothing, wait for button press
      if (digitalRead(EncoderButton) == 0) { // normally 1  
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Entering Menu...", 20, 20);
        delay(500);
        SmartMenuState = 1;
      }
      break;
    // ==============================================
    case 1: // Select Mode
      tft.drawString("Select Mode: ...", 20, 20);
      if (digitalRead(EncoderButton) == 0) { // normally 1
        //delay(200);
        tft.fillScreen(TFT_BLACK);
        SmartMenuState = 2;
      }
      if (LedMode == 0) { // Strobe
        tft.drawString("Strobe", 40, 40);
        delay(50);
      }
      if (LedMode == 1) { // Dimmer
        tft.drawString("Dimmer", 40, 40);
        delay(50);
      }
      LedMode = EncoderIncrement(LedMode);
      tft.drawString("        ", 40, 40);
      break;
    // ==============================================
    case 2: // Select Led Color
      tft.drawString("Select Color: ...", 20, 20);
      if (digitalRead(EncoderButton) == 0) { // normally 1
        //delay(200);
        tft.fillScreen(TFT_BLACK);
        SmartMenuState = 3;
      }

      if (LedColor == 0) { // red
        tft.drawString(" Red ", 40, 40);
        delay(50);
      }
      if (LedColor == 1) { // white
        tft.drawString("White", 40, 40);
        delay(50);
      }
      LedColor = EncoderIncrement(LedColor);
      tft.drawString("        ", 40, 40);
      break;
    // ==============================================
    case 3: // Select Duration
      tft.drawString("Select Duration: ...", 20, 20);
      if (digitalRead(EncoderButton) == 0) { // normally 1
        //delay(200);
        tft.fillScreen(TFT_BLACK);
        SmartMenuState = 4;
      }
      Duration = constrain(Duration, 0, 61); // constrain the value in seconds, 61 is always on
      if(Duration > 60){tft.drawString("Always On", 40, 40);}
      else{tft.drawString(String(Duration), 40, 40);}
      Duration = EncoderIncrement(Duration);
      tft.drawString("              ", 40, 40);
      break;
    // ==============================================
    case 4: // Select Brightness
      tft.drawString("Select Brightness: ...", 20, 20);
      if (digitalRead(EncoderButton) == 0) { // normally 1
        //delay(200);
        tft.fillScreen(TFT_BLACK);
        SmartMenuState = 5;
      }
      Brightness = constrain(Brightness, 0, 10);
      tft.drawString(String(Brightness), 40, 40);
      Brightness = EncoderIncrement(Brightness);
      tft.drawString("    ", 40, 40);
      break;
    // ==============================================
    case 5: // Select Color of near trigger
      tft.drawString("Select Color for Near: ...", 20, 20);
      if (digitalRead(EncoderButton) == 0) { // normally 1
        //delay(200);
        tft.fillScreen(TFT_BLACK);
        SmartMenuState = 6;
      }

      if (NearLedColor == 0) { // red
        tft.drawString(" Red ", 40, 40);
        delay(50);
      }
      if (NearLedColor == 1) { // white
        tft.drawString("White", 40, 40);
        delay(50);
      }
      NearLedColor = EncoderIncrement(NearLedColor);
      tft.drawString("        ", 40, 40);
      break;
    case 6:
      tft.drawString("Exiting Menu...", 20, 20);
      delay(500);
      tft.fillScreen(TFT_BLACK);
      SmartMenuState = 0;
      break;
  }
}


// ======== Function that handles Led Output ===============
void SmartLedMachine() {
  // if there is timer on the timer check the modes and colors and display on the correct channel
  bool choosenColor = LedColor;
  bool choosenMode = LedMode;
  if (NearTrigger) {
    choosenColor = NearLedColor;
    choosenMode = NearLedMode;
  }
  if (LedTimer > 0) {
    tft.fillScreen(TFT_BLACK);
    SmartMenuState = 0; // Clear the menu
    if (choosenMode == 0) { // 0 = strobe, 1 = dimmer
      if (choosenColor == 0) { // 0 = red, 1 = white
        // Serial.println("Strobe State:");
        // Serial.println(StrobeState);
        if (StrobeState == 1){ 
          ledcWrite(RedCh, map(Brightness, 0, 10, 0, 255)); 
          StrobeState = 0;
          }
        else { ledcWrite(RedCh, 0); }
        // Serial.println(LedTimer);
      }
      else { // its Strobe White Case
        if (StrobeState == 1){ 
          ledcWrite(WhiteCh, map(Brightness, 0, 10, 0, 255));
          StrobeState = 0;
          }
        else { ledcWrite(WhiteCh, 0); }
        // Serial.println(LedTimer);
      }
    }

    if (choosenMode == 1) { // 0 = strobe, 1 = dimmer
      if (choosenColor == 1) { // 0 = red, 1 = white
        // Dimmer, White case
        ledcWrite(WhiteCh, map(Brightness, 0, 10, 0, 255));
        // Serial.println("Led is Dimmer, White");
        // Serial.println(LedTimer);
      }
      else { // its Dimmer, red case
        ledcWrite(RedCh, map(Brightness, 0, 10, 0, 255));
        // Serial.println("Led is Dimmer, Red");
        // Serial.println(LedTimer);
      }
    }
  }
  // else there is no time left on timer, turn the leds off
  else {
  // Led is off
  ledcWrite(RedCh, 0);
  ledcWrite(WhiteCh, 0);
  // Serial.println("Led is off");
  // Serial.println(LedTimer);
  }

  // ====== Always on Mode
  if(Duration > 60){
    if (choosenColor == 1) { // 0 = red, 1 = white
        // Dimmer, White case
        ledcWrite(WhiteCh, map(Brightness, 0, 10, 0, 255));
        // Serial.println("Led is Always on, White");
        // Serial.println(LedTimer);
      }
      else { // its Dimmer, red case
        ledcWrite(RedCh, map(Brightness, 0, 10, 0, 255));
        // Serial.println("Led is Always on, Red");
        // Serial.println(LedTimer);
      }
  }
 
} // end of function

// ======== Function that handles ENCODER ==========
int EncoderIncrement(int initialValue) {
  int finalValue;
  encoderNewPosition = myEncoder.read();
  delay(100);
  if (encoderNewPosition != encoderOldPosition) {
    delay(100);
    if (encoderNewPosition > encoderOldPosition) {
      initialValue++;
      //// Serial.println(initialValue);
      encoderOldPosition = encoderNewPosition;
    }
    if (encoderNewPosition < encoderOldPosition) {
      initialValue--;
      //// Serial.println(initialValue);
      encoderOldPosition = encoderNewPosition;
    }
  }
  return finalValue = initialValue;
}

int ReadEncoderButton(){
  int tempVal = digitalRead(EncoderButton);
  delay(50);
  int tempVal2 = digitalRead(EncoderButton);
  if (tempVal == tempVal2){
    return tempVal;
  }
}


// ======== Function active on Data Receive from Alarm Nodes ===============
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&DoorSensorIn, incomingData, sizeof(DoorSensorIn));
  delay(150);
}
