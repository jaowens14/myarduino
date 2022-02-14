

#include <Arduino.h>
#define ButtonInput 33 // normal input button, mock sensor
#define RedLed 19 // normal output led

int state; // state for inside switch case

volatile int RedLedTimer = 0; // this get set inside the buttonAlarm "Machine"

hw_timer_t * timer = NULL; // pointer to variable type hw_timer_t https://diyprojects.io/esp32-timers-alarms-interrupts-arduino-code/

void IRAM_ATTR onTime() { // this the where the work of the timer is done
   if(RedLedTimer >>0){
      RedLedTimer--;         // reduces from 10 down to 0
   }
}

void setup() {
Serial.begin(115200);
pinMode(RedLed, OUTPUT);
pinMode(ButtonInput, INPUT_PULLUP);
 
  timer = timerBegin(0, 80, true);  // these arguments are timerBegin ( TimerIndex or TimerNumber, Prescalar, flag for counting up or down)
  timerAttachInterrupt(timer, &onTime, true); //
  timerAlarmWrite(timer, 1000000, true); // alarm goes every second
  timerAlarmEnable(timer);
}

void loop() {
  buttonAlarm();
  SmartLedMachine();
  }

void buttonAlarm(){
  switch(state){
    case 0: //Standby
      //Serial.println("STANDBY");
      if(digitalRead(ButtonInput) == 0){ //buttton has been pressed
        delay(50);
        //Serial.println("Button Pressed");
        state = 1; // update state to active
        RedLedTimer = 10; // set a 10 second timer
      }
    break;
    case 1: //Active
      //Serial.println("ACTIVE!");
      if(RedLedTimer <= 0){
        state = 0; // if timer is up send back to standby
      }
      //Serial.println(RedLedTimer);
    break;
  }
}


void SmartLedMachine(){
  if(RedLedTimer >> 0){
    // Led is on
    digitalWrite(RedLed, HIGH);
    //Serial.println("Led is on");
    Serial.println(RedLedTimer);
  }
  else{
    // Led is off
    digitalWrite(RedLed, LOW);
    //Serial.println("Led is off");
    Serial.println(RedLedTimer);
  }
}
