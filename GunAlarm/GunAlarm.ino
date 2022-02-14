

//Encoder Setup
#include <Encoder.h>
Encoder myEncoder(17,16);


// Realized that StateMachine might not be needed. Going to Try a switchcase statement instead


//TFT Setup
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
TFT_eSPI tft = TFT_eSPI();


// StateMachine Setup
#include <StateMachine.h>
StateMachine machine = StateMachine();
// Defining States
State* S1 = machine.addState(&state1);
State* S2 = machine.addState(&state2);


// Defining State Functions
void state1(){
  int myButtonState = digitalRead(33);
  tft.drawString("State 1:", 20, 20);
    while(myButtonState == 1){
      EncoderNewPosition = myEncoder.read();
      if(EncoderNewPosition != EncoderOldPosition){
        EncoderOldPosition = EncoderNewPosition;
        Serial.println(EncoderNewPosition);
      }
    }
  }
bool transitionS1S2(){return true;}
void state2(){
  tft.drawString("State 2:", 20, 20);
  

  /* WHile In state 2
   *  READ THE Encoder!!
   *  Take the global value that tracks the values of the different settings to be changed
   *  Increase or Decrease that setting
   *  Print the new results to the screen 
   *  Button press takes you back out
   */
  }
bool transitionS2S1(){return true;}


//Global Variables
const int button = 33;
int buttonState;
long EncoderOldPositon = -999;
long EncoderNewPosition;
long var1 = 0; // variable for setting to be changed - undefined ATM
long var2 = 0; // variable for setting to be changed - undefined ATM

//=================SETUP=================
void setup() {
  Serial.begin (115200);
  pinMode(button, INPUT_PULLUP); 


  // Defining Transitions
  S1->addTransition(&transitionS1S2, S2);
  S2->addTransition(&transitionS2S1, S1);


  // Initializing TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
}

void loop(){ 
  
  delay(500);
  buttonState = digitalRead(button);
  Serial.println(digitalRead(button));
  if(buttonState==0){
    machine.run();
  }
  

  
}
