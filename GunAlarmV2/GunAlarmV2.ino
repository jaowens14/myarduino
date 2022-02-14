

//Encoder Setup
#include <Encoder.h>
Encoder myEncoder(17,16);

//TFT Setup
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
TFT_eSPI tft = TFT_eSPI();


//Global Variables
const int button = 33;
int buttonState;
int buttonCounter;
int stateIndex = 0;
const int stateCount = 4;
long encoderOldPosition=0;
long encoderNewPosition=0;
long var1 = 5; // variable for setting to be changed - undefined ATM
long var2 = 5; // variable for setting to be changed - undefined ATM
bool flag1, flag2;


//=================SETUP=================
void setup() {
  Serial.begin (115200);
  pinMode(button, INPUT_PULLUP); 

  // Initializing TFT
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
}

void loop(){ 
  
  buttonState = digitalRead(button); // normally 1, 0 when pressed

  if(buttonState == 0){
    delay(500);
    if(buttonCounter < stateCount){
      buttonCounter++;
      tft.fillScreen(TFT_BLACK);
    }
    else{
      buttonCounter = 0;
      tft.fillScreen(TFT_BLACK);
    }
    Serial.println(buttonCounter);
  }

  switch(buttonCounter){
    case 0:
    flag1 = false;
    tft.drawString("case 0", 20, 20);
    tft.drawString("Var 1 : ", 20, 40);
    tft.drawString(String(var1), 120, 40);
    tft.drawString("Var 2 : ", 20, 60);
    tft.drawString(String(var2), 120, 60);
    Serial.println("case 0");
    break;
    case 1:
    if(flag1 == false){
      encoderOldPosition=0;
      encoderNewPosition=0;
      flag1 = true;
    }
    tft.drawString("case 1", 20, 20);
    tft.drawString("Rotate to change var1: ", 20, 40);
    if(buttonCounter==1){Serial.println("button counter case");var1 = encoderIncrement(var1);}
    tft.drawString(String(var1), 20, 60);
    Serial.println(buttonCounter);
    break;
    case 2:
    tft.drawString("case 2", 20, 20);
    tft.drawString("Rotate to change var2: ", 20, 40);
    if(buttonCounter==2){var2 = encoderIncrement(var2);}
    tft.drawString(String(var2), 20, 60);
    break;
    case 3:
    tft.drawString("case 3", 20, 20);
    break;
    case 4:
    tft.drawString("case 4", 20, 20);
    break;
  }
}



long encoderIncrement(long initialValue){
  long finalValue;
  encoderNewPosition = myEncoder.read();
  if(encoderNewPosition != encoderOldPosition){
      Serial.println("encoder new position");
      Serial.println(encoderNewPosition);
      if(encoderNewPosition > encoderOldPosition){
        initialValue = map(encoderNewPosition, -200, 200, 0, 10);
      }
      if(encoderOldPosition > encoderNewPosition){
        initialValue = map(encoderNewPosition, -200, 200, 0, 10);
      }
    }
    encoderOldPosition = encoderNewPosition;
    return finalValue = initialValue;
  }




/*
 * 
 * button is sending 1 constantly
 * button press 0
 * if the button press is zero
 */
