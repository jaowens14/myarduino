#include <IRremote.h> // >v3.0.0
                                            
#define PIN_SEND 16

void setup()  
{  
  IrSender.begin(true, PIN_SEND); // Initializes IR sender
  Serial.begin(9600);
}  
                               
void loop()  
{  
  IrSender.sendNEC(0x0102, 0x34, true, 0); // the address 0x0102 with the command 0x34 is sent 
  delay(1000); // wait for one second
}  
