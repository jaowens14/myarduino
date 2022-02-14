  #define MotionSensor 35 // p




void setup() {
  // put your setup code here, to run once:
  pinMode(MotionSensor, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(digitalRead(MotionSensor));
  delay(100);

}
