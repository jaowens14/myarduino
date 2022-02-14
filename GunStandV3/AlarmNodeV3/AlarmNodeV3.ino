
#include <Arduino.h>

// Includes for MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

// Includes for Communication
#include <WiFi.h>
#include <esp_now.h>
#include "BluetoothSerial.h"

// Set up MPU Variables
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Bluetooth Setup
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial alarmBT; // Object for BT

// ESP Now Setup
esp_now_peer_info_t peerInfo;
uint8_t standNodeAddress[] = {0x98, 0xCD, 0xAC, 0x63, 0x8A, 0x20}; // This needs to match the MAC Address of the standNode
// Alarm Node: 98:CD:AC:62:FD:54
// Stand Node: 98:CD:AC:63:86:0C

// Variables for MPU Readings
float axSnap, aySnap, azSnap = 0.0;
float gxSnap, gySnap, gzSnap = 0.0;
float axFloor, ayFloor, azFloor, axCeiling, ayCeiling, azCeiling = 0.0;

bool isTriggered = false; // Bool to keep if the alarm has been triggered

// Functions for the MPU
void fetchRawMpuData() { mpu.getEvent(&a, &g, &temp); }

void snapShotMpuData()
{
    int count = 0;
    unsigned long currMillis = millis();
    const long interval = 5000;
    while (millis() < currMillis + interval) {
        fetchRawMpuData();
        axSnap = axSnap + a.acceleration.x;
        aySnap = aySnap + a.acceleration.y;
        azSnap = azSnap + a.acceleration.z;
        gxSnap = gxSnap + g.gyro.x;
        gySnap = gySnap + g.gyro.y;
        gzSnap = gzSnap + g.gyro.z;
        count++;
    }
    axSnap /= count;
    aySnap /= count;
    azSnap /= count;
    gxSnap /= count;
    gySnap /= count;
    gzSnap /= count;
}

void printSnap()
{
    Serial.print("Snap -> X: "); Serial.print(axSnap); 
    Serial.print(", Y: "); Serial.print(aySnap);
    Serial.print(", Z: "); Serial.println(azSnap);
}

void printReadings()
{
    Serial.print("Readings -> X: "); Serial.print(a.acceleration.x); 
    Serial.print(", Y: "); Serial.print(a.acceleration.y);
    Serial.print(", Z: "); Serial.println(a.acceleration.z);
}

void calcFloorCeilings(float sens)
{
    axFloor = axSnap - sens; axCeiling = axSnap + sens;
    ayFloor = aySnap - sens; ayCeiling = aySnap + sens;
    azFloor = azSnap - sens; azCeiling = azSnap + sens;
}

void initMpu()
{
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  snapShotMpuData();
}

bool fetchSensitiveMpu() 
{
    float sens = 2.0;
    //TODO: Add Gyro Triggers
    fetchRawMpuData();
    calcFloorCeilings(sens);
    if ((a.acceleration.x < axFloor) || (a.acceleration.x > axCeiling)) { return true; }
    if ((a.acceleration.y < ayFloor) || (a.acceleration.y > ayCeiling)) { return true; }
    if ((a.acceleration.z < azFloor) || (a.acceleration.z > azCeiling)) { return true; }
    return false;
}

void printMpuData()
{
    fetchRawMpuData();
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");
}

// Communication Functions
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println("Alarm Triggered");
}

void initPeer()
{
    // Register peer
    memcpy(peerInfo.peer_addr, standNodeAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
  
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
}

void sendTrigger(int trigger)
{
    esp_err_t result = esp_now_send(standNodeAddress, (uint8_t *) &trigger, sizeof(trigger));
}

void beginNow()
{
    WiFi.mode(WIFI_STA);
    //Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(OnDataSent);
    initPeer();
}

//void btBegin()
//{
//    alarmBT.begin("ESP32 Alarm Node");
//}

//bool deactivateListen()
//{
//    if (alarmBT.available()) {
//        int deact = (int)alarmBT.read();
//        if (deact == 49) { return true; }
//        else { return false; }
//    }
//    return false;
//}

// Main Program
void setup() {
  Serial.begin(115200);
  initMpu();
  printSnap();
  beginNow();
 // btBegin();
  Serial.println("Alarm Set...");
}

void loop() {
  if (fetchSensitiveMpu()) {
    Serial.println("Triggered");
    isTriggered = true;
    sendTrigger(1);
  }
  if (isTriggered) {
    sendTrigger(0);
    isTriggered = false;
    Serial.println("Alarm Deactivated.");
  }
  delay(20);
}
