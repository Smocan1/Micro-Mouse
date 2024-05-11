#include <Wire.h>
#include <VL6180X.h>
#include "Adafruit_VL53L0X.h"
 
#define GPIO1 A6
#define GPIO2 A7
#define GPIO3 A0
#define GPIO4 A1
#define GPIO5 A2
 
#define tofAddress1 20
#define tofAddress2 22
#define tofAddress3 24
#define tofAddress4 26
#define tofAddress5 28
 
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
VL6180X tof2;
VL6180X tof3;
Adafruit_VL53L0X tof4 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof5 = Adafruit_VL53L0X();
 
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
 
void setup()
{
  Serial.begin(115200);
  Wire.begin();
 
  pinMode(GPIO1, OUTPUT);
  pinMode(GPIO2, OUTPUT);
  pinMode(GPIO3, OUTPUT);
  pinMode(GPIO4, OUTPUT);
  pinMode(GPIO5, OUTPUT);
  delay(50);
 
  digitalWrite(GPIO1, LOW);
  digitalWrite(GPIO2, LOW);
  digitalWrite(GPIO3, LOW);
  digitalWrite(GPIO4, LOW);
  digitalWrite(GPIO5, LOW);
  delay(50);
 
  digitalWrite(GPIO1, HIGH);
  delay(50);
  tof1.begin(tofAddress1);
  tof1.startRangeContinuous();
  delay(100);
 
  digitalWrite(GPIO2, HIGH);
  delay(50);
  tof2.init();
  tof2.configureDefault();
  tof2.setAddress(tofAddress2);
  tof2.setTimeout(0);
  tof2.stopContinuous();
  delay(50);
  tof2.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 4);
  tof2.startRangeContinuous(100);
  delay(100);
 
  digitalWrite(GPIO3, HIGH);
  delay(50);
  tof3.init();
  tof3.configureDefault();
  tof3.setAddress(tofAddress3);
  tof3.setTimeout(0);
  tof3.stopContinuous();
  delay(50);
  tof3.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 2);
  tof3.startRangeContinuous(100);
  delay(100);
 
  digitalWrite(GPIO4, HIGH);
  delay(50);
  tof4.begin(tofAddress4);
  tof4.startRangeContinuous();
  delay(100);
 
  digitalWrite(GPIO5, HIGH);
  delay(50);
  tof5.begin(tofAddress5);
  tof5.startRangeContinuous();
  delay(100);
 
}
 
void loop()
{
  Serial.print("\tRange1: ");
  Serial.print(tof1.readRange());
  Serial.print("\tRange2: ");
  Serial.print(tof2.readRangeContinuousMillimeters());
  Serial.print("\tRange3: ");
  Serial.print(tof3.readRangeContinuousMillimeters());
  Serial.print("\tRange4: ");
  Serial.print(tof4.readRange());
  Serial.print("\tRange5: ");
  Serial.print(tof5.readRange());
  Serial.println();
}