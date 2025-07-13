#include <Arduino.h>
#include <Wire.h>


float accX_offset, accY_offset, accZ_offset, gyroX_offset, gyroY_offset, gyroZ_offset;
const int MPU_ADDR = 0x68;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float alpha = 0.94; // ratio of gyro to acc usage, (relative to gyro)

float accPitch, accRoll;
float pitch, roll;

float freedom = 25;

//const float noiseThreshold = 0.0;

long last, now;

void readData(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_ADDR, 14, true);

  // 1g = 2^14 = 16394 steps
  // each acc and gyro value are 2 bytes

  accX = ((Wire.read()<<8 | Wire.read()) - accX_offset) / 16384.0;
  accY = ((Wire.read()<<8 | Wire.read()) - accY_offset) / 16384.0;
  accZ = ((Wire.read()<<8 | Wire.read()) - accZ_offset) / 16384.0;

  Wire.read();Wire.read();

  gyroX = ((Wire.read()<<8 | Wire.read()) - gyroX_offset) / 131.0;
  gyroY = ((Wire.read()<<8 | Wire.read()) - gyroY_offset) / 131.0;
  gyroZ = ((Wire.read()<<8 | Wire.read()) - gyroZ_offset) / 131.0;
}

void calibration(){
  int32_t ax = 0, ay = 0, az = 0;
  int32_t gx = 0, gy = 0, gz = 0;

  const int samples = 500;
  for (int i = 0; i<samples; i++){
    Wire.beginTransmission(MPU_ADDR);
    // begin reading at 0x3B
    // mpu6050 registries
    // https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
    Wire.write(0x3B);
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax += (Wire.read()<<8 | Wire.read());
    ay += (Wire.read()<<8 | Wire.read());
    az += (Wire.read()<<8 | Wire.read());

    Wire.read();Wire.read();

    gx += (Wire.read()<<8 | Wire.read());
    gy += (Wire.read()<<8 | Wire.read());
    gz += (Wire.read()<<8 | Wire.read());

    delay(5);
  }


  accX_offset = ax / samples - 16384.0;
  accY_offset = ay / samples;
  accZ_offset = (az / samples);

  gyroX_offset = gx / samples;
  gyroY_offset = gy / samples;
  gyroZ_offset = gz / samples;
}

void setup(){
  Serial.begin(9600);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  delay(100);

  Serial.println("begin calibration");
  calibration();
  Serial.println("end calibration");

  last = millis();

}

void loop(){
  
  readData();

  now = millis();
  float dt = (now-last)/1000.0;
  last = now;

  accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
  accRoll  = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  pitch += gyroY*dt;
  roll += gyroX*dt;

  // Serial.print("accX: ");
  // Serial.print(accX, 2);
  // Serial.print(" | accY: ");
  // Serial.print(accY, 2);
  // Serial.print(" | accZ: ");
  // Serial.println(accZ, 2);

  // if (abs(pitch) < noiseThreshold) pitch = 0.0;
  // if (abs(roll)  < noiseThreshold) roll  = 0.0;
  // if (abs(accPitch) < noiseThreshold) accPitch = 0.0;
  // if (abs(accRoll)  < noiseThreshold) accRoll  = 0.0;

  pitch = alpha * pitch + (1-alpha) * accPitch;
  roll = alpha * roll + (1-alpha) * accRoll;


  // checking posture

  if (freedom<abs(pitch) || (90-freedom)>abs(roll)) {
    Serial.print("fix that shi");
    Serial.println(millis());
  }
  

  // Serial.print("Pitch: ");
  // Serial.print(pitch, 0);
  // Serial.print("°, Roll: ");
  // Serial.print(roll, 0);
  // Serial.println("°");

}