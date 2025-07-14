#include <Arduino.h>
#include <Wire.h>



float accX_offset, accY_offset, accZ_offset;
const int MPU_ADDR = 0x68;
float pitch, roll;
float accX, accY, accZ;


const float noteFrequencies[7] = {
    261.63, 329.63, 392.00, 523.25, // No more tweaks, no more fuss,
    392.00, 329.63, 261.63,

};
// float gyroX, gyroY, gyroZ;
// float alpha = 0; // ratio of gyro to acc usage, (relative to gyro)

bool beeping = false;
int posture_delay = 1000; //miliseconds
float now, then;
float accPitch, accRoll;

bool debounce1 = false;
float pfreedom = 18;

float rfreedom = 18;

//const float noiseThreshold = 0.0;

void readData(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_ADDR, 8, true);

  // 1g = 2^14 = 16394 steps
  // each acc and gyro value are 2 bytes

  accX = ((Wire.read()<<8 | Wire.read()) - accX_offset) / 16384.0;
  accY = ((Wire.read()<<8 | Wire.read()) - accY_offset) / 16384.0;
  accZ = ((Wire.read()<<8 | Wire.read()) - accZ_offset) / 16384.0;

  Wire.read();Wire.read();

  // gyroX = ((Wire.read()<<8 | Wire.read()) - gyroX_offset) / 131.0;
  // gyroY = ((Wire.read()<<8 | Wire.read()) - gyroY_offset) / 131.0;
  // gyroZ = ((Wire.read()<<8 | Wire.read()) - gyroZ_offset) / 131.0;
}

void calibration(){
  int32_t ax = 0, ay = 0, az = 0;
  // int32_t gx = 0, gy = 0, gz = 0;

  const int samples = 500;
  for (int i = 0; i<samples; i++){
    Wire.beginTransmission(MPU_ADDR);
    // begin reading at 0x3B
    // mpu6050 registries
    // https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
    Wire.write(0x3B);
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, 8, true);

    ax += (Wire.read()<<8 | Wire.read());
    ay += (Wire.read()<<8 | Wire.read());
    az += (Wire.read()<<8 | Wire.read());

    Wire.read();Wire.read();

    // gx += (Wire.read()<<8 | Wire.read());
    // gy += (Wire.read()<<8 | Wire.read());
    // gz += (Wire.read()<<8 | Wire.read());

    delay(5);
  }
  for (int i = 0; i<7; i++){
    tone(9, noteFrequencies[i]);
    delay(500);}


  accX_offset = ax / samples - 16384.0;
  accY_offset = ay / samples;
  accZ_offset = (az / samples);

  // gyroX_offset = gx / samples;
  // gyroY_offset = gy / samples;
  // gyroZ_offset = gz / samples;
}

void setup(){
  pinMode(9, OUTPUT);
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

  // last = millis();
}

void loop(){
  
  readData();

  // now = millis();
  // float dt = (now-last)/1000.0;
  // last = now;

  accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
  accRoll  = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // pitch += gyroY*dt;
  // roll += gyroX*dt;

  Serial.print("Pitch: ");
  Serial.print(accPitch, 0);
  Serial.print("°, Roll: ");
  Serial.print(accRoll, 0);
  Serial.println("°");

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




  // checking posture
  if (pfreedom<abs(accPitch) || (90-rfreedom)>abs(accRoll)) {
    now = millis();
    if (beeping == false){
      then = millis();
      beeping = true;
    }
    // wait 2 secs for constant bent posture
    if (now - then > posture_delay){
      tone(9, 1000);
    }
  }
  else{
    noTone(9);
    beeping = false;
  }
  
  if (digitalRead(4) && debounce1 == false){
    debounce1 = true;
    calibration();
  } 

  if (not digitalRead(4)){
    debounce1 = false;
  }
  // Serial.print("Pitch: ");
  // Serial.print(pitch, 0);
  // Serial.print("°, Roll: ");
  // Serial.print(roll, 0);
  // Serial.println("°");

}
