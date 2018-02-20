#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

MPU6050::MPU6050() {
  _gravAccelX = 0; _gravAccelY = 0; _gravAccelZ = 0;
  _offsetGyroX = 0; _offsetGyroY = 0; _offsetGyroZ = 0;
  _accelX = 0; _accelY = 0; _accelZ = 0;
  _gyroX = 0; _gyroY = 0; _gyroZ = 0;
  _aOK = true;

}

/**
 *  Activates the MPU6050 and sets the config registers for the gyro and accelerometer.
 *  
 *  Returns true if successful.
 */
bool MPU6050::initialize() {

  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0);
  byte aOK = Wire.endTransmission(true);

  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(REG_GYRO_CONFIG);
  Wire.write(FS_SEL << 3);
  aOK |= Wire.endTransmission(true);

  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(REG_ACCEL_CONFIG);
  Wire.write(AFS_SEL << 3);
  aOK |= Wire.endTransmission(true);

  return !aOK;
}

/**
 *  Computes the average reading of the accelerometer and gyro.
 *  This method should only be called when the device is not moving.
 *  
 *  Returns true if successful.
 */
bool MPU6050::calibrate() {
  int16_t tmpAX, tmpAY, tmpAZ, tmpGX, tmpGY, tmpGZ;
  bool aOK = false;
  
  for (int i = 0; i < CALIBRATE_SIZE; i++) {
    aOK |= !getRawSensorData(tmpAX, tmpAY, tmpAZ, tmpGX, tmpGY, tmpGZ);
    _gravAccelX += (float)tmpAX / CALIBRATE_SIZE;
    _gravAccelY += (float)tmpAY / CALIBRATE_SIZE;
    _gravAccelZ += (float)tmpAZ / CALIBRATE_SIZE;
    _offsetGyroX += (float)tmpGX / CALIBRATE_SIZE;
    _offsetGyroY += (float)tmpGY / CALIBRATE_SIZE;
    _offsetGyroZ += (float)tmpGZ / CALIBRATE_SIZE;
    delay(1);
  }

  _gravAccelX /= ACCEL_LSB_SENSITIVITY;
  _gravAccelY /= ACCEL_LSB_SENSITIVITY;
  _gravAccelZ /= ACCEL_LSB_SENSITIVITY;
  
  return !aOK;
}

/**
 *  Reads the raw sensor data from the accelerometer and gyro.
 *  
 *  Returns true if successful.
 */
bool MPU6050::getRawSensorData(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(REG_GYRO_XOUT_H);
  byte aOK = Wire.endTransmission(false);

  Wire.requestFrom(IMU_ADDRESS, 14, true);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  return !aOK;
}

/**
 *  Gets calibrated sensor data from the accelerometer and gyro.
 *  Accelerometer data is given in g's (~9.8 m/s^2) and gyro data is in degrees per second.
 *  
 *  Returns true if successful.
 */
bool MPU6050::getSensorData(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ) {
  
  accelX = _accelX;
  accelY = _accelY;  
  accelZ = _accelZ;  
  gyroX = _gyroX;
  gyroY = _gyroY;
  gyroZ = _gyroZ;
  
  return _aOK;
}

void MPU6050::updateSensorData() {
  int16_t tmpAX, tmpAY, tmpAZ, tmpGX, tmpGY, tmpGZ;
  _aOK = getRawSensorData(tmpAX, tmpAY, tmpAZ, tmpGX, tmpGY, tmpGZ);
  
  _accelX = (float) tmpAX / ACCEL_LSB_SENSITIVITY;
  _accelY = (float) tmpAY / ACCEL_LSB_SENSITIVITY;  
  _accelZ = (float) tmpAZ / ACCEL_LSB_SENSITIVITY;  
  _gyroX = ((float) tmpGX - _offsetGyroX) / GYRO_LSB_SENSITIVITY;
  _gyroY = ((float) tmpGY - _offsetGyroY) / GYRO_LSB_SENSITIVITY;
  _gyroZ = ((float) tmpGZ - _offsetGyroZ) / GYRO_LSB_SENSITIVITY;
}

/**
 *  Returns the initial acceleration of the device.
 */
void MPU6050::getInitAccel(float &aX, float &aY, float &aZ) {
  aX = _gravAccelX;
  aY = _gravAccelY;
  aZ = _gravAccelZ;
}

