#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"

#define IMU_ADDRESS 0x68
#define CALIBRATE_SIZE 100
#define ACCEL_LSB_SENSITIVITY 4096 
#define GYRO_LSB_SENSITIVITY 65.5

#define REG_PWR_MGMT_1 0x6b
#define REG_GYRO_CONFIG 0x1b
#define REG_ACCEL_CONFIG 0x1c
#define REG_GYRO_XOUT_H 0x3b
#define FS_SEL 1
#define AFS_SEL 2


class MPU6050 : virtual public IMU {
  
  public:
  MPU6050();
	bool initialize();
	bool calibrate();
	void getInitAccel(float &aX, float &aY, float &aZ);
	bool getRawSensorData(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ);
	bool getSensorData(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ);
  
  private:
    float _gravAccelX = 0, _gravAccelY = 0, _gravAccelZ = 0, _offsetGyroX = 0, _offsetGyroY = 0, _offsetGyroZ = 0;
	
};

#endif
