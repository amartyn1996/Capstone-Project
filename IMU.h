#ifndef IMU_H
#define IMU_H

class IMU {

  public:
    virtual bool initialize() = 0;
    virtual bool calibrate() = 0;
    virtual void getInitAccel(float &aX, float &aY, float &aZ) = 0;
    virtual bool getSensorData(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ) = 0;
    virtual bool getRawSensorData(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) = 0;
    virtual void updateSensorData();
    
};

#endif
