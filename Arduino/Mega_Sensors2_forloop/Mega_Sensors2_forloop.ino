const unsigned long IMU_FREQUENCY = 70;
const unsigned long SONAR_1_FREQUENCY = 20;
const unsigned long BARO_FREQUENCY = 15;

#include "Mega_Sensors2_forloop.h"

void setup() {
  SPI.begin();
  MS5611.init();
  IMU.begin();
  Serial.begin(57600);

//  int IMU_setGyroRange = IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
//  Serial.println(IMU_setGyroRange);
  int IMU_setAccelRange = IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  Serial.println(IMU_setAccelRange);
  int IMU_gyroStatus = IMU.calibrateGyro();
  Serial.println(IMU_gyroStatus);
  int IMU_accelStatus = IMU.calibrateAccel();
  Serial.println(IMU_accelStatus);
//  int IMU_magStatus = IMU.calibrateMag();
//  Serial.println(IMU_magStatus);
  float IMU_gyroBiasX = IMU.getGyroBiasX_rads();
  float IMU_gyroBiasY = IMU.getGyroBiasY_rads();
  float IMU_gyroBiasZ = IMU.getGyroBiasZ_rads();
  Serial.println(IMU_gyroBiasX);
  Serial.println(IMU_gyroBiasY);
  Serial.println(IMU_gyroBiasZ);
  float IMU_accelBiasX = IMU.getAccelBiasX_mss();
  float IMU_accelBiasY = IMU.getAccelBiasY_mss();
  float IMU_accelBiasZ = IMU.getAccelBiasZ_mss();
  Serial.println(IMU_accelBiasX);
  Serial.println(IMU_accelBiasY);
  Serial.println(IMU_accelBiasZ);
  float IMU_scaleFactorX = IMU.getAccelScaleFactorX();
  float IMU_scaleFactorY = IMU.getAccelScaleFactorY();
  float IMU_scaleFactorZ = IMU.getAccelScaleFactorZ();
  Serial.println(IMU_scaleFactorX);
  Serial.println(IMU_scaleFactorY);
  Serial.println(IMU_scaleFactorZ);
//  float IMU_magBiasX = IMU.getMagBiasX_uT();
//  float IMU_magBiasY = IMU.getMagBiasY_uT();
//  float IMU_magBiasZ = IMU.getMagBiasZ_uT();
//  Serial.println(IMU_magBiasX);
//  Serial.println(IMU_magBiasY);
//  Serial.println(IMU_magBiasZ);
//  float IMU_magFactorX = IMU.getMagScaleFactorX();
//  float IMU_magFactorY = IMU.getMagScaleFactorY();
//  float IMU_magFactorZ = IMU.getMagScaleFactorZ();
//  Serial.println(IMU_magFactorX);
//  Serial.println(IMU_magFactorY);
//  Serial.println(IMU_magFactorZ);




  ///// Reference_pressure setting :) //////
  int result = MS5611.read();
  referencePressure = MS5611.getPressure() * 0.01 ;

  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(pub);
  nh.advertise(pub_baro);
  nh.advertise(pub_range);


}

int count_IMU;
long range_time_imu, range_time_baro;

void loop()
{
  pub_IMU();
  pub_Baro();
  pub_Sonar();
  nh.spinOnce();

}
