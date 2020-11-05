////////////////////////////////////////////
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <math.h>
#include <stdio.h> //Scheduler


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//#include <Wire.h>
//#include <SPI.h>
#include "MS5611.h"
#include "MPU9250.h"
#include "MadgwickAHRS.h"

/*-----msgs-----*/
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <james_msgs/Barometer.h>
///////////////////////////////////////////////////////
ros::NodeHandle nh;

/*-------TF-------*/
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

/*-------IMU-------*/
sensor_msgs::Imu imu;
ros::Publisher pub("imumsgs_mpu9250", &imu);
MPU9250 IMU(SPI, 11);
MS5611 MS5611(12);
float acc[3], gyro[3], mag[3];
float D1, D2;
uint16_t fc[6];
int64_t OFF2, SENS2;
int32_t TEMP2;

/*----baromter----*/
james_msgs::Barometer baro_msg;
ros::Publisher pub_baro("/barometer", &baro_msg);
float referencePressure;

/*----Ultrasound----*/
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

const int adc_pin = 0;

char frameid[] = "/ultrasound";

double getRange_Ultrasound(int pin_num) {
  int val = 0;
  for (int i = 0; i < 4; i++) val += analogRead(pin_num);
  double range =  val;
  return range / 322.519685;  // (0.0124023437 /4) ; //cvt to meters
}


/*----Scheduler----*/
const unsigned long extra_FREQUENCY = 0;

unsigned long IMU_RUNTIME = 11;
unsigned long BARO_RUNTIME = 22;
unsigned long SONAR_1_RUNTIME = 0;
unsigned long extra_RUNTIME = 0;

unsigned long IMU_COUNT = 0;
unsigned long BARO_COUNT = 0;
unsigned long SONAR_1_COUNT = 0;
unsigned long extra_COUNT = 0;

unsigned long IMU_CYCLE_T = 1000 / IMU_FREQUENCY;
unsigned long Baro_CYCLE_T = 1000 / BARO_FREQUENCY;
unsigned long SONAR_1_CYCLE_T = 1000 / SONAR_1_FREQUENCY;
unsigned long extra_CYCLE_T = 0;
/////////////////////////////////////////////////////////////////////





void pub_IMU() {
  if (millis() >= IMU_CYCLE_T * IMU_COUNT) {
    IMU.readSensor();
    acc[0] = IMU.getAccelY_mss();
    acc[1] = IMU.getAccelX_mss();
    acc[2] = -IMU.getAccelZ_mss();

    gyro[0] = IMU.getGyroY_rads();
    gyro[1] = IMU.getGyroX_rads();
    gyro[2] = -IMU.getGyroZ_rads();

//    mag[0] = IMU.getMagY_uT();
//    mag[1] = IMU.getMagX_uT();
//    mag[2] = -IMU.getMagZ_uT();
    MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
//    MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], -acc[0], -acc[1], -acc[2], -mag[0], -mag[1], -mag[2]); //with mag

    imu.header.frame_id = "imu_link_9250";
    imu.header.stamp = nh.now();
    imu.orientation.w = q0;
    imu.orientation.x = q1;
    imu.orientation.y = q2;
    imu.orientation.z = q3;

    imu.orientation_covariance[0] = 0.0025;
    imu.orientation_covariance[1] = 0;
    imu.orientation_covariance[2] = 0;
    imu.orientation_covariance[3] = 0;
    imu.orientation_covariance[4] = 0.0025;
    imu.orientation_covariance[5] = 0;
    imu.orientation_covariance[6] = 0;
    imu.orientation_covariance[7] = 0;
    imu.orientation_covariance[8] = 0.0025;

    imu.linear_acceleration.x = acc[0];
    imu.linear_acceleration.y = acc[1];
    imu.linear_acceleration.z = acc[2];

    imu.linear_acceleration_covariance[0] = 0.04;
    imu.linear_acceleration_covariance[1] = 0;
    imu.linear_acceleration_covariance[2] = 0;
    imu.linear_acceleration_covariance[3] = 0;
    imu.linear_acceleration_covariance[4] = 0.04;
    imu.linear_acceleration_covariance[5] = 0;
    imu.linear_acceleration_covariance[6] = 0;
    imu.linear_acceleration_covariance[7] = 0;
    imu.linear_acceleration_covariance[8] = 0.04;

    imu.angular_velocity.x = gyro[0];
    imu.angular_velocity.y = gyro[1];
    imu.angular_velocity.z = gyro[2];

    imu.angular_velocity_covariance[0] = 0.02;
    imu.angular_velocity_covariance[1] = 0;
    imu.angular_velocity_covariance[2] = 0;
    imu.angular_velocity_covariance[3] = 0;
    imu.angular_velocity_covariance[4] = 0.02;
    imu.angular_velocity_covariance[5] = 0;
    imu.angular_velocity_covariance[6] = 0;
    imu.angular_velocity_covariance[7] = 0;
    imu.angular_velocity_covariance[8] = 0.02;





    pub.publish(&imu);
    IMU_COUNT++;
  }
}

void pub_Baro() {
  if (millis() >= Baro_CYCLE_T * BARO_COUNT) {
    baro_msg.header.frame_id = "/barometer";
    baro_msg.header.stamp = nh.now();

    int result_1 = MS5611.read(12);
    float absolutealt = MS5611.getAltitude(MS5611.getPressure() * 0.01); //cm
    float relativealt = MS5611.getAltitude(MS5611.getPressure() * 0.01, referencePressure);
    float temperature = MS5611.getTemperature() * 0.01; //c

    baro_msg.temperature = temperature;
    baro_msg.absolute_alt = absolutealt;
    baro_msg.relative_alt = relativealt;
    pub_baro.publish(&baro_msg);
    BARO_COUNT++;
  }
}

void pub_Sonar() {
  if (millis() >= SONAR_1_CYCLE_T * SONAR_1_COUNT) {

    range_msg.range = getRange_Ultrasound(5);
    if (range_msg.range < range_msg.min_range || range_msg.range > range_msg.max_range ) range_msg.range = range_msg.max_range;
    range_msg.header.frame_id = "sonar";
    range_msg.header.stamp = nh.now();


    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view = 0.6;  // fake
    range_msg.min_range = 0.15;
    range_msg.max_range = 1.5;

    pub_range.publish(&range_msg);


    SONAR_1_COUNT++;
  }
}
