#ifndef _BAROMETER_H_
#define _BAROMETER_H_

#include "Arduino.h"
#include <MS5611.h>

//MS5611 기압계를 사용하기 위한 I2C 핀 : SDA = A4,
//                                   SCL = A5

//MS5611 ms5611;    //기압계 인스턴스

class Barometer
{
  private:
    MS5611* baro_;
    double reference_pressure_;

  public:
    Barometer();
    ~Barometer();
    Barometer& setReference();
    double getAbsoluteAltitude() const;
    double getRelativeAltitude() const;
    double getTemperature() const;
};

Barometer::Barometer() {
  baro_ = new MS5611;
}

Barometer::~Barometer() {
  delete baro_;
}

Barometer& Barometer::setReference(){
  while (!baro_->begin()){
//    throw String("Could not find a valid MS5611 sensor, check wiring!")
  }
  reference_pressure_ = baro_->readPressure();
  return *this;
}

double Barometer::getAbsoluteAltitude() const{
  return baro_->getAltitude(baro_->readPressure());
}

double Barometer::getRelativeAltitude() const{
  return baro_->getAltitude(baro_->readPressure(), reference_pressure_);
}

double Barometer::getTemperature() const{
  return baro_->readTemperature(); 
}


#endif
