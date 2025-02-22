#include "Arduino.h"
#include "Imu.h"
#include "helper.h"
#include "every_sync_configuration.h"

Imu::Imu(ros::NodeHandle *nh, const String &topic, const int rate_hz,
         Timer &timer)
    : Sensor(nh, topic, rate_hz, timer, imu_msg_), kMaxRecursiveUpdateDepth(5u),
      kImuSyncTimeoutUs(4000) {}

void Imu::triggerMeasurement() {
  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (Imu.cpp): Timer interrupt but not overflown.").c_str());
    return;
  }

  DEBUG_PRINTLN((topic_ + " (Imu.cpp): Trigger.").c_str());
  Sensor::setTimestampNow();
  Sensor::newMeasurementIsAvailable();

#ifdef ADD_TRIGGERS
  trigger(ADDITIONAL_TEST_PIN, TRIGGER_PULSE_US,
          NON_INVERTED);
#endif
  // Reset the timer.
  timer_.resetOverflow();
}

void Imu::publish() {
  if (Sensor::isNewMeasurementAvailable()) {
    DEBUG_PRINTLN((topic_ + " (Imu.cpp): Publish."));
    bool success;
#ifdef DEBUG
    success = updateDataIterative();
#else
    success = updateDataIterative();
#endif
    if (!success) {
      Sensor::newMeasurementIsNotAvailable();
      DEBUG_PRINTLN((topic_ + " (Imu.cpp): IMU update failed.").c_str());
    } else {
      imu_msg_.time.data = Sensor::getTimestamp();
      // if (sensor_data_ == nullptr) {
      //   error((topic_ + " (Imu.cpp): sensor_data == nullptr").c_str(), 10);
      // }
      // imu_msg_.gx = sensor_data_[ImuReading::GX];
      // imu_msg_.gy = sensor_data_[ImuReading::GY];
      // imu_msg_.gz = sensor_data_[ImuReading::GZ];
      // imu_msg_.ax = sensor_data_[ImuReading::AX];
      // imu_msg_.ay = sensor_data_[ImuReading::AY];
      // imu_msg_.az = sensor_data_[ImuReading::AZ];
      imu_msg_.gx = 4;
      imu_msg_.gy = 5;
      imu_msg_.gz = 6;
      imu_msg_.ax = 1;
      imu_msg_.ay = 2;
      imu_msg_.az = 3;
#ifndef DEBUG
      publisher_.publish(&imu_msg_);
#endif
      Sensor::newMeasurementIsNotAvailable();
    }
  }
}

void Imu::setupPublisher() {
  publisher_ = ros::Publisher(topic_.c_str(), &imu_msg_);
  DEBUG_PRINT((topic_ + " (Imu.cpp): Setup publisher with topic ").c_str());
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}

void Imu::begin() {
  DEBUG_PRINTLN((topic_ + " (Imu.cpp): Begin.").c_str());
  Sensor::setupTimer();
}
