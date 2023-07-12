////////////////////////////////////////////////////////////////////////////////
//  April 2023
//  Author: Kangkang <1035309950@qq.com>
////////////////////////////////////////////////////////////////////////////////
//  Lidar.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding cameras in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////

#include "Lidar.h"
#include "helper.h"
#include "every_sync_configuration.h"
#include "GNSS.h"

#include <ros/time.h>
#include <tf/transform_broadcaster.h>
extern geometry_msgs::TransformStamped t;
extern tf::TransformBroadcaster broadcaster;
extern utctime UTC;

Lidar::Lidar(ros::NodeHandle *nh, const String &topic, const int rate_hz,
               Timer &timer, const trigger_type &type,
               const uint8_t trigger_pin /*= 0 */,
               const uint8_t exposure_pin /*= 0 */,
               const bool exposure_compensation /*= true*/)
    : Sensor(nh, topic, rate_hz, timer, pps_time_msg_, type),
      trigger_pin_(trigger_pin), exposure_pin_(exposure_pin),
      exposure_compensation_(exposure_compensation), is_configured_(true),
      compensating_(false), exposing_(false), pps_number_(0),
      init_subscriber_((topic + "init").c_str(), &Lidar::initCallback, this),
      initialized_(false) {
  // Check the input.
  if (trigger_pin == 0) {
    error((topic_ + " (Camera.cpp): Trigger pin is set to 0.").c_str(), 10);
  }
  if (exposure_pin == 0) {
    error((topic_ + " (Camera.cpp): Exposure pin is set to 0.").c_str(), 10);
  }
  Sensor::newMeasurementIsNotAvailable();
}

//GPRMC格式样例 
// $GPRMC,222120.3456,A,2237.496474,N,11356.089515,E,0.0,225.5,010523,2.3,W,A*23
//    0      1        2      3      4       5      6  7     8     9   10 11 12 *13
// field 0：$GPRMC, 格式ID，表示该格式为建议的最低特定GPS / TRANSIT数据（RMC）推荐最低定位信息
// field 1: UTC时间, 格式hhmmss.ssss，代表时分秒.毫秒
// field 2: 状态 A:代表定位成功 V:代表定位失败 
// field 3: 纬度 ddmm.mmmmmm 度格式（如果前导位数不足，则用0填充）
// field 4: 纬度 N(北纬)  S(南纬)
// field 5: 经度 dddmm.mmmmmm 度格式（如果前导位数不足，则用0填充）
// field 6: 经度 E(东经) W(西经)
// field 7: 速度（也为1.852 km / h）
// field 8: 方位角，度（二维方向，等效于二维罗盘）
// field 9: UTC日期 DDMMYY 天月年
// field 10: 磁偏角（000-180）度，如果前导位数不足，则用0填充）
// field 11: 磁偏角方向E =东W =西
// field 12: 模式，A =自动，D =差分，E =估计，AND =无效数据（3.0协议内容）
// field 13: 校验和

void Lidar::setup() {
  // Start Serial1 for IMU communication
  Serial1.begin(9600, SERIAL_8N1);
  // delay(20);
  // Serial.setTimeout(2);
  
  if (topic_.length() == 0) {
    // Lidar without a topic are considered as disconnected.
    DEBUG_PRINTLN(
        F("NO_TOPIC (Lidar.cpp): Skip Lidar setup for disconnected Lidar."));
    is_configured_ = false;
    initialized_ = true;
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Lidar.cpp): Setup.").c_str());

  setupInitSubscriber();
  setupPublisher();

  pinMode(trigger_pin_, OUTPUT);
  digitalWrite(trigger_pin_, LOW);
  pinMode(exposure_pin_, INPUT);
}

void Lidar::initialize() {
  if (!is_configured_ || initialized_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Lidar.cpp): Initialize.").c_str());
  // Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US * 10, type_);
  Sensor::setTimestampNow();
  // ++pps_number_;
  pps_time_msg_.number = pps_number_;
  Sensor::newMeasurementIsAvailable();
  publish();
#ifdef DEBUG
  initialized_ = true;
#endif
  initialized_ = true;//强制初始化
}

void Lidar::begin() {
  if (!is_configured_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Lidar.cpp): Begin.").c_str());
  // Maximal exposure time to still be able to keep up with the frequency
  // considering a security factor of 0.99, in us.
  max_exposure_time_us_ = 0.99 * 1e6 / rate_hz_;

  // Setup timer to periodically trigger the Lidar.
  Sensor::setupTimer();
}

void Lidar::setupPublisher() {
  pub_topic_ = topic_ + "pps_time";
  publisher_ = ros::Publisher(pub_topic_.c_str(), &pps_time_msg_);
  DEBUG_PRINT((topic_ + " (Lidar.cpp): Setup publisher with topic ").c_str());
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}

void Lidar::setupInitSubscriber() {
  init_sub_topic_ = topic_ + "init";
  init_subscriber_ = ros::Subscriber<std_msgs::Bool, Lidar>(
      init_sub_topic_.c_str(), &Lidar::initCallback, this);
  DEBUG_PRINT(
      (topic_ + " (Lidar.cpp): Setup init subscriber with topic ").c_str());
  DEBUG_PRINTLN(init_subscriber_.topic_);
#ifndef DEBUG
  nh_->subscribe(init_subscriber_);
#endif
}

void Lidar::initCallback(const std_msgs::Bool &msg) {
  initialized_ = msg.data;
}

bool pps_trig_ = false;
bool pps_count_trig_ = false;
int pps_count_ = 0;
// 100hz generate pulse 1Hz 50ms High level
void Lidar::pps_send_utc_time_from_100hz() {
  // ros::Time pps_time_on_ros = getTimestamp();

  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (Lidar.cpp): Timer interrupt but not overflown.").c_str());
    return;
  }

  if (!is_configured_ || !initialized_) {
    return;
  }

  DEBUG_PRINTLN((topic_ + " (Lidar.cpp): Timer overflow.").c_str());

  // change Lidar class into lidar usage.
  // "Standard" mode where the Lidar is triggered purely periodic.
  if ((pps_number_ % 100) == 0)
  {
    pps_trig_ = true;
    pps_count_trig_ = true; 
  }
  if (pps_count_trig_ == true)
  {
    ++pps_count_ ;
  }

  if (pps_trig_ == true)//start trig
  {
    Sensor::setTimestampNow();
    Sensor::newMeasurementIsAvailable();
    // This part transform ros::time to UTC time.
    ros::Time pps_time_from_ros = getTimestamp();
    String GPMRC_CMD = build_GPRMC_CMD(pps_time_from_ros);
    // Trigger the PPS pulse.
    // Sensor::trigger(trigger_pin_, TRIGGER_PULSE_PPS, type_);
    digitalWrite(LIDAR_TRIGGER_PIN, HIGH);
    // Send NMEA cmd for lidar sync
    // Serial1.println(GPMRC_CMD);
    Serial1.println("$GPRMC,114345.130669,A,3606.6834,N,12021.7778,E,0.0,238.3,080523,,,A*50");

    pps_trig_ = false;
  }

  if (pps_count_ == 5 )//end trig
  {
    digitalWrite(LIDAR_TRIGGER_PIN, LOW);
    pps_count_ = 0 ;
    pps_count_trig_ = false ; 
  }
    // Increament the image number as the camera is triggered now.
    pps_number_++;

    // Set the timer to make sure that the camera is triggered in a periodic
    // mode.
    timer_.setCompare(compare_);

  // Reset the timer.
  timer_.resetOverflow();
}


void Lidar::pps_send_utc_time() {
  // ros::Time pps_time_on_ros = getTimestamp();

  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (Lidar.cpp): Timer interrupt but not overflown.").c_str());
    return;
  }

  if (!is_configured_ || !initialized_) {
    return;
  }

  DEBUG_PRINTLN((topic_ + " (Lidar.cpp): Timer overflow.").c_str());

    // change Lidar class into lidar usage.
    // "Standard" mode where the Lidar is triggered purely periodic.

    Sensor::setTimestampNow();
    Sensor::newMeasurementIsAvailable();

    // This part transform ros::time to UTC time.
    ros::Time pps_time_from_ros = getTimestamp();
    String GPMRC_CMD = build_GPRMC_CMD(pps_time_from_ros);

    // Trigger the PPS pulse.
    Sensor::trigger(trigger_pin_, TRIGGER_PULSE_PPS, type_);
    // Send NMEA cmd for lidar sync
    // Serial1.println(GPMRC_CMD);
    Serial1.println("GPRMC,222120.3456,A,2237.496474,N,11356.089515,E,0.0,225.5,010523,2.3,W,A*23");

    // Increament the image number as the camera is triggered now.
    pps_number_++;

    // Set the timer to make sure that the camera is triggered in a periodic
    // mode.
    timer_.setCompare(compare_);

  // Reset the timer.
  timer_.resetOverflow();
}

void Lidar::triggerMeasurement() {
  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (Camera.cpp): Timer interrupt but not overflown.").c_str());
    return;
  }
  if (!is_configured_ || !initialized_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (Camera.cpp): Timer overflow.").c_str());

  if (exposure_compensation_ && compensating_) {
    DEBUG_PRINTLN((topic_ + " (Camera.cpp): Compensating.").c_str());
    // Exposure-time compensating mode (Nikolic 2014). During exposure, the
    // timer will interrupt in the middle of the exposure time. At the end of
    // the exposure, the external interrupt will trigger exposureEnd() and
    // reset the timer to trigger the camera at the appropriate time.
    if (!exposing_) {
      DEBUG_PRINTLN(
          (topic_ + " (Camera.cpp): Not exposing. Trigger camera.").c_str());
      // The camera is currently not exposing meaning that the interrupt
      // triggers at the beginning of the next image.
      exposing_ = true;

      // Trigger the actual pulse.
      Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

      // Save the current time to estimate the exposure time in the pin
      // interrupt.
      exposure_start_us_ = micros();

      // Increament the image number as the camera is triggered now.
      ++pps_number_;

      // Set the timer to the mid exposure point, e.g. half the exposure time.
      timer_.setCompare(exposure_delay_ticks_ > 0 ? exposure_delay_ticks_ - 1
                                                  : compare_);
    } else {
      DEBUG_PRINTLN(
          (topic_ + " (Camera.cpp): Exposing right now, get timestamp.")
              .c_str());
      // The camera is currently exposing. In this case, the interrupt is
      // triggered in the middle of the exposure time, where the timestamp
      // should be taken.
      Sensor::setTimestampNow();
      Sensor::newMeasurementIsAvailable();
// #ifdef ADD_TRIGGERS
//       trigger(ADDITIONAL_TEST_PIN, TRIGGER_PULSE_US,
//               NON_INVERTED);
// #endif

      // Even though we are still in the compensating mode, deactivating here
      // ensures that we detect if a exposure signal is dropped and we switch
      // to non-compensating mode.
      compensating_ = false;

      // Set the timer to the standard period as we dont know the current
      // exposure time yet.
      timer_.setCompare(compare_);
    }
  } else {
    // "Standard" mode where the camera is triggered purely periodic.
    DEBUG_PRINTLN(
        (topic_ +
         " (Camera.cpp): Not compensating. Trigger camera and take timestamp.")
            .c_str());
    exposing_ = true;

    // Trigger the actual pulse.
    Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

    Sensor::setTimestampNow();
    Sensor::newMeasurementIsAvailable();

    // Save the current time to estimate the exposure time in the pin
    // interrupt.
    exposure_start_us_ = micros();

    // Increament the image number as the camera is triggered now.
    pps_number_++;

    // Set the timer to make sure that the camera is triggered in a periodic
    // mode.
    timer_.setCompare(compare_);
  }
  // Reset the timer.
  timer_.resetOverflow();
}

void Lidar::exposureEnd() {
  DEBUG_PRINTLN((topic_ + " (Camera.cpp): Exposure end.").c_str());
  if (exposure_compensation_) {
    unsigned long last_exposure_time_us = micros() - exposure_start_us_;
    DEBUG_PRINT((topic_ + " (Camera.cpp): exposure time [us] ").c_str());
    DEBUG_PRINTDECLN(last_exposure_time_us);
    calculateDelayTicksAndCompensate(last_exposure_time_us);
    exposing_ = false;
  }
}

void Lidar::publish() {
  if (Sensor::isNewMeasurementAvailable()) {
    DEBUG_PRINTLN((topic_ + " (Camera.cpp): Publish.").c_str());
    pps_time_msg_.time = Sensor::getTimestamp();
    pps_time_msg_.number = pps_number_;
#ifndef DEBUG
    publisher_.publish(&pps_time_msg_);
#endif
    Sensor::newMeasurementIsNotAvailable();
  }
}

void Lidar::calculateDelayTicksAndCompensate(
    const unsigned long &last_exposure_time_us) {
  // The goal here is to shift the time of the next camera trigger half the
  // exposure time before the mid-exposure time.

  // The next frame should be triggered by this time before the mid-exposure
  // time (in CPU ticks).
  if (last_exposure_time_us == 0 ||
      last_exposure_time_us >= max_exposure_time_us_) {
    // In this case, something with the measurement went wrong or the camera
    // is dropping triggers due to a too high exposure time (constrain the
    // maximal exposure time of your camera to be within the period time of
    // your triggering). Switch to non-compensating mode.
    exposure_delay_ticks_ = 0;
    compensating_ = false;
  } else {
    exposure_delay_ticks_ = static_cast<double>(last_exposure_time_us) / 2.0 /
                            1000000.0 * cpu_freq_prescaler_;
    compensating_ = true;
  }

  // Reset the compare register of the timer.
  timer_.setCompare(compare_ - exposure_delay_ticks_);
}
