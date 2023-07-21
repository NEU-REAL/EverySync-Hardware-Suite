#include <stdio.h>
#include "GNSS.h"
#include "helper.h"
#include "every_sync_configuration.h"
#include "string.h"

GNSS::GNSS(ros::NodeHandle *nh, const String &topic, const int rate_hz,
               Timer &timer, const trigger_type &type,
               const uint8_t trigger_pin /*= 0 */,
			         const uint8_t exposure_pin /*= 0 */,
               const bool exposure_compensation /*= true*/)
    : Sensor(nh, topic, rate_hz, timer, pps_time_msg_, type),
      trigger_pin_(trigger_pin), exposure_pin_(exposure_pin),
      exposure_compensation_(exposure_compensation), is_configured_(true),
      compensating_(false), exposing_(false), pps_number_(0),
      init_subscriber_((topic + "init").c_str(), &GNSS::initCallback, this),
      initialized_(false) {
  // Check the input.

  if (exposure_pin == 0) {
    error((topic_ + " (GNSS.cpp): Exposure pin is set to 0.").c_str(), 10);
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

void GNSS::setup() {
  // Start Serial1 for IMU communication
//   Serial1.begin(9600, SERIAL_8N1);
  // delay(20);
  // Serial.setTimeout(2);
  
  if (topic_.length() == 0) {
    // GNSS without a topic are considered as disconnected.
    DEBUG_PRINTLN(
        F("NO_TOPIC (GNSS.cpp): Skip GNSS setup for disconnected GNSS."));
    is_configured_ = false;
    initialized_ = true;
    return;
  }
  DEBUG_PRINTLN((topic_ + " (GNSS.cpp): Setup.").c_str());

  setupInitSubscriber();
  setupPublisher();

//   pinMode(trigger_pin_, OUTPUT);
//   digitalWrite(trigger_pin_, LOW);
  pinMode(trigger_pin_, INPUT);
  pinMode(exposure_pin_, INPUT);
}

void GNSS::initialize() {
  if (!is_configured_ || initialized_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (GNSS.cpp): Initialize.").c_str());
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

void GNSS::begin() {
  if (!is_configured_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (GNSS.cpp): Begin.").c_str());
  // Maximal exposure time to still be able to keep up with the frequency
  // considering a security factor of 0.99, in us.
  // max_exposure_time_us_ = 0.99 * 1e6 / rate_hz_;

  // Setup timer to periodically trigger the GNSS.

  // Sensor::setupTimer();
}

void GNSS::setupPublisher() {
  pub_topic_ = topic_+ "external_time" ;
  publisher_ = ros::Publisher(pub_topic_.c_str(), &pps_time_msg_);
  DEBUG_PRINT((topic_ + " (GNSS.cpp): Setup publisher with topic ").c_str());
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}

void GNSS::setupInitSubscriber() {
  init_sub_topic_ = topic_ + "init";
  init_subscriber_ = ros::Subscriber<std_msgs::Bool, GNSS>(
      init_sub_topic_.c_str(), &GNSS::initCallback, this);
  DEBUG_PRINT(
      (topic_ + " (GNSS.cpp): Setup init subscriber with topic ").c_str());
  DEBUG_PRINTLN(init_subscriber_.topic_);
#ifndef DEBUG
  nh_->subscribe(init_subscriber_);
#endif
}

void GNSS::initCallback(const std_msgs::Bool &msg) {
  initialized_ = msg.data;
}

void GNSS::triggerMeasurement() {
  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    DEBUG_PRINTLN(
        (topic_ + " (GNSS.cpp): Timer interrupt but not overflown.").c_str());
    return;
  }
  if (!is_configured_ || !initialized_) {
    return;
  }
  DEBUG_PRINTLN((topic_ + " (GNSS.cpp): Timer overflow.").c_str());

  if (exposure_compensation_ && compensating_) {
    DEBUG_PRINTLN((topic_ + " (GNSS.cpp): Compensating.").c_str());
    // Exposure-time compensating mode (Nikolic 2014). During exposure, the
    // timer will interrupt in the middle of the exposure time. At the end of
    // the exposure, the external interrupt will trigger exposureEnd() and
    // reset the timer to trigger the GNSS at the appropriate time.
    if (!exposing_) {
      DEBUG_PRINTLN(
          (topic_ + " (GNSS.cpp): Not exposing. Trigger GNSS.").c_str());
      // The GNSS is currently not exposing meaning that the interrupt
      // triggers at the beginning of the next image.
      exposing_ = true;

      // Trigger the actual pulse.
      Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

      // Save the current time to estimate the exposure time in the pin
      // interrupt.
      exposure_start_us_ = micros();

      // Increament the image number as the GNSS is triggered now.
      ++pps_number_;

      // Set the timer to the mid exposure point, e.g. half the exposure time.
      timer_.setCompare(exposure_delay_ticks_ > 0 ? exposure_delay_ticks_ - 1
                                                  : compare_);
    } else {
      DEBUG_PRINTLN(
          (topic_ + " (GNSS.cpp): Exposing right now, get timestamp.")
              .c_str());
      // The GNSS is currently exposing. In this case, the interrupt is
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
    // "Standard" mode where the GNSS is triggered purely periodic.
    DEBUG_PRINTLN(
        (topic_ +
         " (GNSS.cpp): Not compensating. Trigger GNSS and take timestamp.")
            .c_str());
    exposing_ = true;

    // Trigger the actual pulse.
    Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

    Sensor::setTimestampNow();
    Sensor::newMeasurementIsAvailable();

    // Save the current time to estimate the exposure time in the pin
    // interrupt.
    exposure_start_us_ = micros();

    // Increament the image number as the GNSS is triggered now.
    pps_number_++;

    // Set the timer to make sure that the GNSS is triggered in a periodic
    // mode.
    timer_.setCompare(compare_);
  }
  // Reset the timer.
  timer_.resetOverflow();
}

void GNSS::exposureEnd() {
  DEBUG_PRINTLN((topic_ + " (GNSS.cpp): Exposure end.").c_str());
  if (exposure_compensation_) {
    unsigned long last_exposure_time_us = micros() - exposure_start_us_;
    DEBUG_PRINT((topic_ + " (GNSS.cpp): exposure time [us] ").c_str());
    DEBUG_PRINTDECLN(last_exposure_time_us);
    calculateDelayTicksAndCompensate(last_exposure_time_us);
    exposing_ = false;
  }
}

void GNSS::publish() {
  if (Sensor::isNewMeasurementAvailable()) {
    DEBUG_PRINTLN((topic_ + " (GNSS.cpp): Publish.").c_str());
    pps_time_msg_.time = Sensor::getTimestamp();
    pps_time_msg_.number = pps_number_;
#ifndef DEBUG
    publisher_.publish(&pps_time_msg_);
#endif
    Sensor::newMeasurementIsNotAvailable();
  }
}

void GNSS::calculateDelayTicksAndCompensate(
    const unsigned long &last_exposure_time_us) {
  // The goal here is to shift the time of the next GNSS trigger half the
  // exposure time before the mid-exposure time.

  // The next frame should be triggered by this time before the mid-exposure
  // time (in CPU ticks).
  if (last_exposure_time_us == 0 ||
      last_exposure_time_us >= max_exposure_time_us_) {
    // In this case, something with the measurement went wrong or the GNSS
    // is dropping triggers due to a too high exposure time (constrain the
    // maximal exposure time of your GNSS to be within the period time of
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

void GNSS::get_ext_clk_time() {
    Sensor::setTimestampNow();
    Sensor::newMeasurementIsAvailable();
	pps_number_++;
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


// Global Variable UTC 
utctime UTC;
// ros时间转UTC时间 输出String类型可用于UART发送
String build_GPRMC_CMD(ros::Time time_from_ros)
{
	int timecnt_start = micros();//us

	unsigned int Unix_sec = time_from_ros.sec;
	unsigned int Unix_nsec = time_from_ros.nsec;
	unsigned int Unix_msec,Unix_usec;

	int year=1970,mon=0,day=0,tday=0;
	day  = Unix_sec/(24*60*60);
	while(day >= 365)
	{
		if((year%400==0)||((year%4==0)&&(year%100!=0)))	{day-=366;}
		else											{day-=365;}
		year++;
	}
	for(mon=1;mon<13;mon++)
	{
		tday = day;
		switch(mon)
		{
			case 1:	
			case 3:
			case 5:
			case 7:
			case 8:	
			case 10:
			case 12:day-=31;
					break;
			case 4:	
			case 6:
			case 9:
			case 11:day-=30;
					break;							
			case 2:if((year%400==0)||((year%4==0)&&(year%100!=0)))	{ day-=29;}
			       else												{ day-=28;}
			       break;
		}
        if(day < 0)
		{
		  day = tday;
          break;
        }
	}
	// 更改为在头文件中定义全局变量 方便调用与查看
	// utctime UTC;
	UTC.year = year;
	UTC.mon  = mon;
	UTC.day  = day+1;
	UTC.hour = Unix_sec%(24*60*60)/(60*60);
	UTC.min  = Unix_sec%(24*60*60)%(60*60)/60;
	UTC.sec  = Unix_sec%(24*60*60)%(60*60)%60;
	

    int timecnt_end = micros();//test for time_delay of build_GPRMC_CMD(ros::time) us
    int timecnt_length = - ( timecnt_end  - timecnt_start ) ;//us

	//补偿时间转换函数程序执行时间 
	Unix_nsec = Unix_nsec + timecnt_length * 1000 ;//ns 
	if (Unix_nsec >= 1e9 ){
		Unix_nsec = Unix_nsec - 1e9;
		UTC.sec = UTC.sec + 1 ;
	}

	utctime_String UTC_str;
	// year
	UTC_str.utc_year = String(UTC.year % 1000);//2位
	// month
	if(UTC.mon >= 10){
		UTC_str.utc_mon  = String(UTC.mon);
		}
	else{
		UTC_str.utc_mon = "0"+String(UTC.mon);//补0
		}
	// day
	if(UTC.day >= 10){
		UTC_str.utc_day  = String(UTC.day);
		}
	else{
		UTC_str.utc_day = "0"+String(UTC.day);//补0
		}
	// hour
	if(UTC.hour >= 10){
		UTC_str.utc_hour  = String(UTC.hour);
		}
	else{
		UTC_str.utc_hour = "0"+String(UTC.hour);//补0
		}
	// minute
	if(UTC.min >= 10){
		UTC_str.utc_min  = String(UTC.min);
		}
	else{
		UTC_str.utc_min = "0"+String(UTC.min);//补0
		}
	// sec
	if(UTC.sec >= 10){
		UTC_str.utc_sec  = String(UTC.sec);
		}
	else{
		UTC_str.utc_sec = "0"+String(UTC.sec);//补0
		}

	// ms 保留4位
	Unix_usec = Unix_nsec / 1000 ;
	if(Unix_usec >= 1e5){
		UTC_str.utc_ms = String(Unix_usec);
		}
	else if (Unix_usec >= 1e4){
		UTC_str.utc_ms = "0" + String(Unix_usec);
		}
	else if (Unix_usec >= 1e3){
		UTC_str.utc_ms = "00" + String(Unix_usec);
		}
	else if (Unix_usec >= 1e2){
		UTC_str.utc_ms = "000" + String(Unix_usec);
		}
	else if (Unix_usec >= 1e1){
		UTC_str.utc_ms = "0000" + String(Unix_usec);
		}
	else{
		UTC_str.utc_ms = "00000" + String(Unix_usec);//缺位补0
		}


	// Livox 
	// UTC_str.utc_ms = UTC_str.utc_ms[0] + UTC_str.utc_ms[1] + UTC_str.utc_ms[2] ;

	String GPMRC_cmd;
	send_msg GPMRC_send;
    // Serial1.println("$GPRMC,222120.3456,A,2237.496474,N,11356.089515,E,0.0,225.5,010523,2.3,W,A*23");
	//                  $GPRMC,010101.130 ,A,3606.6834  ,N,12021.7778  ,E,0.0,238.3,010807,   , ,A*6C
    // HEAD
    GPMRC_send.part1 = "$GPRMC,";//Ouster & Robosense
    // UTC TIME (hhmmss.ssss)
    GPMRC_send.part2 = UTC_str.utc_hour + UTC_str.utc_min + UTC_str.utc_sec + ".";
    GPMRC_send.part3 = UTC_str.utc_ms + ",";
    // MID
    // GPMRC_send.part4 = "A,2237.496474,N,11356.089515,E,0.0,225.5,";
	GPMRC_send.part4 = "A,3606.6834,N,12021.7778,E,0.0,238.3,";
    // UTC DATE (DDMMYY)
    GPMRC_send.part5 = UTC_str.utc_day + UTC_str.utc_mon + UTC_str.utc_year + ",";
    // LAST
    // GPMRC_send.part6 = "2.3,W,A*";
	GPMRC_send.part6 = ",,A*";

    GPMRC_cmd = GPMRC_send.part1 + GPMRC_send.part2 + GPMRC_send.part3
              + GPMRC_send.part4 + GPMRC_send.part5 + GPMRC_send.part6;
	// Calculate Checksum
	int i, result;
	for(result=GPMRC_cmd[1],i=2;GPMRC_cmd[i]!='*';i++)
	{
		result^=GPMRC_cmd[i];
	}
	GPMRC_send.part7 = String(result,HEX);


	GPMRC_cmd = GPMRC_cmd + GPMRC_send.part7 ;


	return GPMRC_cmd;
}


