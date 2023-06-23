rate 
ok 20 25 30 40 50 60 80 100    
no 35 70 90 120=73-78 150 =89-96  200=115-127 240= 116-130  320=133-157  400 =178-185

uint16_t Sensor::calculatePrescaler(const uint16_t rate_hz) const {
  // All timers on Arduino Zero are assumed to be set to 16 bit.
  const uint32_t kMaxCount = 65536;

  // Amount of clock ticks required to count to the specified period time.
  const uint32_t kRequiredTicks = CPU_FREQ_HZ / static_cast<double>(rate_hz);
  DEBUG_PRINT((topic_ + " (Sensor.cpp): required ticks "));
  DEBUG_PRINTDECLN(kRequiredTicks);
  // Available prescalers on Arduino Zero are restricted to those values.
  const uint16_t kAvailablePrescalers[8] = {1, 2, 4, 8, 16, 64, 256, 1024};

  for (uint8_t i = 0; i < 8; ++i) {
    if (kMaxCount > kRequiredTicks / kAvailablePrescalers[i] &&
        kRequiredTicks % kAvailablePrescalers[i] == 0) {
      return kAvailablePrescalers[i];
    }
  }
  // If this part is reached, no available prescaler fits with the goal
  // framerate on this MCU.
  error((topic_ + " (Sensor.cpp): No prescaler found.").c_str(), 50);
  return 0;
}

void Camera::triggerMeasurement() {
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

#ifdef ILLUMINATION_MODULE
      // If the illumination module is active, the LEDs should turn on just
      // before the camera is exposing.
      digitalWrite(ILLUMINATION_PIN, HIGH);
      // Here, a warm-up delay for the LEDs can be added (needs checking).
      // delayMicroseconds(10);
#endif
      // Trigger the actual pulse.
      Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

      // Save the current time to estimate the exposure time in the pin
      // interrupt.
      exposure_start_us_ = micros();

      // Increament the image number as the camera is triggered now.
      ++image_number_;

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
#ifdef ADD_TRIGGERS
      trigger(ADDITIONAL_TEST_PIN, TRIGGER_PULSE_US,
              Sensor::trigger_type::NON_INVERTED);
#endif

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

#ifdef ILLUMINATION_MODULE
    // Deactivate the LEDs as we are not sure yet whether we get an exposure
    // signal.
    digitalWrite(ILLUMINATION_PIN, LOW);
#endif
    // Trigger the actual pulse.
    Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

    Sensor::setTimestampNow();
    Sensor::newMeasurementIsAvailable();

    // Save the current time to estimate the exposure time in the pin
    // interrupt.
    exposure_start_us_ = micros();

    // Increament the image number as the camera is triggered now.
    image_number_++;

    // Set the timer to make sure that the camera is triggered in a periodic
    // mode.
    timer_.setCompare(compare_);
  }
  // Reset the timer.
  timer_.resetOverflow();
}


