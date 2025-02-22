#ifndef every_sync_configuration_h
#define every_sync_configuration_h

/* ----- General configuration -----*/
// Activate USB serial interface for ARM processor types.
#define USE_USBCON

// Specify the CPU frequency of the controller.
#define CPU_FREQ_HZ 48e6

// Specify the trigger pulse width;
#define TRIGGER_PULSE_US 50

#define TRIGGER_PULSE_PPS 100

/* ----- Camera configuration ----*/
// // Camera 0
// // #define CAM0_TOPIC "/versavis/xsens_imu/"
// #define CAM0_TOPIC "/realvis/xsens_imu/"
// // #define CAM0_TOPIC ""
// #define CAM0_RATE 100
// #define CAM0_TYPE trigger_type::NON_INVERTED
// #define CAM0_TRIGGER_PIN 14
// #define CAM0_EXPOSURE_PIN 5

// Camera 1
// #define CAM1_TOPIC "/versavis/cam1/"
#define CAM1_TOPIC "/realvis/cam1/"
// #define CAM1_TOPIC ""
#define CAM1_RATE 20
#define CAM1_TYPE trigger_type::NON_INVERTED
// #define CAM1_TRIGGER_PIN 15
// #define CAM1_EXPOSURE_PIN 6
#define CAM1_TRIGGER_PIN 14
#define CAM1_EXPOSURE_PIN 5

// Camera 2
// #define CAM2_TOPIC "/realvis/cam2/"
#define CAM2_TOPIC ""
#define CAM2_RATE 20
#define CAM2_TYPE trigger_type::NON_INVERTED
#define CAM2_TRIGGER_PIN 16
#define CAM2_EXPOSURE_PIN 7

/* ----- Triggeralbe IMU -----*/
// Xsens MTi-670-DK 
#define TRIG_IMU_TOPIC "/realvis/imu/"
// #define TRIG_IMU_TOPIC ""
#define TRIG_IMU_RATE 100
#define TRIG_IMU_TYPE trigger_type::NON_INVERTED
// #define TRIG_IMU_TRIGGER_PIN 14
// #define TRIG_IMU_EXPOSURE_PIN 5
#define TRIG_IMU_TRIGGER_PIN 15
#define TRIG_IMU_EXPOSURE_PIN 6

/* ----- LIDAR -----*/
// Possible values: USE_ADIS16445, USE_ADIS16448AMLZ, USE_ADIS16448BMLZ,
// USE_ADIS16460 and USE_VN100
#define LIDAR_TOPIC "/realvis/lidar/"
// #define LIDAR_TOPIC ""
#define LIDAR_RATE 100
#define LIDAR_TYPE trigger_type::NON_INVERTED
#define LIDAR_TRIGGER_PIN 3
// #define LIDAR_TRIGGER_PIN 16
#define LIDAR_EXPOSURE_PIN 17

/* ----- EXTERNAL CLK <-- GNSS MODULE -----*/
#define EXTCLK_TOPIC "/realvis/gnss/"
// #define EXTCLK_TOPIC ""
#define EXTCLK_RATE 1
#define EXTCLK_TYPE trigger_type::NON_INVERTED
// #define EXTCLK_PPS_PIN 3  //4 NOT USED FOR EXT_INTERRUPT
#define EXTCLK_PPS_PIN 7  //4 NOT USED FOR EXT_INTERRUPT
#define EXTCLK_NMEA_PIN 4 //NC


/* ----- IMU -----*/
// Possible values: USE_ADIS16445, USE_ADIS16448AMLZ, USE_ADIS16448BMLZ,USE_BMI_088, USE_ICM_42688
// USE_ADIS16460 and USE_VN100
#define NONE_IMU
#define IMU_TOPIC "/realvis/imu_micro"
#define IMU_RATE 100

/* ----- Additional triggers ----- */
// Define whether additional test outputs should be used.
// #define ADD_TRIGGERS
#ifdef ADD_TRIGGERS
#define ADDITIONAL_TEST_PIN 2
#endif

/* ----- Illumination module ----- */
// Activation of the illumination module.
// #define ILLUMINATION_MODULE
#ifdef ILLUMINATION_MODULE
#define ILLUMINATION_PWM_PIN 2
#define ILLUMINATION_PIN 26
#endif

/* ----- Debug mode. ----- */
// Define whether debug mode should be used. This provides output on the
// standard console but invalidates ROS communication.
// #define DEBUG

#endif // versavis_configuration_h
