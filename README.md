# EverySync --  An Open Hardware Time Syncronization Sensor Suite
**EverySync** is An Open Hardware Time Syncronization Sensor Suite For Every Casual Sensor In Field Of SLAM.
- Our project provides a complete, open-source hardware, firmware and software bundle to perform `Hardware Time Synchronization ` of multiple sensors in SLAM system.
  
<div align=center>
<img src="pic/System_pipline.png"  width="600" align="center">
</div>

Entry of This Project:

* [EverySync -- HardwareSuite](https://github.com/NEU-REAL/EverySync-Hardware-Suite) Open source Hardware Suite of EverySync.
* [EverySync -- PCB and Examples](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/EverySync_PCB) Released. Both Altium Designer and LCEDA versions are provided, make sure the project is easy to reproduce. And we also provide some examples for developers to create their own hardware suites.

<div align=center>
<img src="pic/Examples.png"  width="600" align="center">
</div>


## News
- **May 26, 2022** Finish first version named RealVIS v1.0 .
- **May  8, 2023** Support at least 3 kinds of Lidar.
- **April  2, 2024** Open-source. Submit to IROS2024. Preprint is comming soon.
- **June  30, 2024** Accepted by IROS2024. 
- **December 26, 2024** Proceding of IROS2024 is released.
- **January  13, 2025** Project is totally open source. ⭐Happy New Year⭐. 

## Supported drivers
### Camera 
* [Basler](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/driver/pylon-ros-camera) tested with acA1920-155uc
* [HikVision](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/driver/HIKROBOT-MVS-CAMERA-ROS) tested with HikVision MV-CA023-10UC
#### Camera Support From VersaVIS
* [MatrixVision](https://github.com/ethz-asl/bluefox2/tree/devel/versavis) tested with Bluefox 2 MLC200WG, needs adaption for new format
* [PointGrey/Flir](https://github.com/ethz-asl/flir_camera_driver/tree/devel/versavis) tested with Chameleon 3, Blackfly S
* [CamBoard](https://github.com/ethz-asl/pico_flexx_driver/tree/devel/versavis) tested with CamBoard pico monstar

### TriggeralbeIMU
* [Xsens](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/driver/xsens_ros) Support Xsens MTi-100 Series & Xsens MTi-600 Series.Tested with Xsens MTi-300,MTi-630,MTi-670-DK.

### IMU Support From VersaVIS
* [ADIS](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/every_sync/every_sync/src) Originally Support ADIS16448 from VersaVIS

### Lidar
* [Ouster](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/driver/pylon-ros-camera) tested with Ouster-OS1-32
* [Robosense](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/driver/HIKROBOT-MVS-CAMERA-ROS) tested with Robosense16
* [Livox](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/driver/HIKROBOT-MVS-CAMERA-ROS) tested with Livox Mid-40 , Mid-70 , Mid-360

### GNSS/GPS
* [Ublox](https://github.com/NEU-REAL/EverySync-Hardware-Suite/tree/master/src/driver/ublox_driver) tested with Ublox ZED-F9P

## Citing

```
@inproceedings{wu2024everysync,
  title={EverySync: An Open Hardware Time Synchronization Sensor Suite for Common Sensors in SLAM},
  author={Wu, Xuankang and Sun, Haoxiang and Wu, Rongguang and Fang, Zheng},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={12587--12593},
  year={2024},
  organization={IEEE}
}
```

## Install

### Clone and build
```
cd catkin_ws/src
git clone https://github.com/NEU-REAL/EverySync-Hardware-Suite.git
catkin_make
```

```
cd everysync/firmware
./setup.sh
```

### Setup udev rule
Add yourself to `dialout` group
```
sudo adduser <username> dialout
```

Copy udev rule file to your system:
```
sudo cp firmware/98-versa-vis.rules /etc/udev/rules.d/98-versa-vis.rules
```
Afterwards, use the following commands to reload the rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
```
Note: You might have to reboot your computer for this to take effect. You can check by see whether a `/dev/versavis` is available and pointing to the correct device.

### Configure
Adapt the [configuration file](https://github.com/ethz-asl/versavis/blob/master/firmware/libraries/versavis/src/versavis_configuration.h) to your setup needs. Also check the [datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776) for how to configure the hardware switches.

### Flash firmware on the VersaVIS board
* Install the arduino IDE from [here](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous). Use version 1.8.2!
    - Note that a small modification of the install script (`install.sh`) might be required. In particular you may need to change the line `RESOURCE_NAME=cc.arduino.arduinoide` to `RESOURCE_NAME=arduino-arduinoide` as per the issue [here](https://github.com/arduino/Arduino/issues/6116#issuecomment-290012812).
* Open `firmware/everysync/everysync.ino` in the IDE
* Go to `File -> Preferences`
* Change Sketchbook location to `versavis/firmware/`
* Install board support:
    - For EverySync: [Check here](https://github.com/ethz-asl/versavis_hw/)
* Set `Tools -> Port -> tty/ACM0 (Arduino Zero)`, and `Tools -> Board -> VersaVIS`.
* Compile using the *Verify* menu option
* Flash using the *Upload* menu option


## Usage
* Adapt `everysync/launch/pps_test.launch` to your needs.
* Run with
```
roslaunch everysync pps_test.launch
```
* Wait for successfull initialization.
* Or initialization by publish rostopic.
```
./init_cam.sh
```


## Acknowledgements
**There are several important works which support this project:**
- [VersaVIS](https://github.com/ethz-asl/versavis): An Open Versatile Multi-Camera Visual-Inertial Sensor Suite
- [Versavis-Hw](https://github.com/ethz-asl/versavis_hw): VersaVIS Board package for arduino IDE.


## Licence
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

## Maintenance
We are still working on extending the proposed system and improving code reliability.

