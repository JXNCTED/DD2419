## Completed

- a port of cartesian controller from workshop (radius, base etc. may not be correct)

the tick per rev is 3600 for this encoder

- a node just for debug

`ros2 run chassis_controller joy_controller`

Use joystick to move and grab, right joystick for movement and B for grab at the moment


## phidget

```bash
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
sudo apt install phidget22admin
```

to update the firmware
(may not work for the first time, power cycle everything then seems fine (or loose connector))
```bash
phidget22admin -du # list the serial and port
phidget22admin -M 123456/2 -U # update the one with the port number
phidget22admin -M 123456 -U
```

To launch
```bash
ros2 launch robp_phidgets_motors motors_launch.py
ros2 launch robp_phidgets_encoders encoders_launch.py
ros2 launch robp_phidgets_spatial spatial_launch.py
ros2 launch robp_phidgets_temperature temperature_launch.py
```

## Joy 

```bash
apt-get install ros-humble-teleop-twist-joy
```

launch the joy

```bash
ros2 launch teleop_twist_joy teleop-launch.py 
```

## USB-cam (not tested)

```bash
sudo apt-get install ros-humble-usb-cam
```

## arm

- Recompiled the firmware with the new domain id

under 

/home/group7/dd2419_ws/src/robp_robot/robp_arm/Hiwonder_xArm_ROS2/

run `pio lib install`

change the line inside `setup_entities` about the node setup to

```cpp
	allocator = rcl_get_default_allocator();			// Initialize micro-ROS allocator
	rclc_support_t support;
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	size_t DOMAIN_ID = 7;
	RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	RCCHECK(rclc_node_init_default(&node_hiwonder, "Hiwonder_xArm_node", "", &support)); // create node
```

run

```
pio run 
pio --target upload
```

In order to upload, set the `upload_port = /dev/ttyUSB2` in the `platformio.ini` to ensure the download device is correct

apart from cloning the repo, also need to 

- setup the micor-ros agent 

https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/Hiwonder_xArm_ROS2/SETUP_README.md#ros2--micro-ros

launch the communication with 

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

list the topic to makesure the following topics are found

```bash
/multi_servo_cmd_sub
/parameter_events
/rosout
/servo_pos_publisher
/servo_temp_publisher
/servo_volt_publisher
```

- Control the arm by publishing topics to `/multi_servo_cmd_sub`

detailed in this README

https://github.com/migsdigs/Hiwonder_xArm_ESP32/tree/main?tab=readme-ov-file#hiwonder-xarm-esp32-control-with-ros2

Basically, the data in the `/multi_servo_cmd_sub` of type `std_msgs/Int16MultiArray` has 12 values, the first 6 is the angle of servo in centi-degrees
and the following 6 data are the time to get the target position in miliseconds.

Problem:Super laggy for some reason, takes ~1s to execute the command

## Realsense SDK

followed this guide for the SDK

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packagess

Used this for the ROS wrapper

```bash
sudo apt install ros-humble-realsense2-*
```

launch the node with 
```bash
ros2 launch realsense2_camera rs_launch.py
```

## RP Lidar

Installed a different person's port of the driver

https://github.com/babakhani/rplidar_ros2

Need to change the USB devie in the launch files. the `/ttyUSB1` is for the Lidar. Especially is saw weired behaviors. i.e. launch arm node and the lidar stop spinning and vice versa.

See a demo rviz

```bash
ros2 launch rplidar_ros view_rplidar.launch.py
```

## side note

/ttyUSB0 is the arm
/ttyUSB1 is the lidar