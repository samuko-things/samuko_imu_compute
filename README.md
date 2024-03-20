# Samuko IMU Compute (`sic`) Project
The parent repo of the Samuko IMU Compute (**`sic`**) Project.
<br/>
<br/>
A common problem faced in robotics (or embedded developments) is setting up and processing the Inertia Measurement Unit (IMU) computaions. 
<br/>
Setup processes like magnetometer, gyroscope and accelerometer calibration are usually an issue and can be frustrating as wrong calibration will result to faulty readings. Also, one would usually like to get a less noisy orientation (roll, pitch and yaw) readings i.e the noice should be well filtered. 
<br/>
<br/>
The AIM of this project is to design and make available a IMU setup and computation system `(i.e a physical module, software and libraries)` that will help to easily calibrate IMU sensors (i.e magnetometer, gyroscope and accelerometer) and output a well filetered orientation readings. It uses the `kalman filter` algorithm for its filteration process, `elipsoid fitting` algorithm for its magnometer calibration, `average` algorithm for accelerometer calibration.

>This IMU compute system will allow the user to:
> - easily connect/interface an IMU (`MPU9250 module`).
> - easily calibrate the IMU (`MPU9250 module`).
> - read/get filtered orientation readings (as well as other readings - rate and acceleration with covariances) from the IMU(`MPU9250 module`).
> - easily integrate it into a microcontroller-based (Arduino) project.
> - easily integrate it into a microcomputer-based (Raspberry Pi, etc.) project.
> - easily integrate it into a ROS2-based project.

<br/>
The `sic` project consist of the following sub-parts:

- **`sic_mpu9250_driver module`**: this a the module to which the `MPU9250 IMU module` is interfaced/connected to via header pins. here's the link to the repo of the driver code -> [sic_mpu9250_driver_code](https://github.com/samuko-things-company/sic_mpu9250_driver_code). The module provides a USB serial communication interface using the FTDI programmer to connect with a PC or micro-computer to setup, calibrate, visualize, and use in projects. It also provides an I2C communication interface for microcontroller-based (Arduino) projects.

![sic_img](./docs/sic_img2.jpeg)

- **`sic_calibration_py_codes`**: set of step by step codes to help calibrate, compute necessary covariances, and visualize the filtered readings of the IMU (`MPU9250 module`) connected to the **sic_mpu9250_driver module**. Here's the link to the repo -> [sic_calibration_py_codes](https://github.com/samuko-things-company/sic_calibration_py_codes)
- **`sic_pyserial_lib`**: library that helps communicate with the already setup IMU (`MPU9250 module`) in you PC or microcomputer-based python projects. here's a link to the repo -> [sic_pyserial_lib](https://github.com/samuko-things-company/sic_pyserial_lib) 
- **`sic_ros2_interface`**: ros2 package to help communicate with the already setup IMU (`MPU9250 module`) in your ROS2-based projects **(currently implemented in ROS2-humble LTS)**. here's a link to the repo -> [sic_ros2_interface](https://github.com/samuko-things-company/sic_ros2_interface) 
- **`sic_i2c_lib`**: arduino library that helps communicate with the already setup IMU (`MPU9250 module`) via I2C ADDRESS - 0x68, in your arduino-based project (e.g Arduino nano, UNO, MEGA, ESP32, e.t.c). here's the link to the repo -> [sic_i2c_lib](https://github.com/samuko-things-company/sic_i2c_lib) 
<br/>
<br/>

![sic sub part](./docs/sic_sub_parts.drawio.png)

## HOW TO USE THE `sic` IN YOUR PROJECT
- First of all get the **`MPU9250 IMU module`** and **`sic_mpu9250_driver module`** (it will come preloaded with the driver code and also with an FTDI serial programmer for USB serial communication).

- Interface/connect the **`MPU9250 IMU module`** to the **`sic_mpu9250_driver module`** via the header pin.

- clone or download the **`sic_calibration_py_codes`** repo into your PC. 

- Connect the **`sic_mpu9250_driver module`** to your PC via the FTDI and follow the **`sic_calibration_py_codes`** step by step to setup the calibration and cavariance parameters, and visualize the IMU data.
  > **NOTE:** the parameter values you set are automatically saved to the microcontroller's memory (i.e it remembers the parameter values)

- After successfully setup, disconnect the driver module from the PC and hit the reset button on the `sic_mpu9250_driver module`.

- use it in your prefered project using any of the API library - **`sic_pyserial_lib`**, **`sic_ros2_interface`**, or **`sic_i2c_lib`**.


## RESOURCES USED AND THEIR REFERENCES

|S/N|AUTHOR(S)|INFO/LINKS|WHERE/HOW IT WAS USED|
|--|--|--|:--:|
|01|Tesla Labs|[Magnetometer Calibration](https://teslabs.com/articles/magnetometer-calibration/)|I adapted the ellipsoid fit algorithm for the sic magnetometer calibration in the **`sic_calibration_py_codes`**.|
|02|TKJ Electronics</br>-> Sept 10, 2012|[Kalman FIlter For IMU](https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/)|IMU Kalman filter algorithm in the sic driver code **`sic_mpu9250_driver_code`**.|
|03|Brian Douglas</br>(Matlab Tech Talks)|[Understanding IMU Sensor Fusion](https://www.youtube.com/watch?v=0rlvvYgmTvI&t=317s)|This work formed a base structure used in the **`sic`** sensor fusion steps.|
|04|Carbon Aeronautics|[Kalman FIlter With Gyro and Acc](https://www.youtube.com/watch?v=5HuN9iL-zxU)|sic driver code  IMU Kalman filter|
|05|Bolder Flight</br>(MPU9250 Library)|[MPU9250 library - Github](https://github.com/bolderflight/invensense-imu)|The library was used to communicate with the MPU9250 interfaced with the sic driver module to get raw Acc, Gyro, and Mag readings.|
|06|Lady Ada</br>(Adafruit Sensor Lab)|[Magnetometer Calibration Visualization](https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-jupyter)</br>[Gyro Calibration Visualization](https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration/gyro-calibration-with-jupyter)|Magnetometer Calibration  Data Visualization</br>I adapted the gyro calibration algorithm and visualization for the sic calibration process.|
|07|Paul McWhorter</br>(9axis IMU LESSONS)|[IMU Viz Pitch and Yaw](https://www.youtube.com/watch?v=iQ2zX4_Ftv8&t=247s)</br>[IMU Viz Roll, Pitch and Yaw](https://www.youtube.com/watch?v=7B3KnAj7xzY)</br>[IMU Viz RPY Quaternion](https://www.youtube.com/watch?v=S77r-P6YxAU&t=2004s)|sic IMU visualization Python code|
|08|Flynneva</br>(BNO055 ros2 pkg)|[BNO055 ros2 - Github](https://github.com/flynneva/bno055/tree/main)|I adapted it in developing the sic ros2 interface package.|
|09|Automatic Addison</br>(Eular - Quaternion Conversion)|[Quat To Euler Conversion](https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/)|Adapted it in Arduino code as well as ROS2 interface code to perform conversion between eular and quaternions.|
|10|Brandon McCarron</br>June 2013|[Low-Cost IMU Implementation via Sensor Fusion Algorithms in the Arduino Environment - Journal](https://core.ac.uk/download/18293757.pdf)|IMU Kalman filter algorithm in the sic driver code.|
|11|Obiagba Samuel|[Serial Communication between Arduino and PC - Github](https://github.com/samuko-things/serial_comm_pyserial_and_arduino)|Formed the basis of the communication algorithm used in the **`sic_pyserial_lib`** and the driver code|