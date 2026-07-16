# Embedded Odometry for Ground Robot

## Flash the STM32F3Discovery
What you need
STM32CubeIDE installed on  PC
The Project2scratch project (unzip Project2scratch.zip)

Instructions
Unzip Project2scratch.zip on your PC.
Build the project: Project → Build Project (or press Ctrl+B). Wait for zero errors.
Flash the board: Run → Debug (or press F11).
Embedded Odometry for a Ground Robot
1- IMU Dead Reckoning → Visual Inertial Odometry
2- STM32F3Discovery + Jetson Nano + Raspberry Pi Camera v2
---
Project Structure
```
odometry_project/
├── package1/ IMU Dead Reckoning
│   ├── dr_main.py              ← MAIN entry point (Package 1)
│   ├── imu_receiver.py         ← STM32 USB-CDC reader
│   ├── ahrs.py                 ← Mahony AHRS filter
│   ├── dead_reckoning.py       ← DR pipeline + ZUPT
│   └── visualizer.py           ← Real-time matplotlib visualiser
│
├── package2/                   ← Visual-Inertial Odometry
    ├── vio_main.py             ← MAIN entry point (Package 2)
    ├── visual_odometry.py      ← Standalone VO (camera alone)
    ├── generate_default_params.py  ← RPi Camera v2 for camera parameters as there is no checkerboard
    ├── feature_tracker.py      ← KLT optical flow
    ├── imu_preintegration.py   ← IMU pre-integration (100Hz to camera 30HZ)
    ├── mono_vo.py              ← Essential matrix + RANSAC
    ├── imu_receiver.py         ← STM32 USB-CDC reader 
    └── vio_fusion.py           ← Loosely-coupled IMU+VO fusion


```
---
Quick Start
Step 1 – Install dependencies
pip install numpy opencv-python matplotlib pyserial

Step 2 – Generate camera parameters (no checkerboard needed)
cd package2
python generate_default_params.py

Step 3a – Run Package 1 (IMU Dead Reckoning)
cd package1
python dr_main.py

Step 3b – Run Standalone VO (camera-only)
cd package2
python visual_odometry.py
Step 3c – Run Package 2 (VIO – fused)
cd package2
python vio_main.py

STM32 USB Output Format
The STM32F3Discovery firmware (Project2scratch) streams at 100 Hz:
```
<tick_ms>,<ax_g>,<ay_g>,<az_g>,<gx_rads>,<gy_rads>,<gz_rads>,Stationary\r\n
```
Accelerometer: g-units (converted to m/s² in `imu_receiver.py`)
Gyroscope: rad/s (bias subtracted in firmware)
