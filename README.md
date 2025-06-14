# Model checking for obstacle avoidance

Implementation of model checking for planning obstacle avoidance maneuvers by chaining temporary control tasks in response to disturbances.  At present the program is limited to static environments is not goal-directed.

## Hardware 

<img src="https://github.com/possibilia/mcplanner/blob/main/robot.jpg" width="550" height="500">

- AlphaBot mobile robot development kit by Waveshare https://www.waveshare.com/alphabot-robot.htm. 
- 360 degree 2D laser scanner, RPLIDAR A1MB by Slamtec https://www.slamtec.com/en/Lidar/A1/
- Continuous rotation servos by Parallax https://www.parallax.com/product/parallax-continuous-rotation-servo/
- Raspberry Pi 3 Model B https://www.raspberrypi.com/products/raspberry-pi-3-model-b/

## Software

<img src="https://github.com/possibilia/mcplanner/blob/main/agent.jpg" width="550" height="500">

### Prerequisites 

Install the following SDKs:

- Servos - AlphaBot SDK https://github.com/berndporr/alphabot
- Lidar - RPLIDAR A1M8 SDK https://github.com/berndporr/rplidar_rpi

### Compile 

Generate the build files

```cmake .```

Compile the program

```make```

### Run

```sudo ./autoctrl```

<!-- [I'm an inline-style link](https://youtu.be/FpOAJW28D9s) -->

[![DOI](https://zenodo.org/badge/457007482.svg)](https://zenodo.org/badge/latestdoi/457007482)

  
