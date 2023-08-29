# Closed-loop model checking for tactical planning in C++

<img src="https://github.com/possibilia/mcplanner/blob/main/robot.jpg" width="550" height="500">

## Hardware 

- AlphaBot mobile robot development kit by Waveshare https://www.waveshare.com/alphabot-robot.htm. 
- 360 degree 2D laser scanner, RPLIDAR A1MB by Slamtec https://www.slamtec.com/en/Lidar/A1/
- Continuous rotation servos by Parallax https://www.parallax.com/product/parallax-continuous-rotation-servo/
- Raspberry Pi 3 Model B https://www.raspberrypi.com/products/raspberry-pi-3-model-b/

## Software

### Prerequisites 

Install the following SDKs:

- AlphaBot SDK https://github.com/berndporr/alphabot
- RPLIDAR A1M8 SDK https://github.com/berndporr/rplidar_rpi

### Compilation 

```mkdir build```

```cd build```

```cmake ..```

```make```

This generates build files in a dedicated directory and compiles the program. 

### Run program

From the build directory type:

```sudo ./autoctrl```



  
