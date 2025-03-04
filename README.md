# Worm Charger (Work in progress)

## Synopsis

The DART is a programmable four-wheel robot, equipped with multiple sensors, used by the second year robotics students of ENSTA Bretagne.

![Dart robot](https://github.com/vivipal/worm-charger/blob/main/figs/dart.jpg)

The DART can be controlled remotely via the internet but had to be constantly wired in for it to charge. It was not very convenient as the cables could interfere with the robot's movements.
The goal of this project was to solve this problem. Thanks to our worm charger, the DART can now run on batteries, and when it needs to recharge, it will drive itself to the Worm Charger.
The Worm Charger will then automatically plug it in to charge.

We used a modified WidowXL Robot Arm: the end part of the arm was replaced with a 3D printed piece to mount the charging plug, a camera and an end switch.


The DART was slightly modified with a top charging port, making it easier for the arm to plug. A circle was placed around the port to allow the camera to recognize it and guide it.
</br></br>
<img src="https://github.com/vivipal/worm-charger/blob/main/figs/arm.jpg" width="33%"/>
<img src="https://github.com/vivipal/worm-charger/blob/main/figs/head_arm.jpg" width="33%"/>
<img src="https://github.com/vivipal/worm-charger/blob/main/figs/dart_top.jpg" width="33%"/>


## Charging manoeuver

The charging manoeuver splits into several steps:

  * the arm is placed in a standard pose, camera facing down, on top of the DART.

  * using OpenCV, the camera is able to recognize the circle and the computer controls the motors in order to align the plug with the charging port.
  
  <img style="display: block;margin-left: auto;margin-right: auto;" src="https://github.com/vivipal/worm-charger/blob/main/figs/robot_view.png" width="300"/>

  * using inverted kinematic (roboticstoolbox), the computer generates a trajectory for the motors to go down vertically and plug it int. The end switch is there to confirm that contact had been made and charging has began.

  * Once charging is done, the robot arm will unplug itsef from the DART robot and go back to its standard pose.
  
  -----
  
  Here is the first two steps :
  
  <img src="https://github.com/vivipal/worm-charger/blob/main/figs/robot_in_movement.gif"/> 
