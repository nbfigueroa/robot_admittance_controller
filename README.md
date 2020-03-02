# arm_admittance_controller

Admittance control law to generate desired motion of an end-effector (twist), given a desired control and external wrench for a robotic arm that is not torque-controlled (i.e. velocity or position controlled). 

* Such a controller is necessary to use impedance-control laws and provide compliant behavior when the  velocity/position controlled robot arm is equipped with an external force/torque sensor.

* The current implementation is tested on a UR10 velocity-controlled robot with a robotiq FT 300 force torque sensor. The arm is part of the Robbie Yuri robot of the Interactive Robotics Group (IRG), MIT which is an older version of the [Care-O-Bot](http://www.care-o-bot.org) platform series.

**Disclaimer**: This code was originally forked from [ridgeback_ur5_controller](https://github.com/epfl-lasa/ridgeback_ur5_controller) which is a repo used to control a Clearpath robot with a UR5 from the LASA laboratory at EPFL.

**This branch includes the original code of the admittance controller fro arm+platform robotic system described in this [README](https://github.com/nbfigueroa/ridgeback_ur5_controller/blob/devel/README.md) file. It is not funcitional as is.**


## Installation

## Usage


**Maintainer**: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig AT mit dot edu)
