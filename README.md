# arm_admittance_controller

Admittance control law to generate desired motion of an end-effector (twist), given a desired control wrench and external wrench for a robotic arm that is not torque-controlled (i.e. velocity or position controlled). 

Such a controller is necessary to use impedance-control-like laws and provide compliant human-robot-interaction when the  velocity/position controlled robot arm is equipped with an external force/torque sensor.

* The current implementation has been tested on a UR10 **velocity-controlled robot** equipped with a robotiq FT 300 force torque sensor. The arm is part of the Robbie Yuri robot of the Interactive Robotics Group (IRG), MIT which is an older version of the [Care-O-Bot](http://www.care-o-bot.org) platform series.

* The current implementation is being tested on a Mitsubishi **position-controlled robot** equipped with a Mitsubishi force torque sensor. The robot is part of the MIT-Mitsubishi collaboration.

**Disclaimer**: Parts of this code was originally forked from [ridgeback_ur5_controller](https://github.com/epfl-lasa/ridgeback_ur5_controller) which is a repo used to control a Clearpath robotic platform with a UR5 from the LASA laboratory at EPFL. Plenty modifications have been made in order to work with a standalone robotic arm.

## Installation
...

## Usage
...

## Contact
**Maintainer**: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig AT mit dot edu)

## License
This package is licensed under MIT license.
