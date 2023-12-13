# yaFoC: yet another Field-oriented Control for PMSM motors arduino library

[![Arduino CI](https://github.com/uLipe/yaFoC/workflows/Arduino_CI/badge.svg)](https://github.com/marketplace/actions/arduino_ci)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

yaFoC is another just for fun project to exercise a minimal good vector
control library for PMSM motors, its goals include but is not limited to: portable, simple, resource-constrained friendly, and more important Arduino friendly.

## Features:
* Platform independent;
* Closed-loop PID based speed and position control;
* Very simple API, construct, initialize and invoke the controller periodically;
* Single-precision Floating point implementation;

## Limitations:
* No Sensorless control support;
* Requires and rotor position sensor, for example, incremental encoder.

## Support:
If you find some trouble, open an issue, and if you are enjoying the project
give it a star. Also, you can try reaching me at ryukokki.felipe@gmail.com

## Acknowledgements:
This project has a close inspiration on simpleFoC, which is doing a great job to
desmistify the BLDC vector control: https://github.com/simplefoc/Arduino-FOC
