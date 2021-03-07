# yaFoC: yet another Field-oriented Control for PMSM motors arduino library

![Build](https://github.com/uLipe/yaFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

yaFoC is another just for fun project to exercise a minimal good vector 
control library for PMSM motors, its goals include but is not limited to: portable, simple, resource-constrained friendly, and more important Arduino friendly.

## Features:
* Platform independent;
* Closed-loop PID based currrent control;
* Very simple API, construct, initialize and invoke the controller periodically;
* Single-precision Floating point implementation;

## Limitations:
* No Sensorless control support;
* It needs phase-current sensor of each motor windings;
* Requires and rotor position sensor, for example, incremental encoder.

## Support:
If you find some trouble, open an issue, and if you are enjoying the project
give it a star. Also, you can try reaching me at ryukokki.felipe@gmail.com

## Acknoledgements:
This project has a close inspiration on simpleFoC, which is doing a great job to 
desmistify the BLDC vector control: https://github.com/simplefoc/Arduino-FOC 
