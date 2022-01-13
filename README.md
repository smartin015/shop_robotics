# L2 Workshop Robotics

Firmware and control software for workshop/makerspace robots.

Ensembles of programs are started via the `./launch.sh` script.

## Docs

Contains various docs / presentation elements

## Sim

Contains configuration and setup for physical simulation of hardware (typically using Webots).

This is sometimes paired up to real firmware/control software for integration testing.

## Hardware

Contains KiCAD, FreeCAD, SVG and other files for making the physical bits of robots.

## Firmware

Contains code that goes on the robot itself - firmware uses https://platformio.org/ as an environment to allow
for cross-platform builds and native emulation.

## Control

Contains higher-level control software that receives input from the user/environment and sends commands to the firmware.

## Display

Contains programs for displaying the state of the robot and what it sees - on a computer or on embedded screens

