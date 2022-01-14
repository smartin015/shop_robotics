# L2 Workshop Robotics

Firmware and control software for workshop/makerspace robots.

Ensembles of programs are started via the `./launch.sh` script.

# Layout

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

# Development

To setup a dev environment, you will need:

* [Docker](https://docs.docker.com/engine/install/ubuntu/)
  * IMPORTANT: [BuildKit](https://docs.docker.com/develop/develop-images/build_enhancements/) must be enabled for simulation builds
* [docker-compose](https://docs.docker.com/compose/install/)
* [This repository](https://github.com/smartin015/shop_robotics)

## Building containers

For the simulation container, you will need to enable BuildKit for docker (see above).

```shell
docker-compose build
```

For manual/test image builds of the sim container, with more logs:
```
docker buildx build --progress=plain --tag sim .
```

## Running profiles

```shell
docker-compose -p shop_robotics --profile <webots|gamepad|headless|tribot_headless> up
```
