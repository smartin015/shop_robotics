# L2 Workshop Robotics

Firmware and control software for workshop/makerspace robots.

Ensembles of programs are started via the `./launch.sh` script.

# Layout

https://docs.google.com/drawings/d/1Mtd4x5n-zbuuM7uRhCHO-HnwOfN_e4qv0sVR_wNz95c

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
docker buildx build --progress=plain --tag shop_robotics_sim .
```

## Running profiles

```shell
docker-compose --env-file tribot.env -p shop_robotics --profile <webots|gamepad|headless> up
```

**Webots**

This runs a webots simulation of the AR3 robotic arm, with a separate web interface (localhost:8000) and a simulated copy of the firmware which normally lives in the control board of the robot.

**Headless**

This runs a "stub" simulator (vastly simplified vs Webots) plus the firmware and web interface. Use this to prototype quickly on the web interface / firmware without having to restart Webots every time.

The `tribot_headless` version runs a 3-joint simulation, to match the omniwheel robot's number of wheels.

**Gamepad**

This runs an additional process that connects to a USB XBox 360 controller and allows for jogging the robot. It's currently hardcoded for 3 axes only,
and must be run along with another environment (e.g. `docker-compose --profile headless --profile gamepad up`)

# Extras

Possibly useful for getting MoveIt! control working: 

- (ROS) https://github.com/ongdexter/ar3_core
- (ROS2) https://github.com/RIF-Robotics/ar3_ros


# Lessons learned

* ZeroMQ: IPC can handle PAIR sockets, but:
  * If multiple users (i.e. firmware & sim containers, since firmware runs as a user and not root) the file must have the correct unix permissions (chmod 777 to be lazy)
  * Socket must remain open on both ends. When it closes, it can't be reopened. This is a property of PAIR sockets.
