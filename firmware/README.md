# Robot control firmware


## Architecture

### HAL

A HAL allows for building either a microcontroller (`micro`) version or an on-workstation (`native`) binary for simulation / prototyping.

All interaction with motor drivers, encoders, timers, interrupts etc. are routed through the HAL to allow for the low-level communications to be replaced with ZeroMQ socket communication to simulated hardware (see `hal/native/comms.cpp`)

### State

Robot state storage and updating is handled in `src/state.cpp`. There are three globals:

* `intent`: the state we want the robot to be in
* `actual`: the state the robot is currently in
* `settings`: PID and other tunable parameters that affect the motion of the robot

See `src/state.h` for struct definitions

### Motion

Stepper control logic is located in `src/motion.cpp`. The most important methods are `motion::update()` and `motion::write()`. 

`motion::update()` is called periodically to recalculate the stepper motor timings via PID to approach the intent position and velocity.

`motion::write()` is called at a high frequency (via timer itnerrupt when on actual hardware) and actually writes to the stepper drivers.

### Customization

Hardware- and robot-specific config options are passed as preprocessor includes within `platformio.ini`, allowing for builds that control e.g. 3 vs 6 joint robots


