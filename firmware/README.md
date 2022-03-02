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



## Flashing STM32

```
docker-compose --env-file ar3.env -p shop_robotics run fw_native /bin/bash
pio run -e stm32
```

* Plug in STM32F4 discovery board via USB to computer.
* In [STM32CubeProg](https://www.st.com/en/development-tools/stm32cubeprog.html) connect to the ST-LINK 
* Click the `+`, then `open file`, and select `.../shop_robotics/firmware/.pio/build/stm32/firmware.bin`
* Click the "Download" button and wait for confirmation
* Press the black "Reset" pushbutton on the discovery board to restart the firmware.

**Indicators**

* Orange LED (LD3) - currently unused
* Green LED (LD4) - solid indicates successful startup (see hal/stm32/hw.cpp)
* Red LED (LD5) - solid indicates startup failure (see hal/stm32/hw.cpp)
* Blue LED (LD6) - flashing indicates receiving data from computer via UART 5

