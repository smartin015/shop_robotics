This HAL uses https://github.com/ussserrr/stm32pio, an extra library for generating STM32Cube files in a
PlatformIO compatible way. 

Whenever you make changes to `ShopRobotics.ioc`, run 
```
# To get current project state
stm32pio status

# To check for diffs
stm32pio validate

# To resolve diffs
stm32pio generate
```

Setup commands used (no need to re-run):

```
pip3 install stm32pio
stm32pio init
stm32pio new -b genericSTM32F407VGT6
```

