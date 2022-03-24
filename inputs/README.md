Non-docker gamepad control example

```
# On robot
python3 broker.py --dest /dev/ttyACM0 -j 3

# On control station 
python3 gamepad.py --dest ws://192.168.1.250:8001
```

Can view broker display at `<ip>:8000`


## Ruckig motion planning

Build Ruckig with python bindings and some internal members exposed (the latter allows for accessing e.g. `trajectory.profiles`)

```
git submodule init --recursive
pip install "pybind11[global]"
cd control/ruckig 
mkdir -p build && cd build 
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_MODULE=1 -DEXPOSE_INTERNAL ..
make
```



