Franka Emika Panda Pushing Object in Gazebo
===========================================

## Launch
```sh
$ roslaunch franka_push_gazebo bringup.launch
```

## Tune PID Gain of Joint Angle Controllers
```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
- save parameters in `franka_gazebo/config/pid.yaml`

## Test Moving
```sh
$ python scripts/ik_server.py
```

## Gazebo Interface by Python
```sh
$ python scripts/gazebo_interface.py
```