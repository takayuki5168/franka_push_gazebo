Franka Emika Panda Pushing Object in Gazebo
===========================================

![sample1](https://github.com/takayuki5168/franka_push_gazebo/blob/master/figs/panda.png)

## How to Use
- Launch panda in Gazebo with MoveIt
```sh
$ roslaunch franka_push_gazebo bringup.launch
```
- Launch python interface of Gazebo
```sh
$ python scripts/gazebo_interface.py
In [1]: gi.init_robot_pose()
In [2]: gi.set_cartesian_pose([0.5, 0, 0.12])
```

## Trouble Shooting
### Jerky joints
- Tune PID gains of joint angles controllers, then save in `franka_gazebo/config/pid.yaml`
```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
