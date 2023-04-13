# kinova_rospy
A python API library for controlling the kinova robotic arm

## install

### install kinova-ros

> https://github.com/Kinovarobotics/kinova-ros

### install py library

copy `kinova_control.py` to your ROS workspace



## usage

refer to `demo.py`

first, use below code init kinova.

```python
kinova_robotTypeParser()
getcurrentCartesianCommand()
```

> notice, `kinova_robotTypeParser()` need to input arm type, I set a default param with my robot `j2s7s300`.

second, use function `move_arm()`control arm, use function  `move_finger()` control gripper.

The params of these function are same to kinova offical cmd, as follows:

```shell
rosrun kinova_demo pose_action_client.py -v -r j2s7s300 mdeg -- 0.01 0 0 0 0 10
rosrun kinova_demo fingers_action_client.py j2s7s300 percent -- 100 100 100
```
