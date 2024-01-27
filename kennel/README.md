# kennel

The `kennel` is a kinematics simulator meant to exercise high-level robotics behaviors.
Its purpose is to be lightweight and easy to integrate as part of an automated testing routine.

To run the `kennel` as a stand-alone application:

```bash
ros2 launch kennel kennel_launch.py
```

Note: this will also start the Foxglove Bridge

### Useful commands

Publish velocity commands

```
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
