# kennel

The `kennel` is a kinematics simulator meant to exercise high-level robotics behaviors.
Its purpose is to be lightweight and easy to integrate as part of an automated testing routine.

## How to use

To run the `kennel` as a stand-alone application:

```bash
ros2 launch kennel kennel_launch.py
```

Then you can control your robot, e.g. via a separate application or by directly publishing commands
```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Look at the `wombat_bringup` package for more comprehensive launch scripts.

## Directory structure

The package contains the following directories, in alphabetical order.

 - `bringup`: contains launch files, runtime configuration and other files used to bring up the `kennel` application.
 - `common`: contains the `kennel::common` library.
 This is where plugins base classes and the reusable building blocks for the `kennel` application are implemented.
 - `kennel`: the user-facing library and application for the `kennel`.
 - `kennel_gtest`: library meant to facilitate the use of Kennel in C++ unit-tests.
 - `plugins`: dynamically configurable plugin files used by the `kennel` application
