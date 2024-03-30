# Frontiers Navigation

Run the frontiers navigation node

```
ros2 launch wombat_strategy frontiers_navigation_launch.py
```

## Usage example

Run each command in a different terminal:

 - Start the kennel and nav2
 ```
  ros2 launch wombat_bringup nav2_kennel_launch.py
 ```

 - Start the frontiers navigation server
 ```
 ros2 launch wombat_strategy frontiers_navigation_launch.py
 ```

 - Request to navigate to all frontiers
 ```
 ros2 action send_goal /explore wombat_msgs/action/FrontierNavigation "{}"
 ```
