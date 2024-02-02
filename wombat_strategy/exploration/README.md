# Frontier Exploration


Run the frontier exploration node

```
ros2 launch wombat_strategy exploration_server_launch.py
```

### How to send a goal from command line

If you want to start the exploration from your laptop, you will have to build the `wombat_msgs` package.
Then you can explore the environment using the following call

```
ros2 action send_goal /explore wombat_msgs/action/Explore "{}"
```

If you want to only explore a certain amount of frontiers, you can specify it as a goal parameter

```
ros2 action send_goal /explore wombat_msgs/action/Explore "{num_frontiers: 2}"
```
