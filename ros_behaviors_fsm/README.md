# ROS behaviors FSM

[Project writeup](https://docs.google.com/document/d/1coxiCGMOocB-7vDXaP9QN-EjGjESF1iph1r0ITvcFLs/edit?usp=sharing)

One ROS 2 package containing the FSM, standalone behaviors, bar world, and recordings.

Build from your ROS workspace, with the Neato and Gazebo dependencies installed:

```bash
colcon build --symlink-install --packages-select ros_behaviors_fsm
source install/setup.bash
```

Launch the world in one terminal:

```bash
ros2 launch ros_behaviors_fsm bar_world.py
```

Run one controller at a time in another sourced terminal:

```bash
ros2 run ros_behaviors_fsm fsm_controller
```

The standalone alternatives are:

```bash
ros2 run ros_behaviors_fsm wall_follow
ros2 run ros_behaviors_fsm bump_protocol
ros2 run ros_behaviors_fsm drive_spiral
```

The standalone wall-follow behavior is the version recorded in `broken_wall_follow`; moving it here does not fix its navigation behavior. Bump and spiral retain their original wall-clock sleep timing.

Recordings are in `bags/`: `full_fsm`, `broken_wall_follow`, `bump_protocl`, and `spiral`. From that directory, for example:

```bash
ros2 bag play full_fsm
```

Playback includes movement commands. Stop other controllers and disconnect physical robots before replaying. Playback does not reset the simulation world or pose.
