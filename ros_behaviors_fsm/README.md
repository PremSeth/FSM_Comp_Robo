# RoboBehaviors and Finite State Machines Project

Matthew Townsend  
Prem Sethumadhavan  
Dionysus Lively

The goal of this project was to familiarize ourselves with ros2, writing to nodes, and using simulators to program a Neanto with a number of behaviors. We chose to use a total of four behaviors: wall searching, wall following, a collision protocol, and drawing in a spiral shape. These behaviors utilized publishing to the cmd_vel topic and receiving and utilizing data from the LiDAR and bump sensors on the Neanto. The four integrate together into a Finite State Machine (FSM) we titled ‘Bar-hopping Bradley’ as it simulates the Neato as a drunk. In completing this project, we learned the importance of sketching out plans for behaviors, using ros2 commands and print statements to debug, and how to integrate multiple systems.

- Wall Searching
- Wall Following
- Collision Protocol
- Spiral Pathing
- Finite-State Controller

**Wall Searching**

&nbsp;&nbsp;&nbsp;&nbsp;The wall searching algorithm uses proportional control to guide the neato to facing its right side parallel to the wall. It accomplishes this by using the laserscan topic to receive a message containing all the lidar scan values. Each index of the list with the lidar scan values represents the angle at which it was taken, with 0 being the back of the neato and the angles increasing sequentially by one counterclockwise. In order to make the right side parallel to the wall we use a proportional controller that defines the error as being 90 - the current minimum range’s index which is its angle value. Specifically the minimum range index on its right side from 45-135 degrees. This was an important change that prevented the neato from accidentally overturning too much in both wall searching and especially during wall finding. This value multiplies against a proportional constant and is inputted as a turning value through a Twist message and published to the cmd_vel topic to make the Neato turn until it is aligned with its right side parallel to the closest wall. This runs until the angle error is less than two degrees. Then it switches to the wall following state.

**Wall Following**

&nbsp;&nbsp;&nbsp;&nbsp;The wall following state uses two proportional constants summed together to change the angle at which the neato drives to maintain the neato driving backwards parallel to its right side on the wall. On my first attempt of the wall following algorithm I didn’t realize that 0 was actually the back of the neato and not the front and when making it drive I realized I needed to make the velocity value negative to get it to drive towards the 0 index. Eventually I realized that it was the back, however the wall following works the same even with it going in reverse, so there was no reason to revert it. As previously mentioned, the two proportional constants for the wall following multiply against the angle error and distance error of the neato. Angle error is defined the same as 90 - the angle of the minimum distance index. This uses the same constant to align. The neato drives at a constant velocity and so this will make sure the neato constantly drives parallel to the wall, but it will not make it drive parallel at a specified distance. In order to that, since the neato is a differential drive we need to change its hedging to steer it closer or farther away from the wall. Therefore the heading of the neato changes by a second proportional constant. This constant however multiplies against the current minimum distance of the neato - the intended distance, capped at -5. The reason we use the current minimum distance instead of the distance at 90 is because at an instance where 90 is not the angle of the current minimum distance when driving, for example while following a circle and it is not always the minimum, this value fluctuates a bunch. However, if it takes the current minimum and essentially only holds a value when the current minimum is not the intended distance it makes the transition into 90 being the minimum angle much smoother. While following if it bumps into a wall it activates the collision protocol.

**Collision Protocol**

&nbsp;&nbsp;&nbsp;&nbsp;The purpose of the collision protocol is to make the Neato run specific commands after being bumped, such as running into a wall or hitting an object on the side. When one of the bump sensors is detected, the Neanto should back up, rotate ~180 degrees, and start traveling in the other direction. To implement this, we used a multi-threaded approach. We subscribe to /bump to receive an indication when one of the bump sensors are triggered, and publish to /cmd_vel to control the Neanto’s linear and angular velocity. We run a thread that uses the run_loop function as a callback. We also use a process_bump function that sets a variable to True whenever the sensor is hit, and a drive function that takes the linear and angular velocity we want to send to the Neanto as input, and publishes the velocities to the Neanto. Within the run loop, the Neanto drives forward as long as the bump sensor is not hit by setting the linear velocity to 0.4. If the bump_state variable is true, the terminal first prints that a bump is detected, then begins the protocol. The Neanto moves backwards for a set time by setting a negative linear velocity, turns 180 degrees by setting an angular velocity for a set duration, and then the bump_state variable is set false making it drive forward again. To set durations, we used the sleep command from the time library. After the duration has elapsed, the next step is to begin the spiral path.

**Spiral Path Following**

&nbsp;&nbsp;&nbsp;&nbsp;Once our spiraling thread is activated after bump recovery, the Neato drives in a spiral motion during which the bump protocol is still active. After orienting itself away from the object it collides with, the Neanto drives counter-clockwise in a semi-circle arc roughly 0.4 meter in diameter–not exactly due to slippage. To accomplish this, we chose a linear speed of 0.5 meters per second and calculated the corresponding angular velocity to complete a 180 degree turn over the course of the time it would take to travel the circumference of the theoretical arch. At this point our Neanto is 0.4 meters to the left–from the Neanto’s perspective–of its starting position and facing the opposite direction. During the second half of the unit spiral, the Neato similarly drives in a semicircle motion, only now we pass parameters to drive in an arc only 0.2 meters across. For this second portion we maintained a constant linear velocity of 0.5 m/s and calculated and published a new high angular velocity to the Neato. We determined time needed to drive each arc and, after publishing the constant velocities for each arc, simply halt further execution of the loop until each arc is theoretically completed. As described above, the bump protocol still functions during this “sleep” period since velocities published by the bump message receiver override the spiral cmd_vel messages that are only sent once per cycle. Finally, with the 0.4 meter diameter and the second 0.2 meter arcs completed the Neato is 0.2 meters to the left of its initial starting position and faces the same direction that it faces initially. Once in this position our loop repeats the same movement once more or until either the bump protocol is activated.

**Finite-State Controller**

&nbsp;&nbsp;&nbsp;&nbsp;For our finite-state controller, we decided to create one node that runs a loop and calls specific functions in case of events. Using the self.state attribute, the loop checks if the Neato is either in the Finding, Follow, Bump, or Spiral state, which then executes the corresponding function. We chose to do one node for simplicity and ease of switching between through nodes by using the attribute. The Neato begins in wall searching mode using the Finding state, and then enters the Follow state when it detects a wall. This begins the wall following until Neato bumps into a wall. We intentionally don’t have the wall following work on sharp 90 degree turns to cause a bump. The neato then enters the Bump state, where it reverses and turns for a set duration. After the duration has elapsed, the Neato begins the spiral state, where the Neato does two spirals for a set duration. Finally, the Neato returns to the Follow state, and cycles between the three Follow, Bump, and Spiral states.

**Challenges & Potential Improvements**

&nbsp;&nbsp;&nbsp;&nbsp;The wall following does oscillate between positive and negative distance error values which we saw through our print statements. We believe a derivative term for our P controller for wall following would make our wall following smoother and the error stay closer to 0 without oscillating. Additionally our group workflow was a bit scattered, we had two people working out of one package on github then another on another package and we finally made an entirely new package for our final submission and pulled in pieces of code from the old packages to get everything together. This made our workflow messy and hard to work with. Additionally, for much of our time working we unknowingly had made a master branch which diverged from main and had been dealing with errors pushing to main as a result. We were able to resolve this issue but it took a lot of extra time out of our working time that could have been dedicated towards making a more robust controller. If we were to do this again we would like to have a cleaner approach to group coding. Our current code is based on the simulated Neato’s behavior, but does not account for the properties of the physical Neato. Some values in our code do not translate well to the physical Neato because they do not account for wheel slipping, friction, and the velocity limits of the Neato. A future improvement for this project is adjusting velocities and durations of Neato commands to account for the limitations of the physical world and Neato.

**Final Takeaways**

&nbsp;&nbsp;&nbsp;&nbsp;For this project, we learned the importance of creating a clear sketch of a plan before implementing code. Instead of trying to add features without a plan, we created diagrams and even a storyline to help us stay on track with the project. This allowed us to assign parts and clearly understand what we need to do to implement our finite state machine. Because of this, our behaviors ended up flowing smoothly by the end of the project. We also learned important debugging and visualization tools when using ros2, such as the topic echo command, node graph viewer, and the Neato simulator. We learned the importance of communication and scheduling meetings in advance to make sure we could all actively contribute to the project together and coordinate outside work accordingly as well.






# Installation & Run Guide

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

[Project writeup](https://docs.google.com/document/d/1coxiCGMOocB-7vDXaP9QN-EjGjESF1iph1r0ITvcFLs/edit?usp=sharing)
