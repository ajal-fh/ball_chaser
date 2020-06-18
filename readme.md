# ball_chaser

ROS package for a differential drive robot with camera to chase a ball using OpenCV Hough Circle transform.

## drive_bot

Provides a ROS service server to command linear and angular velocities to a robot.

**Subcribes**: _/cmd_vel_

**Available service**: _/ball_chaser/command_robot_

###Usage

```bash
 
$ rosrun ball_chaser drive_bot
$ rosservice call  /ball_chaser/command_robot 0.5 0.1
```
## proces_image 

Finds the ball using Hough Circle Transform


### Usage

For testing the simulation from _track_robot_ could be used

`roslaunch ball_chaser ball_chaser.launch`


