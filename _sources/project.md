# Project overview

**Project Contacts:** Xiao Wang(x.wang16@leeds.ac.uk), Yanlong Huang
(y.l.huang@leeds.ac.uk) 

```{warning}
### Academic Integrity and Plagiarism - Notice

Please note that plagiarism detection will be conducted for both the reports
and the code. Any instances of plagiarism will be reported for
Academic Integrity. We expect that each student will independently
work on the assignment, and that the solution will be the result from their own
efforts. You are allowed to use sample code given during the labs and 
lectures without reference. You should, however, cite any other work that is
not yours according to the University of Leeds' referencing guidelines.
```

(deadlines-overview-sec)=
## Assessment Overview

This project is summative that accounts for **10%** of the mark.
There are **two submissions** required: 
<!-- <br> (1) your final code in Python 
<br> (2) a video to demonstrate the robot performing the task. 
 -->
<pre> 
1. your code in Python (`~/ros2_ws/src/ros2-project/*') 

2. a video demonstrating the robot performing the task. 
</pre>

<!--
|                     | <p style="color: rgb(255, 0, 0);">Deadline</p> | <p style="color: rgb(255, 0, 0);">How to submit</p>| 
| ------------------- | ----------- | -------- |
|  | 22nd March 2024 5pm   | **Minerva** |

-->
## Project Description

:::
### Project aim and objectives
:::

The aim of the project is to apply what you will learn in this module and
think of ways to solve a robotic problem that requires the combination of
computer vision and motion planning. You are required to write your own ROS2
code in python to let the robot perform the task autonomously in the simulation
hosted by a simulator called Gazebo.

The objectives are as follows:
- learn and apply the fundamental ROS2 framework for robotics systems
- to apply motion planning methods
- to understand and apply basic image processing techniques in autonomous robotic systems

The lab sessions are designed to fulfill these objectives as well as to provide you 
the necessary skills for the completion of the final ROS project.  


```{Note}
We offer the real turtlebot3 burger as well for you to play around and test your
motion commands and image processing modules you have coded through the lab sessions.
It is only formative, which is not accounted for your final grade of this module.
From this week (as of 10/03/2025), a turtlebot will be brought to the usual lab sessions. 
Feel free to ask for it. The usage will be first-come-first-serve.

``` 

:::
### The task
:::

The robot should be able to move and detect the RGB coloured boxed (all three boxes). When a blue box is 
detected, the robot should move up to the blue box and stop at ***roughly within a radius of 1 meter
from the centre of the blue box***. The world looks like this:


```{figure} images/world_2025.png
Task world: the robot's starting position is located at the bottom right compartment.
```

:::
### What has been provided:
:::

The map of the task world is provided in the following git repo:


```{note}
**ros2 project - Instructions**
- In a terminal, go to `cd ~/ros2_ws/src`.
- remove the old `turtlebot3_simulations` package inside your `~/ros2_ws`
- clone the new `turtlebot3_simulations`, which has the new world ready for you:
    - `git clone git@github.com:COMP3631-2025/turtlebot3_simulations.git`
    - then switch to the project branch by doing: 
      `git checkout 2025_ros_project`
- Run: 
`git clone git@github.com:COMP3631-2025/ros2-project <space> ros2-project_<yourUniAccountUserName>`
    - Note at the end of the the command: `git clone [...] ros2-project_xxx`
    - the map is already provided in `~/ros2_ws/src/ros2-project/map`
- Your code for the project will be in the project package `~/ros2_ws/src/` `ros2-project_<yourUniAccountUserName>/ros2-project_<yourUniAccountUserName>`
    - Notice that you should follow the same package file system as in previous labs codes.
- I would recommend to copy the `/map` to lab5 to use its package structure and start from there. Because 
  the package `ros2-project_xxx` is not a complete package yet. So it won't build. So after copying the `map`, delete `ros2-project_xxx`, so the build will won't produce errors.
- Once you have code it up. You can now make the whole package named `ros2-project_xxx`. And copy your code in, then follow a package structure (using previous labs as templates). Run `cd $HOME/ros2_ws` and then `colcon build` to build the new package.
```

:::
### Some hints:
:::
- motion planning helps you to explore the map if you don't see any colours. The map is a massive collection of coordinates (your robot's configuration space). How to reduce it for efficient exploration.
- OpenCV provides functions to calculate the contour area from your colour detection, e.g. `cv2.contourArea()`. Appearing as big or small relative to some threshold value, can inform the robot move forward or backward.
- you can also calculate the location of the CoM of the contour in the camera feed, e.g. with the help of `cv2.moments()` such that to inform the robot to steer left or right.


### Specification and marking scheme ($10\%$)


1. Your code runs with no problem ($3\%$)
2. Using what you learnt from the lectures, write a motion planner to command your robot ($3\%$)
3. Uploaded video to demonstrate a working solution, make sure the view is clear:
    * Robot has a live streaming of the camera feed with the detected objects ($3\%$)
4. Your robot finishes the task, and stays near the blue box ($1\%$)

### Submission instructions
1. Please create a public github repo of your own. Then push your code to it. This will make a `.git` link available to share.
2. Upload your video to your uni one drive. This will also make a link available to share.
3. Further guidance on how to submit the two links will be released soon.