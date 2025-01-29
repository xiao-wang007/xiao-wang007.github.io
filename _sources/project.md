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
1. your final code in Python 

2. a video demonstrating the robot performing the task. 
</pre>


|                     | <p style="color: rgb(255, 0, 0);">Deadline</p> | <p style="color: rgb(255, 0, 0);">How to submit</p>| 
| ------------------- | ----------- | -------- |
|  | 22nd March 2024 5pm   | **Minerva** |


## Project Description

:::
### Project aim and objectives
:::

The aim of the project is to apply what you will learn in this module and
think of ways to solve a robotic problem that require the combination of
computer vision and motion planning. You are required to write your own ROS2
code in python to let the robot perform the task autonomously in the simulation
hosted by a simulator called Gazebo.

The objectives are as follows:
- learn and apply the fundamental ROS2 framework for robotics systems
- to understand rigid body motion and apply motion planning methods
- to understand and apply basic image processing techniques in autonomous robotic systems

The lab sessions are designed to fulfill these objectives as well as to provide you 
the necessary skills for the completion of the final ROS project.  


```{Note}
We offer the real turtlebot3 burger as well for you to play around and test your
motion commands and image processing modules you have coded through the lab sessions.
It is only formative, which is not accounted for your final grade of this module.
Details of the time and usage of the real turtlebot3 burger will be announced later 
in this term. 
``` 

:::
### The task
:::

The robot should be able to move and detect the RGB coloured boxed. When a blue box is 
detected, the robot should move up to the blue box and stop ***within a radius of 1 meter
from the centre of the box***. The coordinates of the boxes are provided.


### Box coordinates (x,y) in metres
The boxes are cubes with edges of 1m; the provided coordinates are the centre of mass of 
the boxes. The RGB triad shown in the image below corresponds to the world frame $xyz$,
to which the box coordinates are referenced.
<pre>
Blue box:  (-6.325623, -9.266654)
Green box: ( 2.289328, -8.241118)
Red box:   (-8.924699,  0.009007)
</pre>


```{figure} images/world_2025.png
Task world: the robot's starting position is located at the bottom right compartment.
```

### Specification and marking scheme ($10\%$)


1. Your code runs with no problem ($2\%$)
2. Using what you learnt from the lectures, write a motion planner to command your robot ($3\%$)
3. Uploaded video to demonstrate a working solution, make sure the view is clear:
    * Robot has a live streaming of the camera feed with the filtered detected object ($3\%$)
    * Once the robot finished running, print to console the distance between your robot and the centre
      of the blue box. ($2\%$)

### Submission instructions