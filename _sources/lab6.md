# Lab 6: Real-robot lab

<!-- **Worksheet Contact:** Rafael Papallas (r.papallas@leeds.ac.uk) -->
**Worksheet Contact:** Xiao Wang (x.wang16@leeds.ac.uk)

```{warning}

Lab 6 is an attendance-based lab starting from 4th of March. Please attend
the session in your timetable.

When you attend the lab, join a team (doesn't have to be your group
project team), every member of your
team should sign in with one of the Teaching Assistants (TAs). The TAs should
have a sign-in list with your names. Please form teams of at most 5 people, 
no more than that.

```

```{note}

You can access the robots to play with what you learnt from the labs, or test the modules for 
your project starting from Week 7 onwards. Access to the robots will be provided upon request
 during lab hours. 

```


## Worksheet objective

The purpose of this worksheet is to learn how to use a real robot. The group
project offers additional points for implementing your solution on a real
robot. This lab will provide you with all the necessary information to
accomplish this.

You will be working in teams, and we expect all of you to actively participate
in using the real robot. You can all collaborate on the same robot and PC.
There is no need to submit anything for this lab.

You can form random teams during the lab sessions; you do not have to be in the
same group as the one assigned for the group project, for example.

## Setup your PC and learn how to use the real-robot

Before beginning the exercises, please carefully read [this
page](using_the_real_robot.md) and set up your account accordingly.

## Exercises

As a group, please complete the following exercises to ensure that you
understand how to fully operate the robot, including actuation, using the
camera for OpenCV tasks, and navigation and mapping.

```{note}
**Access lab6 files - Instructions**

To access the lab6 files, you need to click on this url
[https://classroom.github.com/a/TPWyS0Hc](https://classroom.github.com/a/TPWyS0Hc),
login to your GitHub account and click the button to accept the lab6
assignment.

Then execute the following commands:
- In a terminal, go to `cd ~/ros2_ws/src`.
- Run: `git clone git@github.com:COMP3631/lab6-YOUR_GITHUB_USERNAME lab6`
    - Note the " lab6" at the end of the the command: `git clone [...] lab6`.
- Run `cd $HOME/ros2_ws` and then `colcon build` to build the new package.
```

First things first, make sure that you:

1. SSH to your robot (if you don't know how, you should read [this
   page](using_the_real_robot.md) first).
2. Start the robot drivers using:
```
ros2 launch turtlebot3_bringup robot.launch.py
```

That's the **only command** you need to run on the robot. Keep the terminal window
with the SSH session open, however.

The rest of the commands mentioned in this worksheet should be carried out on a
**desktop PC, not on the robot**. 

Here is an easy way to get started:

1. Enter into a singularity environment using the `ros` on the desktop PC.
2. Launch Code using `code`. Use the embedded terminals in Code to execute the
   rest of the commands, however ...
3. When you open a new terminal in Code either (1) run `workon_real_robot` pick the name from the
   list, (2) directly run `workon_real_robot <name_of_robot>` or (3) simply put
   `workon_real_robot <name_of_robot>` in your `.bashrc` to not have to do this
   with every new terminal window.


:::
#### Exercise 1: Attempt to run the `firstwalk.py`
:::

```{warning}
Please never command the robot to drive faster than 0.2 m/s.
```

In a terminal window on the desktop PC, run:

```
ros2 run lab6 firstwalk
```

This is essentially the same file as in [lab 3](lab3.md), with no modifications
to make it work on a real robot, thanks to ROS abstraction. If the robot
moves back and forth, then you've done it! Great job. Feel free to optionally 
play with different velocities (don't forget to `colcon build` after the changes).

:::
#### Exercise 2: make the robot to move on a circle and square
:::

Now, try creating `square.py` and `circle.py` files under
`~/ros2_ws/src/lab6/lab6` on the PC. Make the robot move in a circle and a
square (you can refer to your solution from `lab3`). Remember to modify
`setup.py` to let ROS know about your new scripts. After making these changes,
run `colcon build` under `~/ros2_ws`. 

Test that your scripts are working on the real-robot by running:

```
ros2 run lab6 circle
```

and then:

```
ros2 run lab6 square
```

:::
#### Exercise 3: working with image frames
:::

There is a camera on the TurtleBot. The camera runs when you launch the
`turtlebot3_bringup`.

Initially, we considered using a third-party library for the sensor that was
publishing the image view to the same topic as the one you subscribed to in the
simulation labs (i.e `/camera/image_raw`). However, this resulted in slow
streaming with a lot of lag. Therefore, we developed our own ROS node that
reads the image frame directly from the robot, compresses it, and sends the
compressed image over the network at our desired frame rate. This led to much
more efficient streaming with minimal lag. You can find our custom code
[here](https://github.com/rpapallas/pi_camera_ros), if optionally interested 
in reading the code. 

Now, check the list of topics (`ros2 topic list`). You should see new topic
names, including `/camera/image/compressed`. If you don't see anything, make
sure you run `workon_real_robot`.

This technical issue also presents a new learning opportunity for you. Your
task is to create a new node that will interface the compressed image topic
with the `image_raw` one (which does not currently exists), so that your code
remains consistent whether working in simulation or with the real robot. You
will run this new node when working with the real robot to make
`/camera/image_raw` available.

You will find a script named `compressed_to_image_raw.py`. Open it and complete
the `TODO` comments.

Once you finished with the TODO comments, make sure to run `colcon build` and
then run (don't forget `workon_real_robot`):

```
ros2 run lab6 compressed_to_image_raw
```

Now your node should read the compressed image over the network, decompress it
and republish it as a raw image under `/camera/image_raw`. Since the conversion
and re-publishing happens on the PC, when we subscribe to the
`/camera/image_raw` on the PC again (say in your `group_project` code), it
would be faster than subscribing to a `/camera/image_raw` published over the
network by the robot.

Test it (don't forget `workon_real_robot`):
```
ros2 run image_view image_view image:=/camera/image_raw
```

Waive to the camera and smile.

:::
#### Navigation and Mapping
:::

In [Lab 4](lab4.md), we covered mapping and navigation. The commands you learned there
also apply here. On the desktop PC (not the robot), you can use the same commands as
you did in simulation (again, don't forget to run `workon_real_robot`).

**Start mapping:**

```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

**Teleoperate robot from keyboard:**

```
ros2 run turtlebot3_teleop teleop_keyboard
```

**Save map to file:**

```
ros2 run nav2_map_server map_saver_cli -f $HOME/ros2_ws/src/lab6/maps/mymap
```

**Load a saved map:**

```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/lab6/maps/mymap.yaml
```

:::
#### Exercise 4: Create a simple map and save it to a file
:::

For this exercise, simply experiment with the commands provided above to ensure
they work and that you know how to generate a map. 

1. Generate a map using the keyboard teleoperation program to move the robot
   around from the desktop PC.
2. Save the map to a file.
3. Reload the map from file using the `turtlebot3_navigation2` stack.
4. Using Rviz, send navigation goals to the robot and see it moving towards that goal.

## Remarks and Checklist

Please check, by the end of this worksheet, that **all members** of the group ...

- Understand how to use the real-robot.
- you know how to send velocities to the robot.
- you know how to utilise the camera with OpenCV.
- understand how to run code on the real robot (i.e., you only run `robot.launch.py` on
  the real-robot, you can run the rest on the desktop PC).