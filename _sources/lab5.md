# Lab 5: Computer Vision: Processing robot's camera view

<!-- **Worksheet Contact:** Andy Bulpitt (a.j.bulpitt@leeds.ac.uk) -->
**Worksheet Contact:** Xiao Wang (x.wang16@leeds.ac.uk)

```{note}
This worksheet should be completed in Week 6 and labs in week 6 will provide support
for this worksheet.

**Access lab5 files - Instructions**

To access the lab5 files, you need to click on this url
[classroom.github.com/a/rLKCMm-Z](https://classroom.github.com/a/rLKCMm-Z),
login to your
GitHub account and click the button to accept the lab5 assignment.

As always, before you start running any commands or code for this worksheet,
make sure {ref}`you are in a Singularity environment <singularity-command>`.

Then execute the following commands:
- In a terminal, go to `cd ~/ros2_ws/src`.
- Run: `git clone git@github.com:COMP3631/lab5-YOUR_GITHUB_USERNAME lab5`
    - Note the " lab5" at the end of the the command: `git clone [...] lab5`.
- Run `cd $HOME/ros2_ws` and then `colcon build` to build the new package.
```

## Worksheet Objective

There are two aims to this worksheet: First to learn the basics of taking in data from the
camera, processing it, and then making decisions based on it and second to learn how to perform image stitching.
By the end of this session, you will:

- Be able to display the camera feed to the screen.
- Extract a specific colour from the image and only display this.
- Make decisions based on the colour detected (and possibly the size of the object).
- Move towards a specific colour object.
- Stop when another colour is detected.
- Be able to match features in images and use these to stitch images together.

## Prerequisites 

Before you start working on this worksheet, make sure to do the following one-time
configuration:

1. Open a terminal window and navigate to your workspace's source directory:
   `cd ~/ros2_ws/src/turtlebot3_simulations`.
2. Run: `git pull` to receive some new files for this lab.
3. Navigate to your workspace and rebuild: `cd ~/ros2_ws && colcon build`.
4. Remember to work in Code with the embedded terminals or run `source
   ~/.bashrc` in the current terminal.

## Starting Turtlebot Simulator with a different world file

In this lab, we will load the simulated Turtlebot into a different environment.
You will find the `rgb.world` file under `$HOME/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds`.
Now in a terminal, execute:

```
ros2 launch turtlebot3_gazebo turtlebot3_rgb_world.launch.py
```

This command starts the simulator, as in the previous lab
session. In the simulator environment, you should now see three spheres of red,
green and blue colour. You can grab and move them around.

## Display camera feed to the screen (and convert the image format)

Now let’s focus on getting your python node capable of reading the camera data
and then getting it to display to a screen. Open the file
`Skeleton_Code_First_Step.py` in `lab5/lab5` and save a copy to `first_step.py`. When following the instructions below modify `first_step.py` so that the original code is still there for reference/backup if needed.  

### OpenCV

To process images and make decisions based on them we will be using a library
called OpenCV (Computer Vision). Specifically, we will be using OpenCV2 so in
any python scripts that you want to handle images you need to import cv2:

```python
import cv2 
```

Since we are handling ROS we also need the use of a library called
`cv_bridge`, which can translate ROS images into images readable by OpenCV. (More
info about `cv_bridge` is here: http://wiki.ros.org/cv_bridge/Tutorials)

### Importing necessary modules

We always start by importing the necessary python modules and classes:

```python
import cv2
import threading 
import numpy as np 
import rclpy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.exceptions import ROSInterruptException
import signal
```

### Subscribing to the image topic

In order to receive and process image data from the cameras we must create a
subscriber to the topic that our camera outputs to. The RGB image from the 3D
sensor is on the topic: `camera/image_raw`. Create a subscriber for this
topic in the constructor (`__init__`) of the `colourIdentifier` class, and specify
the callback function of the `colourIdentifier` class as the callback of the
image topic.

### Converting between openCV and ROS image formats

The image from the camera arrives in the ROS type Image. To manipulate it in
OpenCV, we must convert he image from the ROS format of Image into an OpenCV
image. OpenCV has built in functions to do this for us. Call the
function `imgmsg_to_cv2(data, "bgr8")` on a `CvBridge` object you have created in
your class. Finally, output the camera feed to the window on your screen.

### Displaying image on a new window

We declare that we want to have a named window called 'camera feed' and then we
show it. As the camera resolution is very high we need to resize the window to ensure it fits on the screen. 
This is very important if you are using feng-linux.

```python
cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL) 
cv2.imshow('camera_Feed', <image name>)
cv2.resizeWindow('camera_Feed', 320, 240) 
cv2.waitKey(3) 
```

Try to build and run your script `first_step.py` and see if it works using

```
colcon build
ros2 run lab5 first_step
```

If you only wish to build a single package you can use:

```
colcon build --packages-select lab5
```

## Manipulating the camera feed to produce new images

Next, we're going to look at manipulating our image to isolate all parts of a
certain colour. Isolating and detecting colours from each other is a very
useful tool for robot computer vision.

Notice that when we converted the ROS image into an OpenCV image
that "bgr8" was given as an argument. This is the original colourspace of the
camera feed. This is usually the default colour space of camera images. RGB colour space is 
not convenient in terms of image processing. Because the hue, value and intensity are mixed,
which means if you change the pixel values in any channel of an RGB image, these three components
change at the same time. For each channel, the pixel value is `0~255`, therefore in RGB you have 
$255^3 = 16581375$. This is the source of the hype when you buy an RGB gaming gear with the claimed 16 
million colour choices! In a lot of image processing cases, we are really dealing with only the 
hue. Therefore, HSV (hue, saturation, value) or HSI(hue, saturation, intensity) colour space is
used. For colour manipulation, we are only working with the Hue channel instead of working with all
three channel in RGB colourspace. If you are interested in the conversion, search online for the math.
 OpenCV provides methods for converting from one colourspace to another. This can be done by:

```
 hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
```

Once we have converted the image, what colour we need to select.
In the example below, we are selecting the green colour, which is centred at 60. When we are talking
about colours, we are usually referring to a band of wavelengthes rather than a single wavelength.
By giving it a range, we allow more robustness of detecting green colours. Our perception of colours 
varies slightly from person to person because of psychophysiological reasions. Then using a range of 
hue values to define a colour makes sense. To further widen the green colour detection, we also set a
range for the saturation (less green to greener) and value (less bright to brighter):

```
hsv_green_lower = np.array([60 – self.sensitivity, 100, 100])
hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
```

As a parameter, self.sensitivity can be initialized in `__init__`.

Please note that looking up the Hue for HSV online you will find it is in the
range of 0-360, however for processing in OpenCV it is set from 0-180 instead.
Therefore, the following values will be useful to note for using HSV in OpenCV;
Red will be around $0$ and $180$, green will be $\approx 60$ and blue will be 
$\approx 120$. Since the red is at the ends of the spectrum, you will need to 
define two ranges, i.e. `0~sensitity` and `180-sensivity ~ 180`.

Then we want to create a mask image filtering out anything that isn't within
those three colours. The `inRange` method will return a binary image with the 
same height and width as the camera image. But with 1 as pixel values at locations
where the colour falls inside your specified range, and 0 pixel values otherwise:

```
green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
```

Then to select all three colours. We can combine the masks togather as following:
```
rg_mask = cv2.bitwise_or(red_mask, blue_mask)
```
Because it only takes two masks at the same time, you need to do it again using the 
`rg_mask` with `blue_mask` to incorporate the blue colour. In the excercise later, you
can play around and show the indivdually filtered colour images onto the screen with one window for
each. 

Finally, you can get the filtered image of the robot's camera feed by applying
the combined mask to the camera feed (rgb image) and display:
```
filtered_img = cv2.bitwise_and(image, image, mask=<YOUR MUSK>)
cv2.namedWindow('camera_feed', cv2.WINDOW_NORMAL)
cv2.imshow('camera_Feed', filtered_img)
cv2.resizeWindow('camera_feed', 320, 240)
cv2.waitKey(3)
```

### Exercises

```{note}
[Here](https://docs.opencv.org/4.7.0/) is a link to the OpenCV documentation 
which you will find useful for this lab worksheet and the project.
```

For each exercise below we recommend you copy the Skeleton code to the file suggested before editing it.
To move the robot around the world you can use the teleop_keyboard package:
```
ros2 run turtlebot3_teleop teleop_keyboard
```

1. Try completing the skeleton code `first_step.py` to filter out all colours apart from one in
   an image to start with, for example green.
2. Create a file `second_step.py` from `Skeleton_Code_Second_Step.py`.
   Try adding a second and third colour (red and blue) to ones you want to
   isolate from the initial image and produce an image similar to the output in the previous example. 
3. In addition to colour, you can also use methods to tell if you are going away or coming closer to a target but counting the number of pixels inside a contour. You can extract contours from the mask image (binary image). 
```
contours, _ = cv2.findContours(green_image,mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
``` 
Then you can may find the largest contour by: `c = max(contours, key=cv2.contourArea)`. In addition, you may use other handy functions that OpenCV provides, such as:
```
cv2.minEnclosingCircle() 
cv2.boundingRect() # draw a bounding rectangle to contour
cv2.rectangle() # draw a rectangle 
cv2.circle() # draw circle
```
Please refer to the OpenCV API for more information. Create a file `third_step.py` from `Skeleton_Code_Third_Step.py` and complete the
   skeleton code to send messages based on what colours
   are present in the image view. You can define a flag in the constructor, such as `self.green_found = false`. Follow the comments. You could print when the
   colour is detected or send the message to the lab2 talker/listener.
4. Now that you have the flags based on a colour being present, try creating
   one further node (`fourth_step.py`) that can instruct the robot to follow a particular colour,
   stopping when it catches sight of another colour. Choose two colours – for
   example follow green but stop if red is detected. You can include the
   publisher for the movements from lab3, then instruct to move depending on
   if the colour is detected, if it’s over a certain size, move towards the
   object. If the second colour is detected then stop. The skeleton code can be found in `Skeleton_Code_Fourth_Step.py`.

## Submit to GitHub

Submission instructions: When you think you have completed this worksheet, add,
commit, and push your new files and changes to the git repository with the
correct git message “Lab5 completed.”:

```
cd $HOME/ros2_ws/src/lab5/ 
git add . 
git commit -m "Lab5 completed."
git push
```

Please commit your completed versions of exercises 1, 3, 4 from Part A and Part B:
first_step.py, third_step.py, fourth_step.py and stitch_step.py.

## Remarks and Checklist

Please check, by the end of this worksheet, that ...

- You understand how to run Gazebo simulator with different world file.
- You can see the robot's camera view using a Python code.
- You can process the camera feed to produce new images.
- You completed successfully all exercises.
- You pushed your code to GitHub.

If you have any questions or problems, please kindly ask one of the Teaching team
for the module for help during the lab hours, or post a message on the Teams group.
