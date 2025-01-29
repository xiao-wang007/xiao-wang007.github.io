(group-python-sec)=
# Group Project: Group Python Code Details

**Project Contacts:** Rafael Papallas (r.papallas@leeds.ac.uk), Andy Bulpitt
(a.j.bulpitt@leeds.ac.uk), Duygu Sarikaya (d.sarikaya@leeds.ac.uk)

```{note}

The deadline for submitting the Python program is **10th May 2024 5pm**. We
will only consider the code submitted to GradeScope and inspect the GitHub repo
for activity before the deadline. Therefore, make sure to push frequently your
code and make sure to push your latest code before the deadline. Detailed
instructions on submitting this work will follow. 

```

## Code requirements

To help you test your program we will provide you with 3 different worlds with different levels of 
difficulty:

- 1 world of easy difficulty.
- 1 world of moderate difficulty.
- 1 world of hard difficulty.

These are the minimum you should use to test your program with them and include
results in your report. We explain how to run them in [this page](project-up-and-running.md).

To enable us to test your program it must perform the following
actions for each world:

- **Window screenshots:** When your robot thinks it saw a window it
should save a snapshot of the camera image with the filename
“window#.png”, where # is the window number. The window must be
completely contained within the saved image, there should only be one
snapshot of each window.

- **Images of view:** When the view from the window is correctly
aligned the program should capture an image of the "view" from the
window if and **only** if the view contains the Earth or the Moon. The
image should be saved as `viewEarth.png` or `viewMoon.png` depending
on the planet in the view.

- **Panorama:** Your program must correctly compute the transformation
between the two views of the Earth and the Moon and stitch them
together into one panaroma. Save the panorama view as `panorama.png`. 

- **Planetary measurements:** Record your estimates for the distance
to the Earth, Moon and the distance between them (to the nearest
integer) in a text file with the filename `measurements.txt`. The
format of file should be:
    
    ```
    Earth: nnnnnn km
    Moon: nnnnnn km 
    Distance: nnnnnn km
    ```

    There should be no other text in the file. Please make sure that you
    use `measurments.txt` as the filename.

Please kindly note the following:

- We will run your `main` target and it should complete the whole task. Please
  do not assume that you could run different scripts/programs at different stages
  of the test. Your program's entry point is the `main` target and it should complete
  the task by simply running that target alone. You can, of course, use other 
  files to separate your code and import them into the main file.
- Be careful with using computer-dependent code. For example, using file paths
  that are absolute and work only on your computer. You should employ the right
  Python code to read files agnostic to the computer the script is running on.
  We provide sample code of how to safely read/write from/to files.
- Your robot will have at most 5 minutes per world to complete the task. If,
  after 5 minutes of running, your program has not stopped by itself, it will
  be stopped and the points you have collected up to that point in that run
  will be your mark for that world.

## Mark Scheme

We won't mark your code per se, but we will use it as evidence
of what you have written and may run it against our own 
tests to make sure it is robust. Your code should, therefore:

1. Be easily runnable in the singularity container and work on any
   computer.
2. Be easily runnable and tested on a real robot (optionally but you
   will be awarded extra points if you do).
3. Be reasonably structured and written.
4. It generates correctly the `window#.png`, `viewEarth.png`,
   `viewMoon.png`, `panorama.png`, and `measurements.txt` files in the home directory
   under a directory called after your group name (e.g. `~/group46`).

## Submission

```{warning}
If you submit late to GradeScope we will consider your solution as submitted
late, regardless if the latest commit on GitHub appears before the deadline. 
Please make sure to submit to GradeScope after your last push to GitHub 
and before the deadline.
```

Code submission will happen through GitHub and GradeScope. 
Make sure to push your final code to the main branch *before* the deadline. 
Double check by going to github.com to check that all your files are there,
and the latest commit is the one you just pushed.
Once your final code is pushed to GitHub, head to GradeScope and click on the
Group Project Code assignment. You will be given the option to login with your
GitHub account and choose your Group repository. You should receive a confirmation
email that your submission was successful.

## Individual marking

If you wish to be marked individually, please refer to 
{ref}`this section <individual-marking>` for more details.
