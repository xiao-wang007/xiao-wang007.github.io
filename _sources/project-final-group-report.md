(group-final-sec)=
# Group Project: Final Group Report (35%) Details

**Project Contacts:** Rafael Papallas (r.papallas@leeds.ac.uk), Andy Bulpitt
(a.j.bulpitt@leeds.ac.uk), Duygu Sarikaya (d.sarikaya@leeds.ac.uk)

```{note}
The deadline for the group report is **10th May 2024 5pm** and worths 35%.
Detailed instructions on submitting this work will follow. 
Late submissions are subject to standard university penalty; 5% per late day unless there
are [mitigating circumstances](https://students.leeds.ac.uk/info/10111/assessment/860/mitigating_circumstances).
```
Write up your solution as a group, as if it was a report to a client.
This should be no more than 10 sides. In particular;

- Include details of the design options you considered and justification of why you chose the particular options you did.
- Describe how you have tested your solution. Give examples of different environments/maps you have created to test your program, and the performance of your program in the environments we have provided.
- Include in your report images, a link to a video and data to demonstrate how your solution works or fails. Outline and discuss the limitations of your proposed approach. Suggest scenarios where it might not work.
- State any OpenCV/ROS codes you have used that are not part of the standard distribution. 

### Markscheme

**Design (17 points): Marks will be awarded for:**

- Justification of decisions and general knowledge of possible methods
- Is the design structured well? Are different sub-tasks identified well? Are all of them identified?  Are they integrated well?
- Was efficiency in mind during the design?
- Was robustness in mind during the design? Likelihood of working in a wide range of environments and images (other than those provided)?
- How novel is the design?

**Implementation and Results (8 points): Marks will be awarded for:**

- Efficiency/accuracy of reaching to the rooms, use of planning and search methods.
- Accuracy of identification of the room colours and the windows (especially Earth and Moon).
- Testing and analysis of performance, whether successful or unsuccessful.
- Use of concrete evidence (numbers, figures, tables, and diagrams) to evaluate performance.
- Include pictures from experiments to show certain aspects of your solution or to explain certain limitations.
- Include a **single video** (it can be a link to a YouTube unlisted video) which includes demonstrations
  of your solution in the three provided worlds and in any other world you have created. The individual video of
  each world should be unedited and include the following for each world:
    * Gazebo simulator with clear view of the robot moving in space.
    * Terminal window showing when you start the program.
    * File manager showing initially an empty folder `~/groupX` (where X is the
      number of your group) and while the robot is running screenshots and `measurements.txt` 
      being saved in that directory. If the directory isn't shown in advance and isn't empty 
      we will not consider your video.
    * Once the robot finished running, open and show the saved screenshots, as
      well as the contents of `measurements.txt`.

**Real robot test (5 points): Marks will be awarded if you test your program on a real Turtlebot:**

- Describing what needed to be done to make the program work on the real robot.
- Describing how the robot performed in the real world.
- Include a **single video** (it can be a link to a YouTube unlisted video) which includes demonstrations
  of your solution in the real world. The video 
  should be unedited and include the following:
    * A clear view of the robot moving in space.
    * Terminal window showing when you start the program.
    * File manager showing initially an empty folder `~/groupX` (where X is the
      number of your group) and while the robot is running screenshots and `measurements.txt` 
      being saved in that directory. If the directory isn't shown in advance and isn't empty 
      we will not consider your video.
    * Once the robot finished running, open and show the saved screenshots, as
      well as the contents of `measurements.txt`.

**Writeup (5 points): Marks will be awarded for:**

- Clarity of presentation of solution and results [N.B. Large chunks of code with no explanation are unlikely to gain high marks!].
- Discussion of the strengths and weaknesses of the system presented.
- Presentation.
- Use references to credit the resources you used, if any.


### Formatting and limits

Please use the following formatting rules and limits:

1. Page limit is 10 sides.
2. Appendices must only contain the outputs of your program as defined above under the Testing.
3. Title page, Table of Contents and references do not count to your page limit.
4. Use 10pt font size.
5. Use an A4 paper size.
6. Use single line spacing.
7. Use 2cm for margins.

### Submission instructions

The report is to be submitted electronically on Minerva through Turnitin **as a
PDF file**. Only one member of the group should submit this group report. As
discussed, the code should be submitted to GitHub. Please name your PDF as your
group name (i.e., `GroupX.pdf` where `X` is your group number).

## Individual marking

If you wish to be marked individually, please refer to 
{ref}`this section <individual-marking>` for more details.
