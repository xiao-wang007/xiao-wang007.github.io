# Frequently Asked Questions (FAQ) and Troubleshooting

## I can see topics I didn't publish or I get an error that my network is compromised?

ROS2 by default connects to the network your computer is connected to. Traditionally,
ROS1 used to work localhost by default. Please run the following command to ensure
you are always working localhost:

```
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

## I have issues with Gazebo simulator, it won't launch properly

If you experience issues with Gazebo, it could be that the Gazebo server is
running in the background. Try to kill it first and try again:

```
pkill -9 gzserver
```

(download-singularity)=
## Can I download the singularity image on my computer?

Yes. You can from [here](https://leeds365-my.sharepoint.com/:u:/g/personal/scsxwang_leeds_ac_uk/ERslhTjaMZNCmcJYNKcU2_EBV3ggSAOZ6xk07HJ98pnjqQ?e=NZyJS8).
This is the exact same image with the one we use in the labs. You will need an 
Ubuntu machine, however. We cannot support with individual configuration issues.

(map-saver-issue)=
## I can't save my map to a file I get an error saying "Failed to spin map subscription"

This seems to be an issue when using terminal windows outside Code and in particular
when you run the `cartographer.launch.py`. Please try to run all the commands
within the embedded windows of Code (and make sure to launch Code from within
the Singularity environment).

If you get this error while working on the real-robot, don't forget to run
`workon_real_robot` and select the right name of your robot from the list.

(ssh-connection-issue)=
## When I try to ssh into a robot, I get an error: "Too many authentication failures"

Please try the following:

```
ssh -o PubkeyAuthentication=no student@YOUR_ROBOT_NAME.local
```

## Some topics still show up even though nothing is running

One way to forcefully kill any process that contains the keyword 'ros' is
the following, however, be careful as this command may match something else
that isn't related to ROS:

```
ps aux | grep [r]os | awk '{print $2}' | xargs -r kill -9
```

