# Guide to start

## Terminal 1
```roslaunch turtlebot3_gazebo turtlebot3_house.launch```

## Terminal 2
```roslaunch robotics-final nav.launch```

## Terminal 3
```roslaunch robotics-final task_manager.launch```

## Caution
Please note that ```robotics-final``` argument in the commands might vary depending on the name of the folder.
You also need to perform ```catkin_make``` in your catkin main folder to build everything before you start.
Finally, you need to install turtlebot3 packages from ```sudo apt install``` instead of using the existing packages, as they are deemed faulty.

## Brief description
The project controls a vacuum cleaning robot around the house according to the assigned nodes and rooms. Unfortunately, QR code verificaition was not yet applied, but the rest should work perfectly.

