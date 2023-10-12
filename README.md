# A-star-algorithm-implementation-on-turtlebot
## Implementation of a star algorithm on turtlebot simulated in Gazebo using ROS 

## Team:
Sameer Arjun S      https://github.com/Sameer-Arjun-S 
Rashmi Kapu         https://github.com/Rashmikapu
   

## Test Cases: 
```
Enter Robot clearance1
Enter start point X coordinate: 30
Enter start point Y coordinate: 30
Enter start point orientation (Enter a multiple of 30): 0
Enter goal point X coordinate: 230
Enter goal point Y coordinate: 90
Enter Left Wheel RPM3
Enter Right Wheel RPM4
Start point:[170, 30, 360], Goal point:[110, 230]
```

## Instructions to run the program:
```
Run "roscore" on one terminal
Run "roslaunch proj3p2_rashmik_ssarjun proj3p2_rashmik_ssarjun.launch" on another terminal to launch the file
Run "python3 proj3p2_rashmik_ssarjun.py" on another terminal to run the python script to get 2D visualization
```

## Libraries necessary for program:
```
cv2
time
numpy as np
heapq as hq
copy
math
rospy
from geometry_msgs.msg import Twist
```

## Simulation in Gazebo using ROS
https://github.com/Sameer-Arjun-S/A-star-algorithm-implementation-on-turtlebot/assets/112655999/188fe735-e4aa-44e3-ac2d-ff6e6475ce0a

