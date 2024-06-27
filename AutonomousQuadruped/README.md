## Getting Started


1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiLvgiTXetubiN1i4PRjuR/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run a test:
  a.) roslaunch simulation simulation.launch
  b.) rosrun controller_pkg controller_node
  
You will see the quadruped in front of you and it is walking forward. The goal is to navigate the quadruped through the parcour. If you take a look to the source-code of the controller_node you will see how to control the quadruped with the inputs Phase, SkewPhase, Amplitude change, Amplitude back and Frequency (of the legs).

Part of the task is to play with these control inputs in order to make the robot walk. You will have to experiment with the differend commands to pass the parcour. To give you another hint. A possible combination for rotation in spot is: 

Phase = 0
Skew Phase = 45
Amplitude Change = 0
Amplitude Back = 0
Frequency = 7


# Tips

Here are a couple of hints regarding the implementation. The hints are just suggestions; you are free so solve the task differently:
- Generating point cloud from depth image: use depth_image_proc in http://wiki.ros.org/depth_
image_proc.
- Generating occupancy Grid: use Octomap in http://wiki.ros.org/octomap.
- Please ping us in case you have any questions or if you get stuck in some subtasks.
- Use a global map as your voxel grid representation. Use a smart resolution for your voxel grid representation (e.g. 1m).

