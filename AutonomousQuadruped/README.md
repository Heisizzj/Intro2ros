## Getting Started


1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiLvgiTXetubiN1i4PRjuR/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run:
  a.) roslaunch simulation simulation.launch
  a.) roslaunch perception perception.launch
  b.) roslaunch planning planning.launch
  c.) rosrun controller_pkg controller_node
  
## Dependencies

Install octomap, octomap-rviz-plugin, depth-image-proc, move-base:
`sudo apt-get install ros-noetic-octomap-rviz-plugins`
`sudo apt-get install ros-noetic-octomap-server`
`sudo apt-get install ros-noetic-depth-image-proc`
`sudo apt-get install ros-noetic-move-base`

