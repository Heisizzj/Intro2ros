<<<<<<< HEAD
## Getting Started


1. Clone the repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiLvgiTXetubiN1i4PRjuR/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run:
  a.) roslaunch simulation run.launch
5. There's a keyboard control mode for testing control params:
  a.) roslaunch simulation simulation.launch
  b.) roslaunch perception perception.launch
  c.) roslaunch planning planning.launch
  d.) rosrun controller_key_control key_control
  e.) roslaunch key_board keyboard.launch
 
## Dependencies

Install octomap, octomap-rviz-plugin, depth-image-proc, move-base:
`sudo apt-get install ros-noetic-octomap-rviz-plugins`
`sudo apt-get install ros-noetic-octomap-server`
`sudo apt-get install ros-noetic-depth-image-proc`
`sudo apt-get install ros-noetic-move-base`

=======
#更新了两个检测点是否被占用的测试
1.二维网格是否被占用测试（输出bool值） 接受project map消息 目前不可用仍在测试中

2.检测高度 超过阈值则报告 接受 octomap full 消息 设定resolution为0.05即可 
在代码中可以设定threshold

打印消息为 占用网格且高度超过阈值的path点
ros消息为 非0高度值 

仍在测试中
>>>>>>> cd66d21... 修改了高度检查代码，现在可以检查path在被占用网格上的高度了
