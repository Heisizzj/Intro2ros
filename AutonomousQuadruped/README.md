#更新了两个检测点是否被占用的测试
1.二维网格是否被占用测试（输出bool值） 接受project map消息 目前不可用仍在测试中

2.检测高度 超过阈值则报告 接受 octomap full 消息 设定resolution为0.05即可 
在代码中可以设定threshold

打印消息为 占用网格且高度超过阈值的path点
ros消息为 非0高度值 

仍在测试中
