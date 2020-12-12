1. 输入roslaunch mbot_gazebo mbot_laser_nav_gazebo.launch 启动机器人模型和gazebo
2. 启动mapserver 命令为rosrun map_server map_server catkin_ws/src/mbot_navigation/maps/room.yaml
3. 启动AMCL节点 命令为 rosrun amcl amcl scan:=scan
4. 启动导航功能 rosrun mbot_navigation Nav.py 
	--完成以上步骤后，即可在gazebo中查看机器人状态。也可以启动rviz查看
	
	--导航功能
		暂时默认起点为（-1，-1），目标点为room4。起点和目标点可在Nav.py文件中更改。
		可通过初始化Nav类，传入不同参数实现目标点和起始点的修改。（在集成语音后会对Nav类进行调整）
		初始化Nav类，需要参数为起始点坐标（X，Y）和房间号码（共设置了6个房间，起始值为0。详情请看room.png）
