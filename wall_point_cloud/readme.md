# 项目简介

**项目目标：**本项目为“墙面打磨/喷涂机器人”墙面点云预处理及路径规划模块代码实现，对由在gazebo仿真环境中使用激光雷达扫描获得的房间墙面三维点云数据进行预处理，并在目标打磨/喷涂墙面上进行全覆盖路径规划。

**主要工具：**”open3D“ 3D数据处理库

# 项目文件组成

`Target_wall_extraction.py`:主要实现读取原始房间墙面点云文件、直通滤波、体素下采样、

基于法向量的目标墙面（机器人正对着的墙面）筛选以及边界特征提取等功能。

`traj_plan.py`:主要实现将平面点云映射到二维图像、二维图像处理筛选出待打磨/喷涂区域、全覆盖路径规划等功能。

`accumulated_cloud.pcd`:为原始房间墙面点云文件

`cloudPointEdge.pcd`、`cloudPointEdge_zhitong.pcd`、`inlier_cloudToPlane.pcd`为`Target_wall_extraction.py`文件点云处理过程中产生的中间文件。

`inlier_cloudToPlane.pcd`：为映射到平面的目标平面点云，可进一步转换获得对应二维图像。