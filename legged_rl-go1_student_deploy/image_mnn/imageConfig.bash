#!/bin/bash
# 确保动态重配置服务器已经启动
sleep 5  # 延迟确保节点已启动并且动态重配置服务器已经激活
rosrun dynamic_reconfigure dynparam set /camera/decimation filter_magnitude 4
rosrun dynamic_reconfigure dynparam set /camera/spatial filter_magnitude 1
rosrun dynamic_reconfigure dynparam set /camera/spatial filter_smooth_alpha 0.8 
rosrun dynamic_reconfigure dynparam set /camera/spatial filter_smooth_delta 5
rosrun dynamic_reconfigure dynparam set /camera/hole_filling holes_fill 0

