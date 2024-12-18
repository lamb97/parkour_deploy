#!/bin/bash

function showHelp(){
    echo
    echo "This script can delay the launch of a roslaunch file"
    echo "Place it in the 'scripts' folder of your catkin package"
    echo "and make sure that the file is executable (chmod +x timed_roslaunch.sh)"
    echo
    echo "Run it from command line:"
    echo
    echo "Use: ./timed_roslaunch.sh [number of seconds to delay] [rospkg] [roslaunch file] [arguments (optional)]"
    echo "Or: rosrun [yourpackage] time_roslaunch.sh [number of seconds to delay] [rospkg] [roslaunch file] [arguments (optional)]"
    echo "Example: ./timed_roslaunch.sh 2 turtlebot_navigation amcl_demo.launch initial_pose_x:=17.0 initial_pose_y:=17.0"
    echo
    echo "Or run it from another roslaunch file:"
    echo
    echo '<launch>'
    echo '  <arg name="initial_pose_y" default="17.0" />'
    echo '  <node pkg="semantic_turtle_test" type="timed_roslaunch.sh"'
    echo '    args="2 turtlebot_navigation amcl_demo.launch initial_pose_x:=17.0 initial_pose_y:=$(arg initial_pose_y)"'
    echo '    name="timed_roslaunch" output="screen">'
    echo '  </node>'
    echo '</launch>'
}

if [ "$1" = "-h" ]; then    #如果第一个参数为“-h”,则显示帮助信息
    showHelp
else
    echo "start wait for $1 seconds"
    sleep $1               #延时时长，由执行.sh时给入的第一个参数指定
    echo "end wait for $1 seconds"
    shift
        echo "now running 'roslaunch $@'"
    roslaunch $@          #调用指定的roslaunch文件，参数由执行.sh时去除第一个给入参数后剩下的所有参数指定
fi