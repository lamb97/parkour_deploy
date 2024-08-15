# legged_rl 强化学习部署框架使用方法

更新时间：2024年8月9日

## 依赖

安装 onnx-runtime，好像需要从源码编译。[参考教程](https://f0exxg5fp6u.feishu.cn/docx/BtH6d3SzzonXizxabTTcLbgjnAe)

## 编译

```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## 运行

```
# 启动仿真
export ROBOT_TYPE=go1
roslaunch legged_unitree_description empty_world.launch

# 加载并启动控制器
roslaunch rl_controllers load_amp_controller.launch
```

然后打开 rqt 寻找 robot_steering 插件，发布速度指令，即可（可能也需要施加外力把机器人位置摆正，现在初始化位置会有问题）。

[参考视频见（飞书视频，需要申请权限）](https://f0exxg5fp6u.feishu.cn/wiki/H2IWwAdvyikR1jkZ4YpcG1cxnWf)

## 将 Jit 导出为 Onnx

使用 `src/scripts/export_jit_to_onnx.py` 文件
