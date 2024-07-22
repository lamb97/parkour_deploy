
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include "gsmp_legged_sdk/udp.h"
#include "gsmp_legged_sdk/safety.h"
#include <iostream>
#include <fstream>
#include <legged_hw/LeggedHW.h>

#include <memory.h>
// #include"udp_msg.h"
// #include "./unitree_motor.h"
// #include "./udp.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
#include "Console.hpp"
#include "command.h"
#include "transmit.h"

// add
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <controller_manager_msgs/SwitchController.h>

namespace legged
{
  const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

  struct GsmpMotorData
  {
    double pos_, vel_, tau_;                  // state
    double pos_des_, vel_des_, kp_, kd_, ff_; // command
  };

  struct GsmpImuData
  {
    double ori[4];
    double ori_cov[9];
    double angular_vel[3];
    double angular_vel_cov[9];
    double linear_acc[3];
    double linear_acc_cov[9];
  };

  class GsmpHW : public LeggedHW
  {
  public:
    GsmpHW()
    {
      std::cout << "poweron" << std::endl;
    }
    ~GsmpHW()
    {
      disable_CAN1_MOTOR();
      disable_CAN2_MOTOR();
      std::cout << "poweroff" << std::endl;
    }

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    void read(const ros::Time &time, const ros::Duration &period) override;
    void write(const ros::Time &time, const ros::Duration &period) override;

  private:
    bool setupJoints();

    bool setupImu();
    bool setupContactSensor(ros::NodeHandle &nh);

    GsmpMotorData joint_data_[12]{};
    GsmpImuData imu_data_{};
    bool contact_state_[4]{};
    int contact_threshold_{};

    ros::Subscriber odom_sub_;

    sensor_msgs::Imu yesenceIMU_;

    YKSMotorData yks_send_cmd_zero[12] = {};
    YKSMotorData yks_send_cmd[12];

    void OdomCallBack(const sensor_msgs::Imu::ConstPtr &odom)
    {
      yesenceIMU_ = *odom;
    }

    const vector<int> direction_motor{-1, -1, -1,
                                       1, -1, -1,
                                      -1,  1,  1,
                                       1,  1,  1};
    const vector<int> dircetion_limit{-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1};

    float bias_motor[12] = { 0.027657 , 0.081826 + 3.1415926 + 0.041 - 0.03,-2.6 + 0.252728 - 0.519 + 0.50454,
                             0.100901+0.09, 0.071145 + 3.1415926 + 0.04 + 0.02-0.06,-2.6 + 0.100138 + 0.0446-0.08,
                             0.153543,-0.102427 - 3.1415926 - 0.02 + 0.02 -0.02,2.6 + 0.161555 + 0.02 + 0.028,
                             0.056268, 0.022697 - 3.1415926 - 0.28 + 0.05 +0.09-0.03,2.6 + 0.478180 + 0.001};

  };

} // namespace legged