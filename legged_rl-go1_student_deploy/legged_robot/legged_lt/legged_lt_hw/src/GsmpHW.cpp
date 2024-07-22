#include "legged_gsmp_hw/GsmpHW.h"

#include <signal.h>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>

#include <bits/stdc++.h>

namespace legged
{
  bool GsmpHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {

    // IMU
    odom_sub_ = root_nh.subscribe("/imu/data", 1, &GsmpHW::OdomCallBack, this);

    int ec_slavecount = EtherCAT_Init("enp4s0");
    if (ec_slavecount <= 0)
    {
      std::cout << "未找到从站，程序退出" << std::endl;
      return false;
    }
    else
    {
      std::cout << "从站数量: " << ec_slavecount << std::endl;
    }
    std::cout << "EtherCAT_Init finished!" << std::endl;
    // EtherCAT_Send_Command((YKSMotorData *)yks_send_cmd_zero);
    std::cout << "EtherCAT Sended" << std::endl;

    // EtherCAT_Get_State();
    std::cout << "EtherCAT received" << std::endl;

    if (!LeggedHW::init(root_nh, robot_hw_nh))
    {
      printf("flase_bc_leggedHW::init\n");
      return false;
    }
    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);
    enable_CAN1_MOTOR();
    enable_CAN2_MOTOR();
    return true;
  }

  void GsmpHW::read(const ros::Time &time, const ros::Duration &period)
  {
    EtherCAT_Get_State();
    for (int i = 0; i < 12; i++)
    {
      // std::cout << "joint position:" << std::endl;
      // for (int i = 0; i < 12; i++)
      // {
      //   std::cout << fixed << setprecision(6) << joint_data_[i].pos_ << ",";
      //   if ((i + 1) % 3 == 0)
      //     std::cout << std::endl;
      // }

      joint_data_[i].pos_ = (motorDate_recv[i].pos_ - bias_motor[i]) * direction_motor[i];
      joint_data_[i].vel_ = motorDate_recv[i].vel_ * direction_motor[i];
      joint_data_[i].tau_ = motorDate_recv[i].tau_ * direction_motor[i];
    }

    // 元生艾欸姆尤
    imu_data_.ori[0] = yesenceIMU_.orientation.x; // 姿态
    imu_data_.ori[1] = yesenceIMU_.orientation.y;
    imu_data_.ori[2] = yesenceIMU_.orientation.z;
    imu_data_.ori[3] = yesenceIMU_.orientation.w;
    imu_data_.angular_vel[0] = yesenceIMU_.angular_velocity.x; // 角速度
    imu_data_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
    imu_data_.angular_vel[2] = yesenceIMU_.angular_velocity.z;
    imu_data_.linear_acc[0] = yesenceIMU_.linear_acceleration.x; // 线性加速度
    imu_data_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
    imu_data_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;
    // 元生艾欸姆尤

    // printf("\nimu 1 quat x: %f, y: %f, z: %f, w: %f\n", imu_data_.ori[0],
    //        imu_data_.ori[1],
    //        imu_data_.ori[2],
    //        imu_data_.ori[3]);
    // printf("imu 1 ang x: %f, y: %f, z: %f\n", imu_data_.angular_vel[0],
    //        imu_data_.angular_vel[1],
    //        imu_data_.angular_vel[2]);
    // printf("imu 1 acc x: %f, y: %f, z: %f\n", imu_data_.linear_acc[0],
    //        imu_data_.linear_acc[1],
    //        imu_data_.linear_acc[2]);

    // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto &name : names)
    {
      HybridJointHandle handle = hybridJointInterface_.getHandle(name);
      handle.setFeedforward(0.);
      handle.setVelocityDesired(0.);
      handle.setKd(3.1415);
      handle.setKp(0.);
    }
  }

  void GsmpHW::write(const ros::Time &time, const ros::Duration &period)
  {

    for (int i = 0; i < 12; ++i)
    {

      yks_send_cmd[i].pos_des_ = joint_data_[i].pos_des_ * direction_motor[i] + bias_motor[i];
      yks_send_cmd[i].vel_des_ = joint_data_[i].vel_des_ * direction_motor[i];

      yks_send_cmd[i].kp_ = joint_data_[i].kp_;
      yks_send_cmd[i].kd_ = joint_data_[i].kd_;
      yks_send_cmd[i].ff_ = joint_data_[i].ff_ * direction_motor[i];
    }

    EtherCAT_Send_Command((YKSMotorData *)yks_send_cmd);
  }

  bool GsmpHW::setupJoints()
  {
    for (const auto &joint : urdfModel_->joints_)
    {
      int leg_index, joint_index;
      if (joint.first.find("LF") != std::string::npos)
      {
        leg_index = 0;
        // leg_index = UNITREE_LEGGED_SDK::FR_;
      }
      else if (joint.first.find("LH") != std::string::npos)
      {
        leg_index = 1;
        // leg_index = UNITREE_LEGGED_SDK::FL_;
      }
      else if (joint.first.find("RF") != std::string::npos)
      {
        leg_index = 2;
        // leg_index = UNITREE_LEGGED_SDK::RR_;
      }
      else if (joint.first.find("RH") != std::string::npos)
      {
        leg_index = 3;
        // leg_index = UNITREE_LEGGED_SDK::RL_;
      }
      else
        continue;
      if (joint.first.find("HAA") != std::string::npos)
        joint_index = 0;
      else if (joint.first.find("HFE") != std::string::npos)
        joint_index = 1;
      else if (joint.first.find("KFE") != std::string::npos)
        joint_index = 2;
      else
        continue;

      int index = leg_index * 3 + joint_index;
      hardware_interface::JointStateHandle state_handle(joint.first, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                        &joint_data_[index].tau_);
      jointStateInterface_.registerHandle(state_handle);
      hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                             &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                             &joint_data_[index].kd_, &joint_data_[index].ff_));
    }
    return true;
  }

  bool GsmpHW::setupImu()
  {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
        "base_imu", "base_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
        imu_data_.linear_acc, imu_data_.linear_acc_cov));
    imu_data_.ori_cov[0] = 0.0012;
    imu_data_.ori_cov[4] = 0.0012;
    imu_data_.ori_cov[8] = 0.0012;

    imu_data_.angular_vel_cov[0] = 0.0004;
    imu_data_.angular_vel_cov[4] = 0.0004;
    imu_data_.angular_vel_cov[8] = 0.0004;

    return true;
  }

  bool GsmpHW::setupContactSensor(ros::NodeHandle &nh)
  {
    nh.getParam("contact_threshold", contact_threshold_);
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
      contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contact_state_[i]));
    return true;
  }
} // namespace legged
