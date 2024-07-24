#include "rl_controllers/RLControllerBase.h"
#include <string.h>
#include <pluginlib/class_list_macros.hpp>
#include "rl_controllers/RotationTools.h"
#include "rl_controllers/utilities.h"
#include <kdl_parser/kdl_parser.hpp>

namespace legged
{

  bool RLControllerBase::init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH)
  {
    // Hardware interface
    // std::vector<std::string> jointNames{"LF_HAA", "LF_HFE", "LF_KFE",
    //                                     "RF_HAA", "RF_HFE", "RF_KFE",
    //                                     "LH_HAA", "LH_HFE", "LH_KFE",
    //                                     "RH_HAA", "RH_HFE", "RH_KFE"};
    std::vector<std::string> jointNames{"LF_HAA", "LF_HFE", "LF_KFE",
                                        "RF_HAA", "RF_HFE", "RF_KFE",
                                        "LH_HAA", "LH_HFE", "LH_KFE",
                                        "RH_HAA", "RH_HFE", "RH_KFE"};
    std::vector<std::string> footNames{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    // std::vector<std::string> jointNames{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    //                                     "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    //                                     "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    //                                     "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    // std::vector<std::string> footNames{"FL_FOOT", "FR_FOOT", "RR_FOOT", "RL_FOOT"};
    actuatedDofNum_ = jointNames.size();

    // Load policy model and rl cfg
    if (!loadModel(controllerNH))
    {
      ROS_ERROR_STREAM("[RLControllerBase] Failed to load the model. Ensure the path is correct and accessible.");
      return false;
    }
    if (!loadRLCfg(controllerNH))
    {
      ROS_ERROR_STREAM("[RLControllerBase] Failed to load the rl config. Ensure the yaml is correct and accessible.");
      return false;
    }

    standJointAngles_.resize(actuatedDofNum_);
    lieJointAngles_.resize(actuatedDofNum_);
    auto &StandState = standjointState_;
    auto &LieState = liejointState_;

    joint_dim_ = actuatedDofNum_;
    // 加载kd
    controllerNH.getParam("/Kd_config/kd", cfg_kd);

    realJointPosPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("data_analysis/real_joint_pos", 1);
    realJointVelPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("data_analysis/real_joint_vel", 1);
    realTorquePublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("data_analysis/real_torque", 1);

    rlComputeTauPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("data_analysis/output_torque", 1);
    rlPosPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("data_analysis/rlPos", 1);

    urdf::Model urdfModel;
    if (!urdfModel.initParam("legged_robot_description"))
    {
      std::cerr << "[LeggedRobotVisualizer] Could not read URDF from: \"legged_robot_description\"" << std::endl;
    }
    else
    {
      KDL::Tree kdlTree;
      kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);
      robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    }

    // Stand Lie joint
    lieJointAngles_ << LieState.LF_HAA_joint, LieState.LF_HFE_joint, LieState.LF_KFE_joint,
                       LieState.RF_HAA_joint, LieState.RF_HFE_joint, LieState.RF_KFE_joint,
                       LieState.LH_HAA_joint, LieState.LH_HFE_joint, LieState.LH_KFE_joint,
                       LieState.RH_HAA_joint, LieState.RH_HFE_joint, LieState.RH_KFE_joint;

    standJointAngles_ << StandState.LF_HAA_joint, StandState.LF_HFE_joint, StandState.LF_KFE_joint,
                         StandState.RF_HAA_joint, StandState.RF_HFE_joint, StandState.RF_KFE_joint,
                         StandState.LH_HAA_joint, StandState.LH_HFE_joint, StandState.LH_KFE_joint,
                         StandState.RH_HAA_joint, StandState.RH_HFE_joint, StandState.RH_KFE_joint;

    auto *hybridJointInterface = robotHw->get<HybridJointInterface>();
    for (const auto &jointName : jointNames)
    {
      hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName));
    }

    imuSensorHandles_ = robotHw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

    auto *contactInterface = robotHw->get<ContactSensorInterface>();
    for (const auto &footName : footNames)
    {
      contactHandles_.push_back(contactInterface->getHandle(footName));
    }

    cmdVelSub_ = controllerNH.subscribe("/cmd_vel", 1, &RLControllerBase::cmdVelCallback, this);
    joyInfoSub_ = controllerNH.subscribe("/joy", 1000, &RLControllerBase::joyInfoCallback, this);
    switchCtrlClient_ = controllerNH.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    auto emergencyStopCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {emergency_stop = true;ROS_INFO("Emergency Stop"); };
    emgStopSub_ = controllerNH.subscribe<std_msgs::Float32>("/emergency_stop", 1, emergencyStopCallback);

    // start control
    auto startControlCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      ros::Duration t(0.5);
      if (ros::Time::now() - switchTime > t)
      {
        if (!start_control)
        {
          start_control = true;
          standPercent_ = 0;
          for (size_t i = 0; i < hybridJointHandles_.size(); i++)
          {
            currentJointAngles_[i] = hybridJointHandles_[i].getPosition();
          }
          mode_ = Mode::LIE;
          ROS_INFO("Start Control");
        }
        else
        {
          start_control = false;
          mode_ = Mode::DEFAULT;
          ROS_INFO("ShutDown Control");
        }
        switchTime = ros::Time::now();
      }
    };
    startCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/start_control", 1, startControlCallback);

    // switchMode
    auto switchModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      ros::Duration t(0.8);
      if (ros::Time::now() - switchTime > t)
      {
        if (start_control == true)
        {
          if (mode_ == Mode::STAND)
          {
            standPercent_ = 0;
            for (size_t i = 0; i < hybridJointHandles_.size(); i++)
            {
              currentJointAngles_[i] = hybridJointHandles_[i].getPosition();
            }
            mode_ = Mode::LIE;
            ROS_INFO("STAND2LIE");
          }
          else if (mode_ == Mode::LIE)
          {
            standPercent_ = 0;
            mode_ = Mode::STAND;
            ROS_INFO("LIE2STAND");
          }
        }
        switchTime = ros::Time::now();
      }
    };
    switchModeSub_ = controllerNH.subscribe<std_msgs::Float32>("/switch_mode", 1, switchModeCallback);

    // walkMode
    auto walkModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      ros::Duration t(0.2);
      if (ros::Time::now() - switchTime > t)
      {
        if (mode_ == Mode::STAND)
        {
          mode_ = Mode::WALK;
          ROS_INFO("STAND2WALK");
        }
        switchTime = ros::Time::now();
      }
    };
    walkModeSub_ = controllerNH.subscribe<std_msgs::Float32>("/walk_mode", 1, walkModeCallback);

    // positionMode
    auto positionModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      ros::Duration t(0.2);
      if (ros::Time::now() - switchTime > t)
      {
        if (mode_ == Mode::WALK)
        {
          mode_ = Mode::STAND;
          ROS_INFO("WALK2STAND");
        }
        else if (mode_ == Mode::DEFAULT)
        {
          standPercent_ = 0;
          for (size_t i = 0; i < hybridJointHandles_.size(); i++)
          {
            currentJointAngles_[i] = hybridJointHandles_[i].getPosition();
          }
          mode_ = Mode::LIE;
          ROS_INFO("DEF2LIE");
        }

        switchTime = ros::Time::now();
      }
    };
    positionCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/position_control", 1, positionModeCallback);

    // terrainMode
    auto terrainModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      ros::Duration t(0.2);
      if (ros::Time::now() - switchTime > t)
      {
        if (!terrain_control)
        {
          terrain_control = true;
          std::swap(mode_A_, mode_B_);
          std::cout << "mode_A_ and mode_B_ values switched. mode_A_ = " << mode_A_ << ", mode_B_ = " << mode_B_ << std::endl;
        }
        else
        {
          start_control = false;
          std::swap(mode_A_, mode_B_);
          std::cout << "mode_A_ and mode_B_ values switched. mode_A_ = " << mode_A_ << ", mode_B_ = " << mode_B_ << std::endl;
        }
        switchTime = ros::Time::now();
      }
    };
    terrainSub_ = controllerNH.subscribe<std_msgs::Float32>("/terrain_control", 1, terrainModeCallback);
    
    cv::namedWindow("iamge_processe", cv::WINDOW_AUTOSIZE);
    return true;
  }

  std::atomic<scalar_t> kp_stance{0};
  std::atomic<scalar_t> kd_stance{0};
  // only once
  void RLControllerBase::starting(const ros::Time &time)
  {
    updateStateEstimation(time, ros::Duration(0.002));
    currentJointAngles_.resize(hybridJointHandles_.size());
    scalar_t durationSecs = 2.0;
    standDuration_ = durationSecs * 500.0;
    standPercent_ = 0;
    mode_ = Mode::DEFAULT;
    loopCount_ = 0;

    server_ptr_ = std::make_unique<dynamic_reconfigure::Server<legged_debugger::TutorialsConfig>>(ros::NodeHandle("controller"));
    dynamic_reconfigure::Server<legged_debugger::TutorialsConfig>::CallbackType f;
    f = boost::bind(&RLControllerBase::dynamicParamCallback, this, _1, _2);
    server_ptr_->setCallback(f);

    pos_des_output_.resize(joint_dim_);
    vel_des_output_.resize(joint_dim_);
    pos_des_output_.setZero();
    vel_des_output_.setZero();

    pos_des_filtered_.resize(joint_dim_);
    pos_des_filtered_.setZero();
    vel_des_filtered_.resize(joint_dim_);
    vel_des_filtered_.setZero();
  }

  void RLControllerBase::update(const ros::Time &time, const ros::Duration &period)
  {
    updateStateEstimation(time, period);
    // ROS_WARN("mode: %d", mode_);
    switch (mode_)
    {
    case Mode::DEFAULT:
      handleDefautMode();
      break;
    case Mode::LIE:
      handleLieMode();
      break;
    case Mode::STAND:
      handleStandMode();
      break;
    case Mode::WALK:
      handleWalkMode();
      break;
    default:
      ROS_ERROR_STREAM("Unexpected mode encountered: " << static_cast<int>(mode_));
      break;
    }
    // ROS_INFO("mode::BASE%d", mode_);
    if (emergency_stop)
    {
      emergency_stop = false;
      mode_ = Mode::DEFAULT;
    }

    loopCount_++;
  }

  void RLControllerBase::handleDefautMode()
  {
    for (int j = 0; j < hybridJointHandles_.size(); j++)
      hybridJointHandles_[j].setCommand(0, 0, 0, cfg_kd, 0);

    // ROS_WARN("The value of kdConfig.cfg_kd is: %f", kdConfig.cfg_kd);
  }

  void RLControllerBase::handleLieMode()
  {
    if (standPercent_ <= 1)
    {
      for (int j = 0; j < hybridJointHandles_.size(); j++)
      {
        scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + lieJointAngles_[j] * standPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 50, 3, 0);

        // pos_des = currentJointAngles_[j] * (1 - standPercent_) + lieJointAngles_[j] * standPercent_;
        // hybridJointHandles_[j].setCommand(pos_des, 0, 350, 25, 0);
      }
      standPercent_ += 1 / standDuration_;
      standPercent_ = std::min(standPercent_, scalar_t(1));
    }
  }

  void RLControllerBase::handleStandMode()
  {
    if (standPercent_ <= 1)
    {
      for (int j = 0; j < hybridJointHandles_.size(); j++)
      {
        // scalar_t pos_des = lieJointAngles_[j] * (1 - standPercent_) + standJointAngles_[j] * standPercent_;
        // hybridJointHandles_[j].setCommand(pos_des, 0, 350, 30, 0);
        // //
        scalar_t pos_des = lieJointAngles_[j] * (1 - standPercent_) + standJointAngles_[j] * standPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 50, 3, 0);
      }
      standPercent_ += 1 / standDuration_;
      standPercent_ = std::min(standPercent_, scalar_t(1));
    }
  }

  void RLControllerBase::updateStateEstimation(const ros::Time &time, const ros::Duration &period)
  {
    vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()), jointTor(hybridJointHandles_.size());
    contact_flag_t contacts;
    quaternion_t quat;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
    for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
    {
      jointPos(i) = hybridJointHandles_[i].getPosition();
      jointVel(i) = hybridJointHandles_[i].getVelocity();
      jointTor(i) = hybridJointHandles_[i].getEffort();
    }
    for (size_t i = 0; i < 4; ++i)
    {
      quat.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
      angularVel(i) = imuSensorHandles_.getAngularVelocity()[i];
      linearAccel(i) = imuSensorHandles_.getLinearAcceleration()[i];
    }
    for (size_t i = 0; i < 9; ++i)
    {
      orientationCovariance(i) = imuSensorHandles_.getOrientationCovariance()[i];
      angularVelCovariance(i) = imuSensorHandles_.getAngularVelocityCovariance()[i];
      linearAccelCovariance(i) = imuSensorHandles_.getLinearAccelerationCovariance()[i];
    }

    propri_.jointPos = jointPos;
    propri_.jointVel = jointVel;
    propri_.baseAngVel = angularVel;

    vector3_t gravityVector(0, 0, -1);
    vector3_t zyx = quatToZyx(quat);
    matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
    propri_.projectedGravity = inverseRot * gravityVector;
    propri_.baseEulerXyz = quatToXyz(quat);

    // if (zyx(2) > M_PI_2 || zyx(2) < -M_PI_2)
    // {
    //   if (!emergency_stop && mode_ == Mode::WALK)
    //   {
    //     emergency_stop = true;
    //     ROS_ERROR("FALL DOWN!!!!!!!!!");
    //   }
    // }
    realTorquePublisher_.publish(createFloat64MultiArrayFromVector(jointTor));
    realJointPosPublisher_.publish(createFloat64MultiArrayFromVector(jointPos));
    realJointVelPublisher_.publish(createFloat64MultiArrayFromVector(jointVel));

    // robotStatePublisherPtr_->publishFixedTransforms(true);
    // tf::Transform baseTransform;
    // baseTransform.setOrigin(tf::Vector3(0.0, 0.0, 0.0)); // Origin
    // baseTransform.setRotation(tf::Quaternion(quat.coeffs()(0), quat.coeffs()(1), quat.coeffs()(2), quat.coeffs()(3)));
    // tfBroadcaster_.sendTransform(tf::StampedTransform(baseTransform, time, "world", "base"));

    // std::map<std::string, scalar_t> jointPositions{
    //     {"LF_HAA", jointPos(0)}, {"LF_HFE", jointPos(1)}, {"LF_KFE", jointPos(2)}, {"RF_HAA", jointPos(3)}, {"RF_HFE", jointPos(4)}, {"RF_KFE", jointPos(5)}, {"LH_HAA", jointPos(6)}, {"LH_HFE", jointPos(7)}, {"LH_KFE", jointPos(8)}, {"RH_HAA", jointPos(9)}, {"RH_HFE", jointPos(10)}, {"RH_KFE", jointPos(11)}};
    // robotStatePublisherPtr_->publishTransforms(jointPositions, time);
  }

  void RLControllerBase::cmdVelCallback(const geometry_msgs::Twist &msg)
  {
    command_.x = msg.linear.x;
    command_.y = msg.linear.y;
    command_.yaw = msg.angular.z;
  }

  void RLControllerBase::dynamicParamCallback(legged_debugger::TutorialsConfig &config, uint32_t level)
  {
    kp_stance = config.kp_stance;
    kd_stance = config.kd_stance;
  }

  void RLControllerBase::joyInfoCallback(const sensor_msgs::Joy &msg)
  {
    if (msg.header.frame_id.empty())
    {
      return;
    }
    // memcpy(joyInfo.axes, msg.axes, sizeof(joyInfo.axes));
    // memcpy(joyInfo.buttons, msg.buttons, sizeof(joyInfo.buttons));
    for (int i = 0; i < msg.axes.size(); i++)
    {
      joyInfo.axes[i] = msg.axes[i];
      // std::cout << joyInfo.axes[i];
      // std::cout << std::endl;
    }
    for (int i = 0; i < msg.buttons.size(); i++)
    {
      joyInfo.buttons[i] = msg.buttons[i];
      // std::cout << joyInfo.buttons[i];
      // std::cout << std::endl;
    }
  }
} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::RLControllerBase, controller_interface::ControllerBase)