#pragma once

#include "rl_controllers/Types.h"
#include <robot_state_publisher/robot_state_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <gazebo_msgs/ModelStates.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/Joy.h>

#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <mutex>

#include "TutorialsConfig.h"
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/ParamDescription.h>

#include <atomic>

// 定义DEBUG宏以启用调试输出
#ifdef TIMEPRINT
    #define DEBUG_PRINT(x) std::cout << x << std::endl
    #define DEBUG_TIMER_START(timer) timer.start()
    #define DEBUG_TIMER_DURATION(timer, message) \
        timer.stop(); \
        std::cout << message << " took " << timer.elapsedMilliseconds() << " milliseconds." << std::endl; \
        timer.start()
#else
    #define DEBUG_PRINT(x) do {} while (0)
    #define DEBUG_TIMER_START(timer) do {} while (0)
    #define DEBUG_TIMER_DURATION(timer, message) do {} while (0)
#endif

class Timer {
public:
    void start() {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    void stop() {
        end_time_ = std::chrono::high_resolution_clock::now();
    }

    double elapsedMilliseconds() const {
        std::chrono::duration<double, std::milli> duration = end_time_ - start_time_;
        return duration.count();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time_;
};

namespace legged
{

  struct RLRobotCfg
  {
    struct ControlCfg
    {
      float stiffness;
      float damping;
      float actionScale;
      int decimation;
      float user_torque_limit;
      float user_power_limit;
    };

    struct InitState
    {
      // default joint angles
      scalar_t LF_HAA_joint;
      scalar_t LF_HFE_joint;
      scalar_t LF_KFE_joint;

      scalar_t LH_HAA_joint;
      scalar_t LH_HFE_joint;
      scalar_t LH_KFE_joint;

      scalar_t RF_HAA_joint;
      scalar_t RF_HFE_joint;
      scalar_t RF_KFE_joint;

      scalar_t RH_HAA_joint;
      scalar_t RH_HFE_joint;
      scalar_t RH_KFE_joint;
    };

    struct ObsScales
    {
      scalar_t linVel;
      scalar_t angVel;
      scalar_t dofPos;
      scalar_t dofVel;
      scalar_t heightMeasurements;
    };

    bool encoder_nomalize;

    scalar_t clipActions;
    scalar_t clipObs;

    InitState initState;
    ObsScales obsScales;
    ControlCfg controlCfg;
  };

  struct JointState
  {
    scalar_t LF_HAA_joint;
    scalar_t LF_HFE_joint;
    scalar_t LF_KFE_joint;

    scalar_t LH_HAA_joint;
    scalar_t LH_HFE_joint;
    scalar_t LH_KFE_joint;

    scalar_t RF_HAA_joint;
    scalar_t RF_HFE_joint;
    scalar_t RF_KFE_joint;

    scalar_t RH_HAA_joint;
    scalar_t RH_HFE_joint;
    scalar_t RH_KFE_joint;
  };

  struct JoyInfo
  {
    float axes[8];
    int buttons[11];
  };

  struct Proprioception
  {
    vector_t jointPos;
    vector_t jointVel;
    vector3_t baseAngVel;
    vector3_t baseLinearVel;
    vector3_t baseEulerXyz;
    // vector3_t baseAngZyx;  // base angular pos eular (zyx)
    // quaternion_t baseAngQuat;  // base angular pos quat (zyx)
    vector3_t projectedGravity;
  };

  struct Command
  {
    std::atomic<scalar_t> x;
    std::atomic<scalar_t> y;
    std::atomic<scalar_t> yaw;
  };

  class RLControllerBase : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                                 ContactSensorInterface>
  {
  public:
    enum class Mode : uint8_t
    {
      LIE,
      STAND,
      WALK,
      DEFAULT
    };

    RLControllerBase() = default;
    virtual ~RLControllerBase() = default;
    virtual bool init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH);
    virtual void starting(const ros::Time &time);
    virtual void update(const ros::Time &time, const ros::Duration &period);

    virtual bool loadModel(ros::NodeHandle &nh) { return false; };
    virtual bool loadRLCfg(ros::NodeHandle &nh) { return false; };
    virtual void computeActions(){};
    virtual void computeObservation(){};

    virtual void handleLieMode();
    virtual void handleStandMode();
    virtual void handleDefautMode();
    virtual void handleWalkMode(){};

    std::unique_ptr<dynamic_reconfigure::Server<legged_debugger::TutorialsConfig>> server_ptr_;
    void dynamicParamCallback(legged_debugger::TutorialsConfig &config, uint32_t level);

  protected:
    virtual void updateStateEstimation(const ros::Time &time, const ros::Duration &period);

    virtual void cmdVelCallback(const geometry_msgs::Twist &msg);
    virtual void joyInfoCallback(const sensor_msgs::Joy &msg);

    float mode_A_ = 0;
    float mode_B_ = 1;
    // float mode_A_ = 1;
    // float mode_B_ = 0;
    quaternion_t quat;
    Mode mode_;
    int64_t loopCount_;
    Command command_;
    RLRobotCfg robotCfg_{};
    JointState standjointState_{0.0, 0.9, -1.80,
                                0.0, 0.9, -1.80,
                                0.0, 0.9, -1.80,
                                0.0, 0.9, -1.80};

    JointState liejointState_{0.0, 1.40, -2.60,
                              0.0, 1.40, -2.60,
                              0.0, 1.40, -2.60,
                              0.0, 1.40, -2.60};
    JoyInfo joyInfo;
    std::atomic_bool emergency_stop{false};
    std::atomic_bool start_control{false};
    std::atomic_bool position_control{false};
    std::atomic_bool terrain_control{false};
    ros::Time switchTime;

    vector_t rbdState_;
    vector_t measuredRbdState_;
    Proprioception propri_;

    // hardware interface
    std::vector<HybridJointHandle> hybridJointHandles_;
    hardware_interface::ImuSensorHandle imuSensorHandles_;
    std::vector<ContactSensorHandle> contactHandles_;

    ros::Subscriber cmdVelSub_;
    ros::Subscriber joyInfoSub_;
    ros::Subscriber emgStopSub_;
    ros::Subscriber startCtrlSub_;
    ros::Subscriber switchModeSub_;
    ros::Subscriber walkModeSub_;
    ros::Subscriber positionCtrlSub_;
    ros::Subscriber terrainSub_;
    ros::Subscriber contactSub_;
    controller_manager_msgs::SwitchController switchCtrlSrv_;
    ros::ServiceClient switchCtrlClient_;

    int actuatedDofNum_ = 12;

    ros::Publisher realJointVelPublisher_;
    ros::Publisher realJointPosPublisher_;
    ros::Publisher realTorquePublisher_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

    ros::Publisher rlPosPublisher_;
    ros::Publisher rlComputeTauPublisher_;

    int walkCount_ = 0;
    float cfg_kd;
    vector_t rlPos_;
    rs2::config image_config;

    std::mutex mtx;

  private:
    // PD stand

    vector_t pos_des_output_{};
    vector_t vel_des_output_{};

    vector_t pos_des_filtered_{};
    vector_t vel_des_filtered_{};
    size_t joint_dim_{0};

    std::vector<scalar_t> currentJointAngles_;
    vector_t standJointAngles_;
    vector_t lieJointAngles_;

    scalar_t standPercent_;
    scalar_t standDuration_;
    tf::TransformBroadcaster tfBroadcaster_;
  };
} // namespace legged