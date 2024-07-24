#include "rl_controllers/ParkourController.h"
#include <pluginlib/class_list_macros.hpp>
#include "rl_controllers/RotationTools.h"

#include <typeinfo>
#include <type_traits>
#include <typeindex>

namespace legged
{
    bool ParkourController::init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH)
    {
        RLControllerBase::init(robotHw, controllerNH);

        // latent sub
        auto latentCallback = [this](const std_msgs::Float32MultiArray::ConstPtr &msg)
        {
            std::lock_guard<std::mutex> guard(depth_mutex_);
            depth_latent_ = msg->data;
        };
        latent_sub_ = controllerNH.subscribe<std_msgs::Float32MultiArray>("/latent_topic", 1, latentCallback);
        obs_pub_ = controllerNH.advertise<control_msg::TimestampedFloat32MultiArray>("/obs_info", 1);

        ROS_INFO("parkour init");
        return true;
    }

    void ParkourController::handleWalkMode()
    {
        // compute observation & actions
        if (loopCount_ % robotCfg_.controlCfg.decimation == 0)
        {
            // Timer timer;
            // DEBUG_TIMER_START(timer);
            updateContact();
            updateYawAndDepthLatent();
            computeObservation();
            pubObs();
            computeActions();

            // limit action range
            scalar_t actionMin = -robotCfg_.clipActions;
            scalar_t actionMax = robotCfg_.clipActions;
            std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                           [actionMin, actionMax](scalar_t x)
                           { return std::max(actionMin, std::min(actionMax, x)); });
            // DEBUG_TIMER_DURATION(timer, "ParkourController handleWalkMode");
        }

        // // set action
        for (int i = 0; i < hybridJointHandles_.size(); i++)
        {
            scalar_t pos_des = actions_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(i, 0);
            hybridJointHandles_[i].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness, robotCfg_.controlCfg.damping, 0);
            lastActions_[i] = actions_[i];
            // std::cout << "action:" << i << "::" << actions_[i] << std::endl;
        }
        // ROS_WARN("1:%f",propri_.projectedGravity[0]);

        // ROS_WARN("2:%f",propri_.projectedGravity[1]);
        // ROS_WARN("3:%f",propri_.projectedGravity[2]);

    }

    bool ParkourController::loadRLCfg(ros::NodeHandle &nh)
    {
        auto &initState = robotCfg_.initState;
        auto &controlCfg = robotCfg_.controlCfg;
        auto &obsScales = robotCfg_.obsScales;

        int error = 0;
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_HAA_joint", initState.RF_HAA_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_HFE_joint", initState.RF_HFE_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_KFE_joint", initState.RF_KFE_joint));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_HAA_joint", initState.LF_HAA_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_HFE_joint", initState.LF_HFE_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_KFE_joint", initState.LF_KFE_joint));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_HAA_joint", initState.RH_HAA_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_HFE_joint", initState.RH_HFE_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_KFE_joint", initState.RH_KFE_joint));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_HAA_joint", initState.LH_HAA_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_HFE_joint", initState.LH_HFE_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_KFE_joint", initState.LH_KFE_joint));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness", controlCfg.stiffness));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping", controlCfg.damping));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/action_scale", controlCfg.actionScale));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/decimation", controlCfg.decimation));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robotCfg_.clipObs));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robotCfg_.clipActions));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size", actionsSize_));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size", observationSize_));

        // depth_actor
        privLatent_.resize(29);
        privLatent_ << 5.204, 0, 0, 0.24, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0;
        proprioHistoryBuffer_.resize(450);
        proprioHistoryBuffer_.setZero();

        num_priv_explicit_.resize(9);
        num_priv_explicit_.setZero();
        num_scan_.resize(132);
        num_scan_.setZero();

        contact_.resize(4);
        contact_.setZero();
        // 1
        actions_.resize(actionsSize_);
        observations_.resize(45);
        depth_latent_.assign(32, 0);

        command_.x = 0;
        command_.y = 0;
        command_.yaw = 0;
        baseLinVel_.setZero();
        basePosition_.setZero();
        deltaYaw_ = 0.0;
        deltaNextYaw_ = 0.0;
        std::vector<scalar_t> defaultJointAngles{
            robotCfg_.initState.LF_HAA_joint, robotCfg_.initState.LF_HFE_joint, robotCfg_.initState.LF_KFE_joint,
            robotCfg_.initState.RF_HAA_joint, robotCfg_.initState.RF_HFE_joint, robotCfg_.initState.RF_KFE_joint,
            robotCfg_.initState.LH_HAA_joint, robotCfg_.initState.LH_HFE_joint, robotCfg_.initState.LH_KFE_joint,
            robotCfg_.initState.RH_HAA_joint, robotCfg_.initState.RH_HFE_joint, robotCfg_.initState.RH_KFE_joint};
        lastActions_.resize(actuatedDofNum_);
        lastActions_.setZero();
        defaultJointAngles_.resize(actuatedDofNum_);
        for (int i = 0; i < actuatedDofNum_; i++)
        {
            defaultJointAngles_(i) = defaultJointAngles[i];
        }
        robotCfg_.clipActions = robotCfg_.clipActions / robotCfg_.controlCfg.actionScale;

        ROS_INFO("PAOKU config");
        return (error == 0);
    }

    void ParkourController::updateContact()
    {
        for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
        {
            contact_state_[i] = contactHandles_[i].isContact();
            contact_[i] = contact_state_[i];
            contact_[i] -= 0.5;
            // ROS_INFO("CONTACT_NUM%d: %f", i, contact_[i]);
        }
    }

    /// @brief 这个函数需要在 `computeObservation()` 和 `computeActions()` 之前调用，
    /// 因为这个函数更新了它们需要的 `deltaYaw_`，`deltaNextYaw_` 和 `input32latent_` 这两个变量。
    void ParkourController::updateYawAndDepthLatent()
    {
        auto latentTensor = new MNN::Tensor(input32latent_, MNN::Tensor::CAFFE);
        auto locked = depth_mutex_.try_lock();
        if (locked)
        {
            // deltaYaw_ = depth_latent_[32];
            // deltaNextYaw_ = depth_latent_[33];
            deltaYaw_ = 0;
            deltaNextYaw_ = 0;

            for (size_t i = 0; i < latentTensor->elementSize(); ++i)
            {
                latentTensor->host<float>()[i] = depth_latent_[i];
                // latentTensor->host<float>()[i] = 0;
            }
            depth_mutex_.unlock();
        }
        if (locked)
            input32latent_->copyFromHostTensor(latentTensor);
        delete latentTensor;
    }

    void ParkourController::initMNNTensorWithZero(MNN::Tensor *mnn_tensor)
    {
        auto tmp_tensor = new MNN::Tensor(mnn_tensor, MNN::Tensor::CAFFE);
        for (size_t i = 0; i < tmp_tensor->elementSize(); ++i)
        {
            tmp_tensor->host<float>()[i] = 0;
        }
        mnn_tensor->copyFromHostTensor(tmp_tensor);
        delete tmp_tensor;
    }

    bool ParkourController::loadModel(ros::NodeHandle &nh)
    {
        std::string policyModelPath;
        if (!nh.getParam("/policyModelPath", policyModelPath))
        {
            ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
            return false;
        }
        ROS_INFO_STREAM("Load actor model from path : " << policyModelPath);

        // Create the MNN interpreter instance
        interpreterPolicy_ = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(policyModelPath.c_str()));
        if (!interpreterPolicy_)
        {
            ROS_ERROR_STREAM("Failed to create actor interpreter!");
            return false;
        }

        // Create session
        MNN::ScheduleConfig Policyconfig;
        MNN::BackendConfig backendConfig;
        backendConfig.precision = MNN::BackendConfig::Precision_High;
        Policyconfig.type = MNN_FORWARD_CPU;
        Policyconfig.backendConfig;
        Policysession_ = interpreterPolicy_->createSession(Policyconfig);

        // detect session
        if (!Policysession_)
        {
            ROS_ERROR_STREAM("Failed to create actor session!");
            return false;
        }

        // Get input info
        inputHistorybuf_ = interpreterPolicy_->getSessionInput(Policysession_, "obs_buf");
        input32latent_ = interpreterPolicy_->getSessionInput(Policysession_, "depth_latent");

        initMNNTensorWithZero(inputHistorybuf_);
        initMNNTensorWithZero(input32latent_);

        ROS_INFO_STREAM("actor model loaded successfully!");
        return true;
    }

    void ParkourController::computeActions()
    {
        interpreterPolicy_->runSession(Policysession_);

        // Get output info
        auto outputAction_ = interpreterPolicy_->getSessionOutput(Policysession_, "output_policy");

        MNN::Tensor outputTensorCopy(outputAction_, outputAction_->getDimensionType());
        outputAction_->copyToHostTensor(&outputTensorCopy);
        for (int i = 0; i < outputTensorCopy.elementSize(); ++i)
        {
            actions_[i] = outputTensorCopy.host<float>()[i];
        }
    }

    /// @brief update inputHistorybuf_
    void ParkourController::computeObservation()
    {
        // command
        vector3_t command{command_.x, 0, command_.yaw};
        // actions
        vector_t actions(lastActions_);

        auto &obsScales = robotCfg_.obsScales;

        vector_t proprioObs(45);
        proprioObs << propri_.baseAngVel * obsScales.angVel,             // 3
            propri_.projectedGravity, 
            // 0 ,
            // 0 ,
            // 0 ,                                  // 3                               
            command[0] * 2.0 ,                                               // 3
            command[1] * 2.0 ,                                               // 3
            command[2] * 0.25 ,                                               // 3
            (propri_.jointPos - defaultJointAngles_) * obsScales.dofPos, // 12
            propri_.jointVel * obsScales.dofVel,                         // 12
            actions;


        


        // for (int i = 0; i < 53; i++)
        // {
        //     std::cout << "第" << i << "个obs==" << proprioObs[i] << std::endl;
        // }

        proprioObs.cwiseMax(-robotCfg_.clipObs).cwiseMin(robotCfg_.clipObs);

        vector_t proprioObsHis(proprioObs);
        proprioObsHis[6] = 0;
        proprioObsHis[7] = 0;

        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - 45) =
        proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - 45);
        proprioHistoryBuffer_.tail(45) = proprioObsHis.cast<double>();

        vector_t historyObservations_(665);
        historyObservations_ << proprioObs,
            num_scan_,
            num_priv_explicit_,
            privLatent_,
            proprioHistoryBuffer_;

        auto historyTensor = new MNN::Tensor(inputHistorybuf_, MNN::Tensor::CAFFE);
        for (size_t i = 0; i < historyObservations_.size(); i++)
        {
            historyTensor->host<float>()[i] = historyObservations_[i];
            // std::cout << "input_ten NUM" << i << "=" << historyTensor->host<float>()[i] << std::endl;
        }
        inputHistorybuf_->copyFromHostTensor(historyTensor);
        delete historyTensor;
    }

    void ParkourController::pubObs()
    {
        control_msg::TimestampedFloat32MultiArray obs_msg;
        obs_msg.header.stamp = ros::Time::now();
        obs_msg.data.data.resize(45);
        for (int j = 0; j <45; ++j)
        {
            obs_msg.data.data[j] = proprioHistoryBuffer_[405 + j];
        }
        obs_pub_.publish(obs_msg);
    }
} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::ParkourController, controller_interface::ControllerBase)