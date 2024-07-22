//
// Created by luohx on 23-8-23.
//

#pragma once

#include "rl_controllers/RLControllerBase.h"
#include "control_msg/TimestampedFloat32MultiArray.h"
#include <MNN/ImageProcess.hpp>

namespace legged
{
  const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};
  class ParkourController : public RLControllerBase
  {
    using tensor_element_t = float;

  public:
    ParkourController() = default;
    ~ParkourController() override = default;

  protected:
    virtual bool init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH) override;
    bool loadModel(ros::NodeHandle &nh) override;
    bool loadRLCfg(ros::NodeHandle &nh) override;
    void computeObservation() override;
    void handleWalkMode() override;
    void computeActions() override;
    void updateContact();
    void updateYawAndDepthLatent();
    void initMNNTensorWithZero(MNN::Tensor *mnn_tensor);
    void pubObs();

  private:
    bool contact_state_[4]{};
    int contact_threshold_{};

    vector_t contact_; // 存储每只脚的接触力

    vector3_t baseLinVel_;
    vector3_t basePosition_;
    vector_t zyx;
    vector_t lastActions_;
    vector_t defaultJointAngles_;

    int is2RecObs_ = 0;
    int actionsSize_;
    int observationSize_;
    std::vector<tensor_element_t> hiddenStateData_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> observations_;
    std::vector<tensor_element_t> propri_in_;
    std::vector<tensor_element_t> depthLatent_;
    std::vector<std::vector<double>> imu_obs;
    ContactSensorInterface contactSensorInterface_;
    // vector_t historyObservations_;

    // image

    std::shared_ptr<MNN::Interpreter> interpreterImage_;
    MNN::Session *Imagesession_;
    MNN::Tensor *inputImage_;
    MNN::Tensor *inputObs_;
    MNN::Tensor *inputHidden_;
    MNN::Tensor *outputHidden_;
    MNN::Tensor *outputdepthLatent_;
    MNN::Tensor *obsTensor_;
    MNN::Tensor *hiddenTensor_;
    MNN::CV::ImageProcess::Config imageProcessConfig_;
    cv::Mat float_img;

    ros::Publisher latent_pub_;

    int bpp;
    int size_h;
    int size_w;

    std::shared_ptr<MNN::Interpreter> interpreterPolicy_;
    MNN::Session *Policysession_ = nullptr;
    MNN::Tensor *inputHistorybuf_;
    MNN::Tensor *input32latent_;
    MNN::Tensor *outputAction_;
    // MNN::Tensor *historyTensor_;
    // MNN::Tensor *latentTensor_;

    bool hiddenFirst_{true};
    // depth_actor
    vector_t privLatent_;
    double deltaYaw_;
    double deltaNextYaw_;
    vector_t proprioHistoryBuffer_;
    vector_t num_scan_;
    vector_t num_priv_explicit_;

    std::mutex depth_mutex_;
    std::vector<float> depth_latent_;

    ros::Publisher obs_pub_;
    ros::Subscriber latent_sub_;
  };
}
// namespace legged