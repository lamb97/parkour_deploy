#ifndef IMAGENODE_H
#define IMAGENODE_H

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "control_msg/TimestampedFloat32MultiArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>

#include <thread>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <fstream>
#include <ros/package.h>


#include "Types.h"
#include "TimeDebug.h"

#include <Eigen/Geometry>

#include <MNN/ImageProcess.hpp>
#include <boost/thread/mutex.hpp>
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

struct Command
{
std::atomic<scalar_t> x;
std::atomic<scalar_t> y;
std::atomic<scalar_t> yaw;
};

class ImageNode {
    using tensor_element_t = float;
public:
    ImageNode(ros::NodeHandle& nh, std::string depth_image_topic, std::string joint_info_topic);
    ~ImageNode(){}
    bool init(ros::NodeHandle& nh);
    std::string depth_image_topic_;
    // 深度裁剪
    cv::Mat depthClipping(const cv::Mat& image, double minDepth, double maxDepth);

    // 高斯噪声
    cv::Mat addGaussianNoise(const cv::Mat& image, double mean = 0, double sigma = 25);

    // 随机伪影
    cv::Mat addRandomArtifacts(const cv::Mat& image, int numArtifacts = 100);

    // 孔洞填补
    cv::Mat fillHoles(const cv::Mat& image);

    // 空间滤波
    cv::Mat spatialFilter(const cv::Mat& image, int kernelSize = 5);

protected:
    void obsCallback(const control_msg::TimestampedFloat32MultiArray::ConstPtr& joint_msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg);
    //void MessageCallback(const control_msg::TimestampedFloat32MultiArray::ConstPtr& msg, const sensor_msgs::Image::ConstPtr& img_msg);
    void computeLaten();
    bool loadRLCfg(ros::NodeHandle& nh);
    bool loadModel(ros::NodeHandle& nh);
    void updateInputImage();
    void updateInputObs();
    // void converHiddenStateDataToInputHidden();
    void printTensorDimensions(const MNN::Tensor *tensor);
    void initMNNTensorWithZero(MNN::Tensor* mnn_tensor);


private:
    void messageProcessingLoop();

    RLRobotCfg robotCfg_{};

    ros::NodeHandle nh_;
    std::string obs_info_topic_;
    boost::mutex image_boostmutex_;
    boost::mutex joint_boostmutex_;
    std::mutex obs_mutex_;
    std::mutex image_mutex_;
    boost::thread processing_thread_;
    std::atomic<bool> stop_thread_;
    
    ros::Subscriber obs_sub_;
    ros::Subscriber image_sub_;
    ros::Publisher latent_pub_;
    control_msg::TimestampedFloat32MultiArray copied_joint_msg_;
    sensor_msgs::Image copied_img_msg_;
    //message_filters::Subscriber<control_msg::TimestampedFloat32MultiArray> obs_sub_;
    //message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    //message_filters::TimeSynchronizer<control_msg::TimestampedFloat32MultiArray, sensor_msgs::Image> sync_;

    int actionsSize_;
    int observationSize_;
    int size_h;
    int size_w;
    int actuatedDofNum_ = 12;
    double noise_strength = 0.;

    std_msgs::Float32MultiArray latentMsg_; 

    std::vector<tensor_element_t> hiddenStateData_;
    std::vector<tensor_element_t> observations_;
    std::vector<tensor_element_t> obs_tmp_;
    std::vector<tensor_element_t> depthLatent_;
    vector_t historyObservations_;
    cv::Mat tmp_img_;
    // vector_t proprioObs;

    std::shared_ptr<MNN::Interpreter> interpreterImage_;
    MNN::Session *Imagesession_;
    MNN::Tensor *inputImage_;
    MNN::Tensor *inputObs_;
    MNN::Tensor *inputHidden_;
    MNN::Tensor *outputHidden_;
    MNN::Tensor *outputdepthLatent_;
    // MNN::Tensor *obsTensor_;
    MNN::Tensor *imageTensor_;
    // MNN::Tensor *hiddenTensor_;
    MNN::CV::ImageProcess::Config imageProcessConfig_;

    ros::Duration depthDelay_;
    ros::Duration tpImage_;
    ros::Time reciveImageTime_;
    ros::Time sendImageTime_;
    cv::Mat resizDepthImage_;

    std::string imagePath_;
    ros::Subscriber dataSub_;
    ros::Time switchTime;
    std::atomic_bool dataCollect{false};

};

#endif // IMAGENODE_H
