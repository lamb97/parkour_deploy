#include "ImageNode.h"
#include <opencv2/opencv.hpp>

ImageNode::ImageNode(ros::NodeHandle &nh, std::string depth_image_topic, std::string obs_info_topic)
    /*     // : obs_sub_(nh, obs_info_topic, 100),
        //   image_sub_(nh, depth_image_topic, 100),
        //   sync_(obs_sub_, image_sub_, 1) // 初始化TimeSynchronizer */
    : nh_(nh),
      obs_info_topic_(obs_info_topic),
      depth_image_topic_(depth_image_topic),
      stop_thread_(true)
{
    std::cout << "depth_image_topic:" << depth_image_topic << std::endl;
    // std::cout << "obs_info_topic:" << obs_info_topic << std::endl;
}

bool ImageNode::init(ros::NodeHandle &nh)
{
    loadRLCfg(nh);
    loadModel(nh);
    std::string package_path = ros::package::getPath("rl_controllers");
    imagePath_ = package_path + "/image_data/imageSimData.csv";

    obs_sub_ = nh.subscribe(obs_info_topic_, 1, &ImageNode::obsCallback, this);
    image_sub_ = nh.subscribe(depth_image_topic_, 1, &ImageNode::imageCallback, this);

    latent_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/latent_topic", 1);

    cv::namedWindow("Resizable Window", cv::WINDOW_NORMAL);
    cv::resizeWindow("Resizable Window", 800, 600);
    //TODO：53
    obs_tmp_.assign(53, 0);
    observations_.assign(53, 0);
    //TODO：512
    hiddenStateData_.assign(512, 0);
    //TODO：34
    latentMsg_.data.assign(34, 0);

    tmp_img_ = cv::Mat::zeros(60, 106, CV_16UC1);

    // start control
    auto dataCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
        ros::Duration t(0.5);
        if (ros::Time::now() - switchTime > t)
        {
            if (!dataCollect)
            {
                dataCollect = true;
                ROS_INFO("DATA COLLECT");
            }
            else
            {
                dataCollect = false;
            }
            switchTime = ros::Time::now();
        }
    };
    dataSub_ = nh.subscribe<std_msgs::Float32>("/sub_data", 1, dataCallback);
    processing_thread_ = boost::thread(&ImageNode::messageProcessingLoop, this);
    ROS_INFO("image init");
    return true;
}

void ImageNode::obsCallback(const control_msg::TimestampedFloat32MultiArray::ConstPtr &obs_msg)
{
    std::lock_guard<std::mutex> guard(obs_mutex_);
    obs_tmp_ = obs_msg->data.data;
}

void ImageNode::imageCallback(const sensor_msgs::Image::ConstPtr &img_msg)
{
    std::lock_guard<std::mutex> guard(image_mutex_);
    reciveImageTime_ = ros::Time::now();
    cv::Mat temp_img(
        img_msg->height,
        img_msg->width,
        CV_16UC1,
        const_cast<uint8_t *>(img_msg->data.data()),
        img_msg->step); // qian kaobei
    cv::resize(temp_img, resizDepthImage_, cv::Size(106, 60), 0, 0, cv::INTER_NEAREST);
    tmp_img_ = temp_img.clone(); // 深拷贝数据
}
//TODO：message
void ImageNode::messageProcessingLoop()
{
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        ROS_INFO_STREAM_ONCE("enter processMessages");
        // Timer timer_;
        // DEBUG_TIMER_START(timer_);
        updateInputImage();
        updateInputObs();
        // DEBUG_TIMER_DURATION(timer_, "Running the inputmsg conver");
        computeLaten(); // 进行推理并发布推理结果
        rate.sleep();
    }
}

// 函数：打印Tensor的维度
void ImageNode::printTensorDimensions(const MNN::Tensor *tensor)
{
    // 获取Tensor的维度
    auto dims = tensor->shape();

    // 输出维度信息
    std::cout << "Tensor dimensions: [";
    for (size_t i = 0; i < dims.size(); ++i)
    {
        std::cout << dims[i];
        if (i != dims.size() - 1)
        {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    std::cout << "inputSize:" << tensor->elementSize() << std::endl;
}

// 函数：打印cv::Mat对象的维度信息
void printMatInfo(const cv::Mat &img)
{
    std::cout << "Dimensions: " << img.rows << " x " << img.cols << std::endl;

    // 获取数据类型的字符串表示
    std::string rtype;
    uchar depth = img.type() & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (img.type() >> CV_CN_SHIFT);
    switch (depth)
    {
    case CV_8U:
        rtype = "8U";
        break;
    case CV_8S:
        rtype = "8S";
        break;
    case CV_16U:
        rtype = "16U";
        break;
    case CV_16S:
        rtype = "16S";
        break;
    case CV_32S:
        rtype = "32S";
        break;
    case CV_32F:
        rtype = "32F";
        break;
    case CV_64F:
        rtype = "64F";
        break;
    default:
        rtype = "User";
        break;
    }
    rtype += "C";
    rtype += (chans + '0');

    std::cout << "Data type: CV_" << rtype << std::endl;
}

// 打印 MNN Tensor 的函数定义
void printMNNTensor(const MNN::Tensor *tensor)
{
    if (tensor == nullptr)
    {
        std::cerr << "Tensor is null!" << std::endl;
        return;
    }

    // 打印数据类型
    std::cout << "Data Type: ";
    switch (tensor->getType().code)
    {
    case halide_type_int:
        std::cout << "int" << std::endl;
        break;
    case halide_type_uint:
        std::cout << "uint" << std::endl;
        break;
    case halide_type_float:
        std::cout << "float" << std::endl;
        break;
    case halide_type_handle:
        std::cout << "handle" << std::endl;
        break;
    default:
        std::cout << "unknown" << std::endl;
        break;
    }

    // 获取张量的维度信息
    const std::vector<int> &dims = tensor->shape();
    int dimensionCount = dims.size();

    // 打印张量的维度信息
    std::cout << "Tensor dimensions: ";
    for (int i = 0; i < dimensionCount; ++i)
    {
        // std::cout << dims[i] << " ";/
    }
    std::cout << std::endl;

    // 计算张量中的元素总数，并打印每一步的结果
    int elementCount = 1;
    for (int i = 0; i < dimensionCount; ++i)
    {
        elementCount *= dims[i];
    }
    std::cout << "Total elements: " << elementCount << std::endl;

    // 确认数据类型为 float
    if (tensor->getType().code != halide_type_float)
    {
        std::cerr << "Tensor data type is not float!" << std::endl;
        return;
    }

    // 获取张量数据的指针
    const float *data = tensor->host<float>();

    // 打印张量的值
    std::cout << "Tensor values: ";
    for (int i = 0; i < elementCount; ++i)
    {
        // std::cout << data[i] << " ";
    }
    std::cout << std::endl;
}

void ImageNode::updateInputImage()
{
    cv::Mat depth_img;
    image_mutex_.lock();
    depth_img = tmp_img_.clone(); // 深拷贝数据
    image_mutex_.unlock();

    depth_img = depth_img(cv::Rect(4, 0, depth_img.cols - 8, depth_img.rows - 2));
    depth_img.convertTo(depth_img, CV_32F, 1.0 / 1000.0);
    double far_clip_ = 2.0;
    double near_clip_ = 0.0;
    // std::cout << depth_img << "\n**********.......*****\n";
    cv::threshold(depth_img, depth_img, far_clip_, far_clip_, cv::THRESH_TRUNC);
    // std::cout << depth_img << "\n************ssssssssssss***\n";
    cv::threshold(depth_img, depth_img, near_clip_, near_clip_, cv::THRESH_TOZERO); // 跟python不是特别对齐，只有阈值为0时，一样。不过感觉c++版本更合理

    // std::cout << depth_img << "\n***************\n";
    //TODO：『：58，87』

    cv::Mat resized_img;
    cv::resize(depth_img, resized_img, cv::Size(87, 58), 0, 0, cv::INTER_CUBIC);

    /*
        为图像添加噪声
    */
    // 配置噪声的强度
    // noise_strength = 0.; // 根据你的配置调整
    // 创建与depth_img同样大小的噪声矩阵
    // cv::Mat noise = cv::Mat(depth_img.size(), CV_32F);
    // // 使用OpenCV的randu函数生成均匀分布的噪声
    // cv::randu(noise, -0.5, 0.5);
    // // 将噪声添加到深度图像
    // depth_img += noise_strength * 2.0 * noise;

    depth_img = (resized_img - near_clip_) / (far_clip_ - near_clip_) - 0.5;
    // depth_img = depth_img*0 + 0.5;

    // std::cout << depth_img << "\n";

    // 归一化深度图像到0-255
    cv::Mat normalizedDepth;
    double minVal, maxVal;
    minVal = near_clip_;
    maxVal = far_clip_;
    // cv::minMaxIdx(depth_img, &minVal, &maxVal); // 找到深度图中的最小和最大值
    cv::convertScaleAbs(depth_img, normalizedDepth, 255 / (maxVal - minVal));

    // 应用伪彩色
    cv::Mat coloredDepth;
    cv::applyColorMap(normalizedDepth, coloredDepth, cv::COLORMAP_JET);

    // 显示处理后的帧
    cv::imshow("Resizable Window", coloredDepth);

    // 检测按键（如果按下ESC键，退出）
    if (cv::waitKey(1) == 27)
    {
        exit(0);
    }

    // std::cout << depth_img << "\n";

    ///////pre_process
    cv::Mat clippedImage = depthClipping(depth_img, 0.0, 10.0); // 示例深度裁剪范围
    cv::Mat noisyImage = addGaussianNoise(clippedImage);
    cv::Mat artifactImage = addRandomArtifacts(noisyImage);
    cv::Mat filledImage = fillHoles(artifactImage);
    cv::Mat filteredImage = spatialFilter(filledImage);

    auto imageDims = inputImage_->shape();

    auto tensorData = inputImage_->host<float>(); // Adjust according to the expected data type in the tensor
    auto tensorHeight = imageDims[1];
    auto tensorWidth = imageDims[2];

    auto imageTensor = new MNN::Tensor(inputImage_, MNN::Tensor::CAFFE);
    for (int y = 0; y < tensorHeight; ++y)
    {
        for (int x = 0; x < tensorWidth; ++x)
        {
            int tensorIndex = y * tensorWidth + x; // MNN的索引
            imageTensor->host<float>()[tensorIndex] = depth_img.at<float>(y, x);
        }
    }
    inputImage_->copyFromHostTensor(imageTensor);
    delete imageTensor;
}

void ImageNode::updateInputObs()
{
    obs_mutex_.lock();
    observations_ = obs_tmp_;
    obs_mutex_.unlock();

    // for (float num : obs_tmp_) {
    //     std::cout << num << " ";
    // }
    // std::cout << "dsasd" << std::endl;

    auto obsTensor = new MNN::Tensor(inputObs_, MNN::Tensor::CAFFE);
    for (size_t i = 0; i < obsTensor->elementSize(); i++)
    {
        obsTensor->host<float>()[i] = observations_[i];
        // obsTensor->host<float>()[i] = 0.0;
    }
    // std::cout << "obsTensor->elementSize()::" << obsTensor->elementSize() << std::endl;
    inputObs_->copyFromHostTensor(obsTensor);
    delete obsTensor;
}

// void ImageNode::converHiddenStateDataToInputHidden() {
//     // 运行模型前的输入数据设置
//     auto hiddenTensor_ = new MNN::Tensor(inputHidden_, MNN::Tensor::CAFFE);
//     for (int i = 0; i < hiddenStateData_.size(); i++) {
//         hiddenTensor_->host<float>()[i] = hiddenStateData_[i];
//     }
//     inputHidden_->copyFromHostTensor(hiddenTensor_);
//     delete hiddenTensor_;

// }

void ImageNode::initMNNTensorWithZero(MNN::Tensor *mnn_tensor)
{
    auto tmp_tensor = new MNN::Tensor(mnn_tensor, MNN::Tensor::CAFFE);
    for (size_t i = 0; i < tmp_tensor->elementSize(); ++i)
    {
        tmp_tensor->host<float>()[i] = 0;
    }
    mnn_tensor->copyFromHostTensor(tmp_tensor);
    delete tmp_tensor;
}

void ImageNode::computeLaten()
{
    // 运行模型
    Timer timer_;
    // printMNNTensor(inputImage_);
    // std::cout << "------------------------------------------" << std::endl;
    // DEBUG_TIMER_START(timer_);
    // printTensorDimensions(inputImage_);
    interpreterImage_->runSession(Imagesession_);
    // DEBUG_TIMER_DURATION(timer_, "Running the model session");
    //  printMNNTensor(inputImage_);

    // printMNNTensor(outputdepthLatent_);
    inputHidden_->copyFromHostTensor(outputHidden_);
    // memcpy(hiddenStateData_.data(), outputHidden_->host<float>(), hiddenStateData_.size() * sizeof(float));
    // 将数据从 outputdepthLatent_ 复制到 latentMsg_
    // std::copy(outputdepthLatent_->host<float>(),
    //         outputdepthLatent_->host<float>() + outputdepthLatent_->elementSize(),
    //         latentMsg_.data.begin());
    auto tmp_tensor = new MNN::Tensor(outputdepthLatent_, MNN::Tensor::CAFFE);
    outputdepthLatent_->copyToHostTensor(tmp_tensor);
    for (size_t i = 0; i < tmp_tensor->elementSize(); ++i)
    {
        latentMsg_.data[i] = tmp_tensor->host<float>()[i];
        // latentMsg_.data[i] = 0;
    }
    delete tmp_tensor;

    // for (float num : latentMsg_.data) {
    //     std::cout << num << " ";
    // }
    // std::cout << std::endl;

    // 发布latent输出
    sendImageTime_ = ros::Time::now();
    tpImage_ = sendImageTime_ - reciveImageTime_;
    if (tpImage_ < depthDelay_)
    {
        tpImage_ = depthDelay_ - tpImage_;
        tpImage_.sleep();
    }
    // std::cout << "t_p=" << ros::Time::now() - reciveImageTime_ << std::endl;
    latent_pub_.publish(latentMsg_);
}

bool ImageNode::loadModel(ros::NodeHandle &nh)
{
    std::string encoderModelPath;
    if (!nh.getParam("/encoderModelPath", encoderModelPath))
    {
        ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
        return false;
    }
    ROS_INFO_STREAM("Load Image_model model from path : " << encoderModelPath);

    // Create the MNN interpreter instance
    interpreterImage_ = std::unique_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(encoderModelPath.c_str()));
    if (!interpreterImage_)
    {
        ROS_ERROR_STREAM("Failed to create MNN interpreter!");
        return false;
    }

    // Create session with default options
    MNN::ScheduleConfig config;
    config.numThread = 4; // Set the number of threads
    config.type = MNN_FORWARD_CPU;
    MNN::BackendConfig backendConfig;
    backendConfig.power = MNN::BackendConfig::Power_High;
    backendConfig.memory = MNN::BackendConfig::Memory_High;
    backendConfig.precision = MNN::BackendConfig::Precision_Low;
    config.backendConfig = &backendConfig;

    Imagesession_ = interpreterImage_->createSession(config);
    if (!Imagesession_)
    {
        ROS_ERROR_STREAM("Failed to create MNN session!");
        return false;
    }
    // Get input info
    inputImage_ = interpreterImage_->getSessionInput(Imagesession_, "image");
    inputObs_ = interpreterImage_->getSessionInput(Imagesession_, "propri");
    inputHidden_ = interpreterImage_->getSessionInput(Imagesession_, "hidden_state.1");
    // Get output info
    outputdepthLatent_ = interpreterImage_->getSessionOutput(Imagesession_, "depth_latent");
    outputHidden_ = interpreterImage_->getSessionOutput(Imagesession_, "hidden_state");

    initMNNTensorWithZero(inputImage_);
    initMNNTensorWithZero(inputObs_);
    initMNNTensorWithZero(inputHidden_);

    ROS_INFO_STREAM("Image model loaded successfully!");
    return true;
}

bool ImageNode::loadRLCfg(ros::NodeHandle &nh)
{
    depthDelay_ = ros::Duration(0.08);
    return (true);
}



////////////pre_process
// Depth_Clipping
cv::Mat depthClipping(const cv::Mat& image, double minDepth, double maxDepth) {
    cv::Mat clippedImage;
    cv::threshold(image, clippedImage, minDepth, 255, cv::THRESH_BINARY);
    cv::threshold(clippedImage, clippedImage, maxDepth, 255, cv::THRESH_TRUNC);
    return clippedImage;
}

// Gaussian_Noise:
cv::Mat addGaussianNoise(const cv::Mat& image, double mean = 0, double sigma = 25) {
    cv::Mat noise(image.size(), CV_32F);
    cv::RNG rng;
    rng.fill(noise, cv::RNG::NORMAL, mean, sigma);
    cv::Mat noisyImage;
    image.convertTo(noisyImage, CV_32F);
    cv::add(noisyImage, noise, noisyImage);
    noisyImage.convertTo(noisyImage, CV_8UC1);
    return noisyImage;
}

// Random_Artifacts
cv::Mat addRandomArtifacts(const cv::Mat& image, int numArtifacts = 100) {
    cv::Mat artifactImage = image.clone();
    cv::RNG rng;
    for (int i = 0; i < numArtifacts; ++i) {
        int x = rng.uniform(0, image.cols);
        int y = rng.uniform(0, image.rows);
        artifactImage.at<uchar>(y, x) = rng.uniform(0, 256);
    }
    return artifactImage;
}

// Hole-Filling
cv::Mat fillHoles(const cv::Mat& image) {
    cv::Mat binaryImage;
    cv::threshold(image, binaryImage, 1, 255, cv::THRESH_BINARY);
    cv::Mat filledImage = image.clone();
    cv::findContours(binaryImage, std::vector<std::vector<cv::Point>>(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(filledImage, std::vector<std::vector<cv::Point>>(), -1, cv::Scalar(255), cv::FILLED);
    return filledImage;
}

// Spatial and Temporal Filtering
cv::Mat spatialFilter(const cv::Mat& image, int kernelSize = 5) {
    cv::Mat filteredImage;
    cv::GaussianBlur(image, filteredImage, cv::Size(kernelSize, kernelSize), 0);
    return filteredImage;
}