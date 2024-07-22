/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "realsense_gazebo_plugin/RealSensePlugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_SCALE_M 0.001

#define DEPTH_CAMERA_TOPIC "depth"

using namespace gazebo;

/////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin() {
  this->depthCam = nullptr;
  this->prefix = "";
  this->pointCloudCutOffMax_ = 5.0;
}

/////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin() {}

/////////////////////////////////////////////////
void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Output the name of the model
  std::cout
      << std::endl
      << "RealSensePlugin: The realsense_camera plugin is attach to model "
      << _model->GetName() << std::endl;

  _sdf = _sdf->GetFirstElement();

  cameraParamsMap_.insert(std::make_pair(DEPTH_CAMERA_NAME, CameraParams()));

  do {
    std::string name = _sdf->GetName();
    if (name == "depthUpdateRate")
      _sdf->GetValue()->Get(depthUpdateRate_);
    else if (name == "depthTopicName")
      cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "depthCameraInfoTopicName")
      cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "depthOpticalframeName")
      cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame =
          _sdf->GetValue()->GetAsString();
    else if (name == "rangeMinDepth")
      _sdf->GetValue()->Get(rangeMinDepth_);
    else if (name == "rangeMaxDepth")
      _sdf->GetValue()->Get(rangeMaxDepth_);
    else if (name == "pointCloud")
      _sdf->GetValue()->Get(pointCloud_);
    else if (name == "pointCloudTopicName")
      pointCloudTopic_ = _sdf->GetValue()->GetAsString();
    else if (name == "pointCloudCutoff")
      _sdf->GetValue()->Get(pointCloudCutOff_);
    else if (name == "pointCloudCutoffMax")
      _sdf->GetValue()->Get(pointCloudCutOffMax_);
    else if (name == "prefix")
      this->prefix = _sdf->GetValue()->GetAsString();
    else if (name == "robotNamespace")
      break;
    else
      throw std::runtime_error("Ivalid parameter for ReakSensePlugin");

    _sdf = _sdf->GetNextElement();
  } while (_sdf);

  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();

  // Get Cameras Renderers
  this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
                       smanager->GetSensor(prefix + DEPTH_CAMERA_NAME))
                       ->DepthCamera();

  // Check if camera renderers have been found successfuly
  if (!this->depthCam) {
    std::cerr << "RealSensePlugin: Depth Camera has not been found"
              << std::endl;
    return;
  }

  // Resize Depth Map dimensions
  try {
    this->depthMap.resize(this->depthCam->ImageWidth() *
                          this->depthCam->ImageHeight());
  } catch (std::bad_alloc &e) {
    std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
              << std::endl;
    return;
  }

  // Setup Transport Node
  this->transportNode = transport::NodePtr(new transport::Node());
  this->transportNode->Init(this->world->Name());

  // Setup Publishers
  std::string rsTopicRoot = "~/" + this->rsModel->GetName();

  this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, depthUpdateRate_);

  // Listen to depth camera new frame event
  this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(
      std::bind(&RealSensePlugin::OnNewDepthFrame, this));

  // Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RealSensePlugin::OnUpdate, this));
}


/////////////////////////////////////////////////
void RealSensePlugin::OnNewDepthFrame() {
  // Get Depth Map dimensions
  unsigned int imageSize =
      this->depthCam->ImageWidth() * this->depthCam->ImageHeight();

  // Instantiate message
  msgs::ImageStamped msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i) {
    // Check clipping and overflow
    if (depthDataFloat[i] < rangeMinDepth_ ||
        depthDataFloat[i] > rangeMaxDepth_ ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0) {
      this->depthMap[i] = 0;
    } else {
      this->depthMap[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
  msgs::Set(msg.mutable_time(), this->world->SimTime());
  msg.mutable_image()->set_width(this->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->depthCam->ImageWidth() *
                                this->depthCam->ImageDepth());
  msg.mutable_image()->set_data(this->depthMap.data(),
                                sizeof(*this->depthMap.data()) * imageSize);

  // Publish realsense scaled depth map
  this->depthPub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnUpdate() {}
