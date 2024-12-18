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

#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <memory>
#include <string>

namespace gazebo {
#define DEPTH_CAMERA_NAME "depth"

struct CameraParams {
  CameraParams() {}

  std::string topic_name;
  std::string camera_info_topic_name;
  std::string optical_frame;
};

/// \brief A plugin that simulates Real Sense camera streams.
class RealSensePlugin : public ModelPlugin {
  /// \brief Constructor.
public:
  RealSensePlugin();

  /// \brief Destructor.
  ~RealSensePlugin();

  // Documentation Inherited.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Callback for the World Update event.
  void OnUpdate();

  /// \brief Callback that publishes a received Depth Camera Frame as an
  /// ImageStamped
  /// message.
  virtual void OnNewDepthFrame();

protected:
  /// \brief Pointer to the model containing the plugin.
  physics::ModelPtr rsModel;

  /// \brief Pointer to the world.
  physics::WorldPtr world;

  /// \brief Pointer to the Depth Camera Renderer.
  rendering::DepthCameraPtr depthCam;

  /// \brief String to hold the camera prefix
  std::string prefix;

  /// \brief Pointer to the transport Node.
  transport::NodePtr transportNode;

  // \brief Store Real Sense depth map data.
  std::vector<uint16_t> depthMap;

  /// \brief Pointer to the Depth Publisher.
  transport::PublisherPtr depthPub;

  /// \brief Pointer to the Depth Camera callback connection.
  event::ConnectionPtr newDepthFrameConn;

  /// \brief Pointer to the World Update event connection.
  event::ConnectionPtr updateConnection;

  std::map<std::string, CameraParams> cameraParamsMap_;

  bool pointCloud_ = false;
  std::string pointCloudTopic_;
  double pointCloudCutOff_, pointCloudCutOffMax_;

  double depthUpdateRate_;

  float rangeMinDepth_;
  float rangeMaxDepth_;
};
}
#endif
