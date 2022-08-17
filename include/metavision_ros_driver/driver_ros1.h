// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef METAVISION_ROS_DRIVER__DRIVER_ROS1_H_
#define METAVISION_ROS_DRIVER__DRIVER_ROS1_H_

#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <string>

#include "metavision_ros_driver/MetaVisionDynConfig.h"
#include "metavision_ros_driver/event_publisher_base.h"
#include "metavision_ros_driver/image_updater.h"
#include "metavision_ros_driver/synchronizer.h"

namespace metavision_ros_driver
{
class MetavisionWrapper;  // forward decl

class DriverROS1 : public Synchronizer
{
public:
  explicit DriverROS1(ros::NodeHandle & nh);
  ~DriverROS1();
  // ----------- from synchronizer ---------
  bool sendReadyMessage() override;

private:
  using Config = MetaVisionDynConfig;
  using CameraInfo = sensor_msgs::CameraInfo;

  // service call to dump biases
  bool saveBiases(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);

  // related to dynanmic config (runtime parameter update)
  void setBias(int * current, const std::string & name);
  void configure(Config & config, int level);

  // for primary sync
  void secondaryReadyCallback(const std_msgs::Header::ConstPtr &);

  // functions related to frame and camerainfo publishing
  void cameraInfoConnectCallback(const ros::SingleSubscriberPublisher & pub);
  void imageConnectCallback(const image_transport::SingleSubscriberPublisher & pub);
  void frameTimerExpired(const ros::TimerEvent &);
  void updateFrameTimer();
  void startNewImage();

  // misc helper functions
  bool start();
  bool stop();
  void makeEventPublisher();
  void configureWrapper(const std::string & name);

  // ------------------------  variables ------------------------------
  ros::NodeHandle nh_;
  std::shared_ptr<MetavisionWrapper> wrapper_;
  std::shared_ptr<EventPublisherBase> eventPub_;
  std::string frameId_;  // ROS frame id

  // ------ related to sync
  ros::Publisher secondaryReadyPub_;      // secondary publishes on this
  ros::Subscriber secondaryReadySub_;     // primary subscribes on this
  ros::Duration readyIntervalTime_{1.0};  // frequency of publishing ready messages
  ros::Time lastReadyTime_;               // last time ready message was published

  // ------ related to dynamic config and services
  Config config_;
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
  ros::ServiceServer saveBiasService_;

  // ------ related to rendered image publishing
  ImageUpdater imageUpdater_;
  double fps_;  // frequency of image publishing
  ros::Timer frameTimer_;
  image_transport::Publisher imagePub_;
  sensor_msgs::Image imageMsgTemplate_;

  // -------- related to camerainfo
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  ros::Publisher cameraInfoPub_;
  CameraInfo cameraInfoMsg_;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__DRIVER_ROS1_H_
