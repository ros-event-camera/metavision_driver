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

#ifndef METAVISION_DRIVER__DRIVER_ROS1_H_
#define METAVISION_DRIVER__DRIVER_ROS1_H_

#include <dynamic_reconfigure/server.h>
#include <event_camera_msgs/EventPacket.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <string>

#include "metavision_driver/MetaVisionDynConfig.h"
#include "metavision_driver/bias_parameter.h"
#include "metavision_driver/callback_handler.h"
#include "metavision_driver/resize_hack.h"

namespace metavision_driver
{
class MetavisionWrapper;  // forward decl

class DriverROS1 : public CallbackHandler
{
  using Config = MetaVisionDynConfig;
  using EventPacketMsg = event_camera_msgs::EventPacket;
  using Trigger = std_srvs::Trigger;

public:
  explicit DriverROS1(ros::NodeHandle & nh);
  ~DriverROS1();

  // ---------------- inherited from CallbackHandler -----------
  void rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end) override;
  void eventCDCallback(
    uint64_t t, const Metavision::EventCD * begin, const Metavision::EventCD * end) override;
  // ---------------- end of inherited  -----------

private:
  // service call to dump biases
  bool saveBiases(Trigger::Request & req, Trigger::Response & res);

  // related to dynanmic config (runtime parameter update)
  void setBias(int * field, const std::string & name);
  int getBias(const std::string & name) const;

  void configure(Config & config, int level);

  // for primary sync
  bool secondaryReadyCallback(Trigger::Request & req, Trigger::Response & res);

  // misc helper functions
  void start();
  bool stop();
  void configureWrapper(const std::string & name);
  void initializeBiasParameters(const std::string & sensorVersion);
  // ------------------------  variables ------------------------------
  ros::NodeHandle nh_;
  std::shared_ptr<MetavisionWrapper> wrapper_;
  int width_;   // image width
  int height_;  // image height
  bool isBigEndian_;
  std::string frameId_;  // ROS frame id
  std::string encoding_;
  uint64_t seq_{0};        // sequence number
  size_t reserveSize_{0};  // recommended reserve size
  uint64_t lastMessageTime_{0};
  uint64_t messageThresholdTime_{0};  // threshold time for sending message
  size_t messageThresholdSize_{0};    // threshold size for sending message
  EventPacketMsg::Ptr msg_;
  ros::Publisher eventPub_;

  // ------ related to sync
  ros::ServiceServer secondaryReadyServer_;
  // ------ related to dynamic config and services
  Config config_;
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
  ros::ServiceServer saveBiasService_;
  using ParameterMap = std::map<std::string, BiasParameter>;
  ParameterMap biasParameters_;
};
}  // namespace metavision_driver
#endif  // METAVISION_DRIVER__DRIVER_ROS1_H_
