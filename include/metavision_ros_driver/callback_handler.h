// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef METAVISION_ROS_DRIVER__CALLBACK_HANDLER_H_
#define METAVISION_ROS_DRIVER__CALLBACK_HANDLER_H_

#include <metavision/sdk/driver/camera.h>

namespace metavision_ros_driver
{
class CallbackHandler
{
public:
  CallbackHandler() {}
  virtual ~CallbackHandler() {}
  virtual void publishExtTrigger(const Metavision::EventExtTrigger * start, const Metavision::EventExtTrigger * end) = 0;
  virtual void publish(const Metavision::EventCD * start, const Metavision::EventCD * end) = 0;
  virtual bool keepRunning() = 0;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__CALLBACK_HANDLER_H_
