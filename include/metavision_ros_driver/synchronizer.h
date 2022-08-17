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

#ifndef METAVISION_ROS_DRIVER__SYNCHRONIZER_H_
#define METAVISION_ROS_DRIVER__SYNCHRONIZER_H_

namespace metavision_ros_driver
{
class Synchronizer
{
public:
  Synchronizer() {}
  virtual ~Synchronizer() {}
  virtual bool sendReadyMessage() = 0;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__SYNCHRONIZER_H_
