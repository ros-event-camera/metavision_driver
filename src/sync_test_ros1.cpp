// -*-c++-*----------------------------------------------------------------------------------------
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

#include "metavision_ros_driver/sync_test_ros1.h"

#include <event_array_msgs/decode.h>

#include "metavision_ros_driver/logging.h"

namespace metavision_ros_driver
{
SyncTestROS1::SyncTestROS1(const ros::NodeHandle & nh) : nh_(nh)
{
  resetVariables();
  useROSTime_ = nh_.param<bool>("use_ros_time", false);
  ROS_INFO((useROSTime_ ? "using ROS time stamps!" : "using sensor time stamps!"));

  if (!start()) {
    ROS_ERROR("sync_test startup failed!");
    throw std::runtime_error("startup of SyncTestROS1 node failed!");
  }
}

SyncTestROS1::~SyncTestROS1() {}

void SyncTestROS1::resetVariables()
{
  minDiff_ = std::numeric_limits<int64_t>::max();
  maxDiff_ = std::numeric_limits<int64_t>::min();
  count_ = 0;
  sumOfDiffs_ = 0;
  sumOfDiffWall_ = 0;
}

bool SyncTestROS1::start()
{
  camSub_[0] = nh_.subscribe("events_cam_0", 1, &SyncTestROS1::callbackCam0, this);
  camSub_[1] = nh_.subscribe("events_cam_1", 1, &SyncTestROS1::callbackCam1, this);
  return (true);
}

static uint64_t get_first_event_time(uint64_t time_base, const uint8_t * packed)
{
  uint64_t t;
  uint16_t x, y;
  (void)event_array_msgs::mono::decode_t_x_y_p(packed, time_base, &t, &x, &y);
  return (t);
}

void SyncTestROS1::callbackCam0(EventArrayConstPtr msg)
{
  if (!msg->events.empty()) {
    uint64_t base = useROSTime_ ? ros::Time(msg->header.stamp).toNSec() : msg->time_base;
    lastTimeCam0_ = get_first_event_time(base, reinterpret_cast<const uint8_t *>(&msg->events[0]));
  }
}

void SyncTestROS1::callbackCam1(EventArrayConstPtr msg)
{
  if (msg->events.empty() || lastTimeCam0_ == 0) {
    return;
  }
  uint64_t base = useROSTime_ ? ros::Time(msg->header.stamp).toNSec() : msg->time_base;
  const uint64_t stamp =
    get_first_event_time(base, reinterpret_cast<const uint8_t *>(&msg->events[0]));
  const int64_t dt = (int64_t)stamp - (int64_t)lastTimeCam0_;
  maxDiff_ = std::max(maxDiff_, dt);
  minDiff_ = std::min(minDiff_, dt);
  count_++;
  sumOfDiffs_ += dt * 1e-9;
  sumOfDiffWall_ += (ros::Time::now().toNSec() - ros::Time(msg->header.stamp).toNSec()) * 1e-9;
  if (lastTimeStats_ == 0) {
    lastTimeStats_ = stamp;
  } else if (stamp > lastTimeStats_ + statInterval_) {
    lastTimeStats_ = stamp;
    const double avg = sumOfDiffs_ / count_;
    const double avgWall = sumOfDiffWall_ / count_;
    ROS_INFO_STREAM(
      "avg diff: " << avg << " min: " << minDiff_ * 1e-9 << " max: " << maxDiff_ * 1e-9
                   << " avg wall: " << avgWall << " count: " << count_);
    resetVariables();
  }
}

}  // namespace metavision_ros_driver
