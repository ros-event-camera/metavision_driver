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

#ifndef METAVISION_ROS_DRIVER__IMAGE_UPDATER_H_
#define METAVISION_ROS_DRIVER__IMAGE_UPDATER_H_

#include <metavision/sdk/driver/camera.h>

#include <memory>
#include <mutex>
#ifdef USING_ROS_1
#include <sensor_msgs/Image.h>
#else
#include <sensor_msgs/msg/image.hpp>
#endif

#include "metavision_ros_driver/callback_handler.h"

namespace metavision_ros_driver
{
class ImageUpdater : public CallbackHandler
{
public:
#ifdef USING_ROS_1
  typedef std::unique_ptr<sensor_msgs::Image> ImgPtr;
#else
  using ImgPtr = sensor_msgs::msg::Image::UniquePtr;
#endif
  // ---------- inherited from CallbackHandler
  void triggerEventCallback(
    const Metavision::EventExtTrigger *, const Metavision::EventExtTrigger *) override{};

  bool keepRunning() override { return (true); }

  void cdEventCallback(const Metavision::EventCD * start, const Metavision::EventCD * end) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (img_ != nullptr) {
      const size_t n = end - start;
      for (unsigned int i = 0; i < n; i++) {
        const auto & e = start[i];
        const uint32_t offset = e.x * 3 + img_->step * e.y + (e.p ? 0 : 2);
        img_->data[offset] = 255;
      }
    }
  }

  void rawDataCallback(const uint8_t* data, size_t size) override
  {
	  // TODO fill this in
  }

  // ----------------- end of inherited from CallbackHandler ----

  // note: returns reference to pointer to allow for std::move()
  inline ImgPtr getImage()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return (std::move(img_));
  }
  // check if currently building an image
  inline bool hasImage()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return (img_ != nullptr);
  }

  // deallocate memory associated with pointer
  inline void resetImagePtr()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    img_.reset();
  }

  // take ownership of pointer and memory
  inline void setImage(ImgPtr * img)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    img_ = std::move(*img);
  }

private:
  ImgPtr img_;
  std::mutex mutex_;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__IMAGE_UPDATER_H_
