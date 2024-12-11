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

#ifndef METAVISION_DRIVER__CALLBACK_HANDLER_H_
#define METAVISION_DRIVER__CALLBACK_HANDLER_H_

#if METAVISION_VERSION < 5
#include <metavision/sdk/driver/camera.h>
#else
#include <metavision/sdk/stream/camera.h>
#endif

namespace metavision_driver
{
class CallbackHandler
{
public:
  CallbackHandler() {}
  virtual ~CallbackHandler() {}
  virtual void rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end) = 0;
  virtual void eventCDCallback(
    uint64_t t, const Metavision::EventCD * start, const Metavision::EventCD * end) = 0;
};
}  // namespace metavision_driver
#endif  // METAVISION_DRIVER__CALLBACK_HANDLER_H_
