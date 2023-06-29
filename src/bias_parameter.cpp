// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "metavision_driver/bias_parameter.h"

namespace metavision_driver
{
static const std::map<std::string, std::map<std::string, BiasParameter>> biasParameters{
  {{"UNKNOWN", {}},
   {"3.1",
    {{"bias_diff_on", {374, 499, "on threshold level"}},
     {"bias_diff_off", {100, 234, "off threshold level"}},
     {"bias_fo", {1250, 1800, "source follower low pass filter"}},
     {"bias_hpf", {900, 1800, "differentiator high pass filter"}},
     {"bias_pr", {0, 1800, "photoreceptor (frontend) bias"}},
     {"bias_refr", {1300, 1800, "refractory time bias"}}}},
   {"4.1",
    {{"bias_diff_on", {95, 140, "on threshold level"}},
     {"bias_diff_off", {25, 65, "off threshold level"}},
     {"bias_fo", {45, 110, "source follower low pass filter"}},
     {"bias_hpf", {0, 120, "differentiator high pass filter"}},
     {"bias_refr", {30, 100, "refractory time bias"}}}},
   {"4.2",
    {{"bias_diff_on", {-85, 140, "on threshold level"}},
     {"bias_diff_off", {-35, 190, "off threshold level"}},
     {"bias_fo", {-35, 55, "source follower low pass filter"}},
     {"bias_hpf", {0, 120, "differentiator high pass filter"}},
     {"bias_refr", {-20, 235, "refractory time bias"}}}}}};

const std::map<std::string, BiasParameter> & BiasParameter::getAll(
  const std::string & sensorVersion)
{
  const auto it = biasParameters.find(sensorVersion);
  if (it == biasParameters.end()) {
    return (biasParameters.find("UNKNOWN")->second);
  }
  return (it->second);
}
}  // namespace metavision_driver
