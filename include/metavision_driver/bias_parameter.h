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

#ifndef METAVISION_DRIVER__BIAS_PARAMETER_H_
#define METAVISION_DRIVER__BIAS_PARAMETER_H_

#include <map>
#include <string>

namespace metavision_driver
{
struct BiasParameter
{
  BiasParameter(int nv, int xv, const std::string & inf) : minVal(nv), maxVal(xv), info(inf) {}
  int minVal;
  int maxVal;
  std::string info;

  static const std::map<std::string, BiasParameter> & getAll(const std::string & sensorVersion);
};
}  // namespace metavision_driver
#endif  // METAVISION_DRIVER__BIAS_PARAMETER_H_
