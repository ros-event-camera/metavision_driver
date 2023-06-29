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

#ifndef METAVISION_DRIVER__RESIZE_HACK_H_
#define METAVISION_DRIVER__RESIZE_HACK_H_
namespace metavision_driver
{
//
// resize without initializing, taken from
// https://stackoverflow.com/questions/15219984/
// using-vectorchar-as-a-buffer-without-initializing-it-on-resize

template <typename V>
void resize_hack(V & v, size_t newSize)
{
  struct vt
  {
    typename V::value_type v;
    vt() {}
  };
  static_assert(sizeof(vt[10]) == sizeof(typename V::value_type[10]), "alignment error");
  typedef std::vector<
    vt, typename std::allocator_traits<typename V::allocator_type>::template rebind_alloc<vt>>
    V2;
  reinterpret_cast<V2 &>(v).resize(newSize);
}
}  // namespace metavision_driver
#endif  // METAVISION_DRIVER__RESIZE_HACK_H_
