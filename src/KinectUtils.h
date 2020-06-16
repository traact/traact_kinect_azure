/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTUTILS_H_
#define TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTUTILS_H_

#include <map>
#include <k4a/k4a.hpp>

namespace traact::component::vision {

class KinectUtils {
 public:
  static std::map<std::string, k4a_wired_sync_mode_t> k4a_wired_sync_mode;
  static std::map<std::string, k4a_depth_mode_t> k4a_depth_mode;
  static std::map<std::string, k4a_color_resolution_t> k4a_color_resolution;
  static std::map<std::string, k4a_image_format_t> k4a_image_format;
  static std::map<std::string, k4a_fps_t> k4a_fps;

};

}

#endif //TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTUTILS_H_
