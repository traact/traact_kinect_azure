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

#include <rttr/registration>
#include <rttr/type>
#include <traact/facade/Plugin.h>

#include "KinectAzureColorOutput.h"
#include "KinectAzureDepthOutput.h"
#include "KinectAzureIROutput.h"
#include "KinectAzureBodyOutput.h"
#include "KinectAzurePlayer.h"

namespace traact::vision {

	

class KinectAzurePlugin : public traact::facade::Plugin {
 public:

  void fillPatternNames(std::vector<std::string> &pattern_names) override {
    pattern_names.emplace_back("KinectAzureColorOutput");
    pattern_names.emplace_back("KinectAzureDepthOutput");
    pattern_names.emplace_back("KinectAzureIROutput");
    pattern_names.emplace_back("KinectAzureBodyOutput");
    pattern_names.emplace_back("KinectAzurePlayer");



  }
  pattern::Pattern::Ptr instantiatePattern(const std::string &pattern_name) override {
    if (pattern_name == "KinectAzureColorOutput")
      return component::vision::KinectAzureColorOutput::getPattern();
    
	if (pattern_name == "KinectAzureDepthOutput")
      return component::vision::KinectAzureDepthOutput::getPattern();
    if (pattern_name == "KinectAzureIROutput")
      return component::vision::KinectAzureIROutput::getPattern();
    if (pattern_name == "KinectAzureBodyOutput")
      return component::vision::KinectAzureBodyOutput::getPattern();
    if (pattern_name == "KinectAzurePlayer")
      return component::vision::KinectAzurePlayer::getPattern();
	  


    return nullptr;
  }
  component::Component::Ptr instantiateComponent(const std::string &pattern_name,
                                                 const std::string &new_component_name) override {

    if (pattern_name == "KinectAzureColorOutput")
      return std::make_shared<component::vision::KinectAzureColorOutput>(new_component_name);
	
    if (pattern_name == "KinectAzureDepthOutput")
      return std::make_shared<component::vision::KinectAzureDepthOutput>(new_component_name);
    if (pattern_name == "KinectAzureIROutput")
      return std::make_shared<component::vision::KinectAzureIROutput>(new_component_name);
    if (pattern_name == "KinectAzureBodyOutput")
      return std::make_shared<component::vision::KinectAzureBodyOutput>(new_component_name);
    if (pattern_name == "KinectAzurePlayer")
      return std::make_shared<component::vision::KinectAzurePlayer>(new_component_name);
	  
    return nullptr;
  }

  RTTR_ENABLE(traact::facade::Plugin)
};

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

  using namespace rttr;
  registration::class_<traact::vision::KinectAzurePlugin>("KinectAzurePlugin").constructor<>()
      (
          //policy::ctor::as_std_shared_ptr
      );
}