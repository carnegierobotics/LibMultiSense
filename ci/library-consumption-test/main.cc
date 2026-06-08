///
/// @file
///
/// Copyright 2013-2025
/// Carnegie Robotics, LLC
/// 4501 Hatfield Street, Pittsburgh, PA 15201
/// http://www.carnegierobotics.com
///
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
/// following conditions are met:
///     * Redistributions of source code must retain the above copyright notice, this list of conditions and the
///       following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
///       following disclaimer in the documentation and/or other materials provided with the distribution.
///     * Neither the name of the Carnegie Robotics, LLC nor the names of its contributors may be used to endorse or
///       promote products derived from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
/// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
/// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
/// Significant history (date, user, job code, action):
///   2026-05-15, emusser@carnegierobotics.com, IRAD, Created file.
///

//
// LibMultiSense library-consumption test.
//
// This is a simple program that exercises exported MULTISENSE_API symbols.  Calling them is enough to verify that:
//   - the installed public headers compile cleanly,
//   - the consumer links against the installed MultiSense target,
//   - the installed shared library can be loaded at runtime (RPATH on Linux/macOS, DLL search path on Windows).
//
// When the install was built with the optional JSON serialization or OpenCV helpers, we additionally exercise those
// features.  This file includes <nlohmann/json.hpp> and <opencv2/opencv.hpp> directly without the consumer ever calling
// find_package(nlohmann_json) or find_package(OpenCV), ensuring that those dependencies propagate transitively through
// find_package(MultiSense).
//

#include <iostream>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#ifdef CONSUME_JSON_SERIALIZATION
#include <MultiSense/MultiSenseSerialization.hh>
#endif

#ifdef CONSUME_OPENCV
#include <opencv2/core.hpp>
#endif

int main()
{
    //
    // Exercise the exported MULTISENSE_API symbols.
    //

    const auto status = multisense::to_string(multisense::Status::OK);
    const auto source = multisense::to_string(multisense::DataSource::LEFT_RECTIFIED_RAW);

    std::cout << "MultiSense library consumption test: " << status << ", " << source << std::endl;

#ifdef CONSUME_JSON_SERIALIZATION
    //
    // Exercise the JSON serialization helpers.
    //

    multisense::Channel::Config json_config;
    json_config.ip_address = "10.66.171.21";
    const nlohmann::json serialized = json_config;
    const auto roundtrip = serialized.get<multisense::Channel::Config>();
    if (roundtrip.ip_address != json_config.ip_address)
    {
        std::cerr << "JSON serialization round-trip mismatch: "
                  << roundtrip.ip_address << " != " << json_config.ip_address << std::endl;
        return 1;
    }
    std::cout << "JSON serialization consumption: " << serialized.dump() << std::endl;
#endif

#ifdef CONSUME_OPENCV
    //
    // Exercise the OpenCV-aware helpers.
    //

    const multisense::FeatureMessage features;
    const std::vector<cv::KeyPoint> keypoints = features.cv_keypoints();
    if (!keypoints.empty())
    {
        std::cerr << "OpenCV consumption: expected empty keypoints from default FeatureMessage" << std::endl;
        return 1;
    }
    std::cout << "OpenCV consumption: cv_keypoints() returned " << keypoints.size() << " keypoints" << std::endl;
#endif

    return 0;
}
