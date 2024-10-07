/**
 * @file LibMultiSense/FeatureDetectorConfigMessage.hh
 *
 * This message contains the current feature detector configuration.
 *
 * Copyright 2013-2024
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegiearobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Significant history (date, user, job code, action):
 *   2024-25-01, patrick.smith@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef LibMultisense_FeatureDetectorConfigMessage
#define LibMultisense_FeatureDetectorConfigMessage

#include "MultiSense/details/utility/Portability.hh"
#include "MultiSense/details/wire/Protocol.hh"
#include "MultiSense/details/wire/SecondaryAppConfigMessage.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

struct FeatureDetectorConfigItems {

    //
    // The maximum number of features detected per image
    uint32_t numberOfFeatures;

    //
    // Enable/Disable feature grouping
    bool grouping;

    //
    // Enable motion detection
    // Currently this functions as enable/disable but could be used to specify
    // which octave motion detection is performed on.
    // Current Octave: 3
    uint32_t motion;

};

class FeatureDetectorConfig {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_FEATURE_DETECTOR_CONFIG;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    //
    // Parameters representing the current camera configuration

    FeatureDetectorConfigItems configItems;

    SecondaryAppConfig configData;

    //
    // Constructors

    FeatureDetectorConfig( SecondaryAppConfig & d ) {

        configData = d;
        if (d.dataLength != sizeof(configItems))
        {
          throw std::runtime_error("Error Invalid receipt length\n");
        }

        memcpy(&configItems, d.data, d.dataLength);

    };
    FeatureDetectorConfig() {};

};

}}}} // namespaces

#endif
