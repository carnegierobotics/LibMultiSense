/**
 * @file LibMultiSense/ExposureConfigMessage.hh
 *
 * Copyright 2021-2022
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * https://www.carnegierobotics.com
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
 *   2021-03-17, malvarado@carnegierobotics.com, IRAD, Create file.
 **/

#ifndef LibMultiSense_ExposureConfigMessage
#define LibMultiSense_ExposureConfigMessage

#include "MultiSense/MultiSenseTypes.hh"
#include "MultiSense/details/utility/Portability.hh"
#include "Protocol.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class ExposureConfig {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_EXPOSURE_CONFIG;
    static CRL_CONSTEXPR VersionType VERSION = 2;

    uint32_t exposure;
    uint8_t  autoExposure;
    uint32_t autoExposureMax;
    uint32_t autoExposureDecay;
    float    autoExposureThresh;

    uint16_t autoExposureRoiX;
    uint16_t autoExposureRoiY;
    uint16_t autoExposureRoiWidth;
    uint16_t autoExposureRoiHeight;

    SourceType exposureSource;
    float      autoExposureTargetIntensity;
    float      gain;

    ExposureConfig(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    ExposureConfig():
          exposure(0),
          autoExposure(0),
          autoExposureMax(0),
          autoExposureDecay(0),
          autoExposureThresh(0.0),
          autoExposureRoiX(0),
          autoExposureRoiY(0),
          autoExposureRoiWidth(crl::multisense::Roi_Full_Image),
          autoExposureRoiHeight(crl::multisense::Roi_Full_Image),
          exposureSource(Default_Exposure_Source),
          autoExposureTargetIntensity(Default_Target_Intensity),
          gain(Default_Gain)
        {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & exposure;
        message & autoExposure;
        message & autoExposureMax;
        message & autoExposureDecay;
        message & autoExposureThresh;

        message & autoExposureRoiX;
        message & autoExposureRoiY;
        message & autoExposureRoiWidth;
        message & autoExposureRoiHeight;

        message & exposureSource;

        if (version >= 2)
        {
            message & autoExposureTargetIntensity;
            message & gain;
        }
        else
        {
            autoExposureTargetIntensity = Default_Target_Intensity;
            gain = Default_Gain;
        }


    }
};

}}}} // namespaces

#endif
