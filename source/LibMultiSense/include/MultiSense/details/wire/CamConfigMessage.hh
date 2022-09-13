/**
 * @file LibMultiSense/CamConfigMessage.hh
 *
 * This message contains the current camera configuration.
 *
 * Copyright 2013-2022
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
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
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, Significant rewrite.
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_CamConfigMessage
#define LibMultiSense_CamConfigMessage

#include "MultiSense/details/utility/Portability.hh"
#include "MultiSense/details/wire/ExposureConfigMessage.hh"
#include "MultiSense/details/wire/Protocol.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class CamConfig {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_CAM_CONFIG;
    static CRL_CONSTEXPR VersionType VERSION = 9;

    //
    // Parameters representing the current camera configuration

    uint16_t width;
    uint16_t height;
    float    framesPerSecond;
    float    gain;

    uint32_t exposure;
    uint8_t  autoExposure;
    uint32_t autoExposureMax;
    uint32_t autoExposureDecay;
    float    autoExposureThresh;

    float    whiteBalanceRed;
    float    whiteBalanceBlue;
    uint8_t  autoWhiteBalance;
    uint32_t autoWhiteBalanceDecay;
    float    autoWhiteBalanceThresh;

    float fx, fy;
    float cx, cy;
    float tx, ty, tz;
    float roll, pitch, yaw;

    //
    // Version 2 additions

    int32_t disparities;

    //
    // Version 3 additions

    float stereoPostFilterStrength;

    //
    // Version 4 additions

    bool hdrEnabled;

    //
    // Version 5 additions

    uint16_t autoExposureRoiX;
    uint16_t autoExposureRoiY;
    uint16_t autoExposureRoiWidth;
    uint16_t autoExposureRoiHeight;

    //
    // Version 6 additions

    uint32_t cameraProfile;

    //
    // Version 7 additions

    SourceType exposureSource;
    std::vector<ExposureConfig> secondaryExposureConfigs;

    //
    // Version 8 additions

    float autoExposureTargetIntensity;
    float gamma;

    //
    // Version 9 additions

    bool sharpeningEnable;
    float sharpeningPercentage;

    //
    // Constructors

    CamConfig(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CamConfig():
        width(0),
        height(0),
        framesPerSecond(0.0),
        gain(0.0),
        exposure(0),
        autoExposure(0),
        autoExposureMax(0),
        autoExposureDecay(0),
        autoExposureThresh(0.0),
        whiteBalanceRed(0.0),
        whiteBalanceBlue(0.0),
        autoWhiteBalance(0),
        autoWhiteBalanceDecay(0),
        autoWhiteBalanceThresh(0.0),
        fx(0.0),
        fy(0.0),
        cx(0.0),
        cy(0.0),
        tx(0.0),
        ty(0.0),
        tz(0.0),
        roll(0.0),
        pitch(0.0),
        yaw(0.0),
        disparities(0),
        stereoPostFilterStrength(0.0),
        hdrEnabled(false),
        autoExposureRoiX(0),
        autoExposureRoiY(0),
        autoExposureRoiWidth(crl::multisense::Roi_Full_Image),
        autoExposureRoiHeight(crl::multisense::Roi_Full_Image),
        cameraProfile(0),
        exposureSource(Default_Exposure_Source),
        secondaryExposureConfigs(),
        autoExposureTargetIntensity(Default_Target_Intensity),
        gamma(Default_Gamma),
        sharpeningEnable(false),
        sharpeningPercentage(0.0f)
        {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & width;
        message & height;

        message & framesPerSecond;
        message & gain;

        message & exposure;
        message & autoExposure;
        message & autoExposureMax;
        message & autoExposureDecay;
        message & autoExposureThresh;

        message & whiteBalanceRed;
        message & whiteBalanceBlue;
        message & autoWhiteBalance;
        message & autoWhiteBalanceDecay;
        message & autoWhiteBalanceThresh;

        message & fx;
        message & fy;
        message & cx;
        message & cy;

        message & tx;
        message & ty;
        message & tz;

        message & roll;
        message & pitch;
        message & yaw;

        if (version >= 2)
            message & disparities;
        else
            disparities = -1;

        if (version >= 3)
            message & stereoPostFilterStrength;
        else
            stereoPostFilterStrength = 0.5f;

        if (version >= 4)
            message & hdrEnabled;
        else
            hdrEnabled = false;

        if (version >= 5)
        {
            message & autoExposureRoiX;
            message & autoExposureRoiY;
            message & autoExposureRoiWidth;
            message & autoExposureRoiHeight;
        }
        else
        {
            autoExposureRoiX = 0;
            autoExposureRoiY = 0;
            autoExposureRoiWidth = crl::multisense::Roi_Full_Image;
            autoExposureRoiHeight = crl::multisense::Roi_Full_Image;
        }

        if (version >= 6)
        {
            message & cameraProfile;
        }
        else
        {
            cameraProfile = 0;
        }

        if (version >= 7)
        {
            message & exposureSource;
            message & secondaryExposureConfigs;
        }
        else
        {
            exposureSource = Default_Exposure_Source;
            secondaryExposureConfigs = std::vector<ExposureConfig>();
        }

        if (version >= 8)
        {
            message & autoExposureTargetIntensity;
            message & gamma;
        }
        else
        {
            autoExposureTargetIntensity = Default_Target_Intensity;
            gamma = Default_Gamma;
        }

        if (version >= 9)
        {
          message & sharpeningEnable;
          message & sharpeningPercentage;
        }
        else
        {
          sharpeningEnable = false;
          sharpeningPercentage = 0.0f;
        }

    }
};

}}}} // namespaces

#endif
