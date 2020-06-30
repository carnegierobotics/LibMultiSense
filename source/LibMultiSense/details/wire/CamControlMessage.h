/**
 * @file LibMultiSense/CamControlMessage.h
 *
 * This message contains the current camera configuration.
 *
 * Copyright 2013
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
 *   2012-04-14, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_CamControlMessage
#define LibMultiSense_CamControlMessage

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class CamControl {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_CMD_CAM_CONTROL;
    static CRL_CONSTEXPR VersionType VERSION = 4;

    //
    // Parameters representing the current camera configuration

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

    //
    // Additions in version 2

    float    stereoPostFilterStrength; // [0.0, 1.0]

    //
    // Additions in version 3

    bool     hdrEnabled;

    //
    // Additions in version 4

    bool  storeSettingsInFlash;  // boolean

    //
    // Constructors

    CamControl(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CamControl() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
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

        if (version >= 2)
            message & stereoPostFilterStrength;
        else
            stereoPostFilterStrength = 0.5f;

        if (version >= 3)
            message & hdrEnabled;
        else
            hdrEnabled = false;

        if (version >= 4)
            message & storeSettingsInFlash;
        else
            storeSettingsInFlash = false;
    }
};

}}}}; // namespaces

#endif
