/**
 * @file LibMultiSense/SysDeviceInfoMessage.h
 *
 * This message contains general device information
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
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysDeviceInfoMessage
#define LibMultiSense_SysDeviceInfoMessage

#include <algorithm>
#include <string>
#include "Protocol.h"
#include "../utility/BufferStream.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class PcbInfo {
public:
    static CRL_CONSTEXPR VersionType VERSION = 1;

    std::string name;
    uint32_t    revision;

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & name;
        message & revision;
    };
};

class SysDeviceInfo {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_SYS_DEVICE_INFO;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    //
    // These constants are stored in flash on the device, do
    // not change these, only add.
    //
    // crl::multisense::DeviceInfo:: has similar constants
    // that can be changed at will (just remember to
    // map any differences when translating between
    // WIRE and API.)

    static CRL_CONSTEXPR uint8_t  MAX_PCBS = 8;

    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_SL    = 1;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S7    = 2;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_M     = 3;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S7S   = 4;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S21   = 5;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_ST21  = 6;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_BCAM             = 100;

    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV2000_GREY   = 1;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV2000_COLOR  = 2;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV4000_GREY   = 3;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV4000_COLOR  = 4;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_IMX104_COLOR   = 100;

    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_NONE = 0;
    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_SL_INTERNAL = 1;
    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_S21_EXTERNAL = 2;

    std::string key;
    std::string name;
    std::string buildDate;
    std::string serialNumber;
    uint32_t    hardwareRevision;

    uint8_t     numberOfPcbs;
    PcbInfo     pcbs[MAX_PCBS];

    std::string imagerName;
    uint32_t    imagerType;
    uint32_t    imagerWidth;
    uint32_t    imagerHeight;

    std::string lensName;
    uint32_t    lensType;
    float       nominalBaseline;          // meters
    float       nominalFocalLength;       // meters
    float       nominalRelativeAperture;  // f-stop

    uint32_t    lightingType;
    uint32_t    numberOfLights;

    std::string laserName;
    uint32_t    laserType;

    std::string motorName;
    uint32_t    motorType;
    float       motorGearReduction;

    //
    // Constructors

    SysDeviceInfo(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysDeviceInfo() :
        hardwareRevision(0),
        imagerType(0),
        imagerWidth(0),
        imagerHeight(0),
        lensType(0),
        nominalBaseline(0),
        nominalFocalLength(0),
        nominalRelativeAperture(0.0),
        lightingType(0),
        numberOfLights(0),
        laserType(0),
        motorType(0),
        motorGearReduction(0.0) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & key;
        message & name;
        message & buildDate;
        message & serialNumber;
        message & hardwareRevision;

        message & numberOfPcbs;

        uint8_t num = std::min(numberOfPcbs, (uint8_t) MAX_PCBS);
        for(uint8_t i=0; i<num; i++)
            pcbs[i].serialize(message, version);

        message & imagerName;
        message & imagerType;
        message & imagerWidth;
        message & imagerHeight;
        message & lensName;
        message & lensType;
        message & nominalBaseline;
        message & nominalFocalLength;
        message & nominalRelativeAperture;
        message & lightingType;
        message & numberOfLights;
        message & laserName;
        message & laserType;
        message & motorName;
        message & motorType;
        message & motorGearReduction;
    }
};

}}}}; // namespaces

#endif
