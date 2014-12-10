/**
 * @file LibMultiSense/SysDeviceModesMessage.h
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
 *   2013-06-17, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysDeviceModesMessage
#define LibMultiSense_SysDeviceModesMessage

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class DeviceMode {
public:
    uint32_t width;
    uint32_t height;
    uint32_t supportedDataSources;
    uint32_t disparities;

    DeviceMode(uint32_t w=0,
               uint32_t h=0,
               uint32_t s=0,
               uint32_t d=0) : 
        width(w),
        height(h),
        supportedDataSources(s),
        disparities(d) {};
};

class SysDeviceModes {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_SYS_DEVICE_MODES;
    static CRL_CONSTEXPR VersionType VERSION = 2;

    //
    // Available formats

    std::vector<DeviceMode> modes;

    //
    // Constructors

    SysDeviceModes(utility::BufferStreamReader& r, 
                   VersionType                  v) {serialize(r,v);};
    SysDeviceModes() {};
        
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        uint32_t length = modes.size();
        message & length;
        modes.resize(length);

        //
        // Serialize by hand here to maintain backwards compatibility with
        // pre-v2.3 firmware.

        for(uint32_t i=0; i<length; i++) {

            DeviceMode& m = modes[i];

            message & m.width;
            message & m.height;
            message & m.supportedDataSources;
            message & m.disparities; // was 'flags' in pre v2.3
        }
    }
};

}}}}; // namespaces

#endif
