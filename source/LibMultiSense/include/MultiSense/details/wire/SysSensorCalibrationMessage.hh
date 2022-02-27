/**
 * @file LibMultiSense/SysCameraCalibrationMessage.hh
 *
 * This message contains camera calibration
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
 *   2013-05-23, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysSensorCalibrationMessage
#define LibMultiSense_SysSensorCalibrationMessage

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysSensorCalibration {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_SYS_SENSOR_CAL;
    static CRL_CONSTEXPR VersionType VERSION = 2;


    uint8_t adc_gain[2];
    int16_t bl_offset[2];

    //version 2

    uint8_t vramp[2];

    //
    // Constructors

    SysSensorCalibration(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysSensorCalibration()
    {
        adc_gain[0] = 0;
        adc_gain[1] = 0;
        bl_offset[0] = 0;
        bl_offset[1] = 0;
        vramp[0] = 0;
        vramp[1] = 0;
    };

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        SER_ARRAY_1(adc_gain,2);
        SER_ARRAY_1(bl_offset,2);

        if(version >=2) {
        	SER_ARRAY_1(vramp,2);
        } else {
            vramp[0] = 109;
            vramp[1] = 109;
        }
    }
};

}}}} // namespaces

#endif
