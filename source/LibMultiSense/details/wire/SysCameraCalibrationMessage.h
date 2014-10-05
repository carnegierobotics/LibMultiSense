/**
 * @file LibMultiSense/SysCameraCalibrationMessage.h
 *
 * This message contains camera calibration
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
 *   2013-05-23, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysCameraCalibrationMessage
#define LibMultiSense_SysCameraCalibrationMessage

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {
    
class CameraCalData {
public:
    static CONSTEXPR VersionType VERSION = 1;

    float M[3][3];
    float D[8];
    float R[3][3];
    float P[3][4];

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        SER_ARRAY_2(M, 3, 3);
        SER_ARRAY_1(D, 8);
        SER_ARRAY_2(R, 3, 3);
        SER_ARRAY_2(P, 3, 4);
    };
};

class SysCameraCalibration {
public:
    static CONSTEXPR IdType      ID      = ID_DATA_SYS_CAMERA_CAL;
    static CONSTEXPR VersionType VERSION = 1;

    //
    // 2 MPix 

    CameraCalData left;
    CameraCalData right;

    //
    // Constructors

    SysCameraCalibration(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysCameraCalibration() {};

    //
    // Serialization routine
    
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        left.serialize(message, version);
        right.serialize(message, version);
    }
};

}}}}; // namespaces

#endif
