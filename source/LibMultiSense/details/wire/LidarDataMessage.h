/**
 * @file LibMultiSense/LidarDataMessage.h
 *
 * This message contains raw LIDAR data.
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
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_LidarDataMessage
#define LibMultiSense_LidarDataMessage

#include <typeinfo>

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ LidarDataHeader {
public:

    static CONSTEXPR IdType      ID          = ID_DATA_LIDAR_SCAN;
    static CONSTEXPR VersionType VERSION     = 1;
    static CONSTEXPR uint32_t    SCAN_POINTS = 1081;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORDPOD_FIRMWARE

    uint32_t           scanCount;
    uint32_t           timeStartSeconds;
    uint32_t           timeStartMicroSeconds;
    uint32_t           timeEndSeconds;
    uint32_t           timeEndMicroSeconds;
    int32_t            angleStart; // microradians
    int32_t            angleEnd;
    uint32_t           points;
#ifdef SENSORPOD_FIRMWARE
    uint32_t           distanceP[SCAN_POINTS];   // millimeters
    uint32_t           intensityP[SCAN_POINTS];
#else
    uint32_t          *distanceP;
    uint32_t          *intensityP;
#endif // SENSORPOD_FIRMWARE

    LidarDataHeader()
        :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        points(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class LidarData : public LidarDataHeader {
public:

    //
    // Constructors

    LidarData(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    LidarData() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & scanCount;
        message & timeStartSeconds;
        message & timeStartMicroSeconds;
        message & timeEndSeconds;
        message & timeEndMicroSeconds;
        message & angleStart;
        message & angleEnd;
        message & points;

	const uint32_t rangeSize     = sizeof(uint32_t) * points;
	const uint32_t intensitySize = sizeof(uint32_t) * points;

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {

            message.write(distanceP, rangeSize);
            message.write(intensityP, intensitySize);

        } else {

            distanceP = (uint32_t *) message.peek();
            message.seek(message.tell() + rangeSize);

            intensityP = (uint32_t *) message.peek();
            message.seek(message.tell() + intensitySize);
        }
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
