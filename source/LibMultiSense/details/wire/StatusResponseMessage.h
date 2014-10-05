/**
 * @file LibMultiSense/StatusResponseMessage.h
 *
 * This message contains status information.
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

#ifndef LibMultiSense_StatusResponseMessage
#define LibMultiSense_StatusResponseMessage

#include "details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class StatusResponse {
public:
    static CONSTEXPR IdType      ID                  = ID_DATA_STATUS;
    static CONSTEXPR VersionType VERSION             = 2;
    static CONSTEXPR float       INVALID_TEMPERATURE = -99999.0;

    //
    // Subsytem status

    static CONSTEXPR uint32_t STATUS_GENERAL_OK     = (1<<0);
    static CONSTEXPR uint32_t STATUS_LASER_OK       = (1<<1);
    static CONSTEXPR uint32_t STATUS_LASER_MOTOR_OK = (1<<2);
    static CONSTEXPR uint32_t STATUS_CAMERAS_OK     = (1<<3);
    static CONSTEXPR uint32_t STATUS_IMU_OK         = (1<<4);

    //
    // The reported uptime for the system
    //

    utility::TimeStamp uptime;
    uint32_t           status;
    float              temperature0; // celsius
    float              temperature1;
    
    //
    // Version 2 additions

    float              temperature2; // celsius
    float              temperature3;

    float              inputVolts;    // volts
    float              inputCurrent;  // amps
    float              fpgaPower;     // watts
    float              logicPower;
    float              imagerPower;

    //
    // Constructors

    StatusResponse(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    StatusResponse() : uptime(), 
                       status(0), 
                       temperature0(INVALID_TEMPERATURE), 
                       temperature1(INVALID_TEMPERATURE),
                       temperature2(INVALID_TEMPERATURE),
                       temperature3(INVALID_TEMPERATURE),
                       inputVolts(-1.0),
                       inputCurrent(-1.0),
                       fpgaPower(-1.0),
                       logicPower(-1.0),
                       imagerPower(-1.0) {};
                       
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & uptime;
        message & status;
        message & temperature0;
        message & temperature1;

        if (version >= 2) {
            message & temperature2;
            message & temperature3;
            message & inputVolts;
            message & inputCurrent;
            message & fpgaPower;
            message & logicPower;
            message & imagerPower;
        }   
    }
};

}}}}; // namespaces

#endif
