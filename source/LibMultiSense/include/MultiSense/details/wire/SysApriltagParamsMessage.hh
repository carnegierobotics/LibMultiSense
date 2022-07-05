/**
 * @file LibMultiSense/SysApriltagParamsMessage.hh
 *
 * This message contains general device information
 *
 * Copyright 2022
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
 *   2022-02-08, drobinson@carnegierobotics.com, IRAD, created file.
 **/

#ifndef LibMultiSense_SysApriltagParamsMessage
#define LibMultiSense_SysApriltagParamsMessage

#include <algorithm>
#include <string>
#include "Protocol.hh"
#include "../utility/BufferStream.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysApriltagParams {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_DATA_SYS_APRILTAG_PARAM;
    static CRL_CONSTEXPR VersionType VERSION = 1;

    std::string family;
    uint8_t max_hamming;
    double quad_detection_blur_sigma;
    double quad_detection_decimate;
    uint64_t min_border_width;
    bool refine_quad_edges;
    double decode_sharpening;

    //
    // Constructors
    //
    SysApriltagParams(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysApriltagParams()
    {
        family = "tagStandard52h13";
        max_hamming = 0;
        quad_detection_blur_sigma = 0.75;
        quad_detection_decimate = 1.0;
        min_border_width = 5;
        refine_quad_edges = false;
        decode_sharpening = 0.25;
    };

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        (void) version;

        message & family;
        message & max_hamming;
        message & quad_detection_blur_sigma;
        message & quad_detection_decimate;
        message & min_border_width;
        message & refine_quad_edges;
        message & decode_sharpening;
    }

};

}}}} // namespaces

#endif
