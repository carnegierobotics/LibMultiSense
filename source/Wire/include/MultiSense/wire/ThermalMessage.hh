/**
 * @file ThermalMessage.hh
 *
 * Wire definitions for the T6 thermal secondary application.
 *
 * Copyright 2026
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
 **/

#ifndef LibMultiSense_ThermalMessage
#define LibMultiSense_ThermalMessage

#include <stdint.h>

#include "MultiSense/utility/BufferStream.hh"
#include "MultiSense/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

static CRL_CONSTEXPR uint32_t THERMAL_GROUP_MAGIC = 0x43543654u;
static CRL_CONSTEXPR uint16_t THERMAL_GROUP_VERSION = 1;
static CRL_CONSTEXPR uint8_t THERMAL_MAX_IMAGERS = 8;

enum ThermalPayloadType : uint16_t {
    THERMAL_PAYLOAD_FRAME_GROUP = 1,
    THERMAL_PAYLOAD_CALIBRATION = 2,
};

enum ThermalImageFlag : uint8_t {
    THERMAL_IMAGE_FLAG_RECTIFIED = (1u << 0),
};

enum ThermalControlCommand : uint16_t {
    THERMAL_CONTROL_SET_RECTIFIED = 1,
    THERMAL_CONTROL_SET_BITS_PER_PIXEL = 2,
    THERMAL_CONTROL_SET_POST_PROCESSING = 3,
};

class ThermalImageDescriptor {
public:
    static CRL_CONSTEXPR uint32_t WIRE_SIZE = 16;

    uint32_t offset;
    uint32_t length;
    uint16_t width;
    uint16_t height;
    uint8_t bitsPerPixel;
    uint8_t flags;
    uint8_t imagerId;
    uint8_t reserved;

    ThermalImageDescriptor()
        : offset(0), length(0), width(0), height(0), bitsPerPixel(0),
          flags(0), imagerId(0), reserved(0) {}

    explicit ThermalImageDescriptor(utility::BufferStreamReader& stream)
        : ThermalImageDescriptor() { serialize(stream); }

    template<class Archive>
    void serialize(Archive& stream)
    {
        stream & offset;
        stream & length;
        stream & width;
        stream & height;
        stream & bitsPerPixel;
        stream & flags;
        stream & imagerId;
        stream & reserved;
    }
};

class ThermalGroupHeader {
public:
    static CRL_CONSTEXPR uint32_t WIRE_SIZE = 36;

    uint32_t magic;
    uint16_t version;
    uint16_t payloadType;
    uint32_t headerLength;
    int64_t frameId;
    uint32_t timeSeconds;
    uint32_t timeMicroseconds;
    uint8_t ptpLocked;
    uint8_t numImages;
    uint8_t reserved[2];
    uint32_t imagerEnableMask;

    ThermalGroupHeader()
        : magic(0), version(0), payloadType(0), headerLength(0), frameId(0),
          timeSeconds(0), timeMicroseconds(0), ptpLocked(0), numImages(0),
          reserved{0, 0}, imagerEnableMask(0) {}

    explicit ThermalGroupHeader(utility::BufferStreamReader& stream)
        : ThermalGroupHeader() { serialize(stream); }

    template<class Archive>
    void serialize(Archive& stream)
    {
        stream & magic;
        stream & version;
        stream & payloadType;
        stream & headerLength;
        stream & frameId;
        stream & timeSeconds;
        stream & timeMicroseconds;
        stream & ptpLocked;
        stream & numImages;
        stream & reserved[0];
        stream & reserved[1];
        stream & imagerEnableMask;
    }
};

class ThermalControl {
public:
    static CRL_CONSTEXPR uint32_t WIRE_SIZE = 12;

    uint32_t magic;
    uint16_t version;
    uint16_t command;
    uint32_t value;

    ThermalControl()
        : magic(THERMAL_GROUP_MAGIC), version(THERMAL_GROUP_VERSION),
          command(0), value(0) {}

    explicit ThermalControl(utility::BufferStreamReader& stream)
        : ThermalControl() { serialize(stream); }

    template<class Archive>
    void serialize(Archive& stream)
    {
        stream & magic;
        stream & version;
        stream & command;
        stream & value;
    }
};

class ThermalConfig {
public:
    static CRL_CONSTEXPR uint32_t WIRE_SIZE = 20;

    uint32_t magic;
    uint16_t version;
    uint16_t reserved0;
    uint8_t rectified;
    uint8_t bitsPerPixel;
    uint8_t maxImagers;
    uint8_t reserved1;
    uint32_t imagerEnableMask;
    uint16_t width;
    uint16_t height;

    ThermalConfig()
        : magic(0), version(0), reserved0(0), rectified(0), bitsPerPixel(0),
          maxImagers(0), reserved1(0), imagerEnableMask(0), width(0), height(0) {}

    explicit ThermalConfig(utility::BufferStreamReader& stream)
        : ThermalConfig() { serialize(stream); }

    template<class Archive>
    void serialize(Archive& stream)
    {
        stream & magic;
        stream & version;
        stream & reserved0;
        stream & rectified;
        stream & bitsPerPixel;
        stream & maxImagers;
        stream & reserved1;
        stream & imagerEnableMask;
        stream & width;
        stream & height;
    }
};

}}}} // namespaces

#endif // LibMultiSense_ThermalMessage
