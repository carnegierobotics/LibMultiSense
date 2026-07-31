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
#include <stdexcept>
#include <vector>

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

///
/// @brief A validated, non-owning view of one image in a thermal frame group
///
class ThermalImage {
public:
    const ThermalImageDescriptor& descriptor() const { return m_descriptor; }
    const uint8_t* data() const { return m_data; }
    uint32_t size() const { return m_descriptor.length; }

private:
    friend class ThermalFrameGroup;

    ThermalImage(const ThermalImageDescriptor& descriptor, const uint8_t *data)
        : m_descriptor(descriptor), m_data(data) {}

    ThermalImageDescriptor m_descriptor;
    const uint8_t *m_data;
};

///
/// @brief A validated, non-owning view of a complete thermal frame-group payload
///
/// The buffer used to construct this object must outlive the view.
///
class ThermalFrameGroup {
public:
    static CRL_CONSTEXPR uint32_t WIRE_SIZE = ThermalGroupHeader::WIRE_SIZE;

    explicit ThermalFrameGroup(utility::BufferStreamReader& stream)
    {
        const std::size_t payloadStart = stream.tell();
        const std::size_t payloadSize = stream.size() - payloadStart;
        const uint8_t *payload = static_cast<const uint8_t *>(stream.data()) + payloadStart;

        if (payloadSize < ThermalGroupHeader::WIRE_SIZE)
            throw std::runtime_error("Thermal payload is shorter than its group header");

        m_header.serialize(stream);

        if (m_header.magic != THERMAL_GROUP_MAGIC)
            throw std::runtime_error("Invalid thermal frame-group magic");
        if (m_header.version != THERMAL_GROUP_VERSION)
            throw std::runtime_error("Unsupported thermal frame-group version");
        if (m_header.payloadType != THERMAL_PAYLOAD_FRAME_GROUP)
            throw std::runtime_error("Thermal payload is not a frame group");
        if (m_header.numImages == 0 || m_header.numImages > THERMAL_MAX_IMAGERS)
            throw std::runtime_error("Invalid thermal image count");

        const uint32_t expectedHeaderLength =
            ThermalGroupHeader::WIRE_SIZE +
            static_cast<uint32_t>(m_header.numImages) * ThermalImageDescriptor::WIRE_SIZE;
        if (m_header.headerLength != expectedHeaderLength ||
            m_header.headerLength > payloadSize)
            throw std::runtime_error("Invalid thermal frame-group header length");

        uint64_t expectedOffset = m_header.headerLength;
        m_images.reserve(m_header.numImages);
        for (uint8_t index = 0; index < m_header.numImages; ++index) {
            ThermalImageDescriptor descriptor(stream);
            if (descriptor.bitsPerPixel != 8 && descriptor.bitsPerPixel != 16)
                throw std::runtime_error("Unsupported thermal image depth");
            if (descriptor.width == 0 || descriptor.height == 0)
                throw std::runtime_error("Invalid thermal image dimensions");

            const uint64_t expectedLength =
                static_cast<uint64_t>(descriptor.width) * descriptor.height *
                (descriptor.bitsPerPixel / 8u);
            const uint64_t imageEnd = static_cast<uint64_t>(descriptor.offset) +
                                      descriptor.length;
            if (descriptor.offset != expectedOffset ||
                descriptor.length != expectedLength || imageEnd > payloadSize)
                throw std::runtime_error("Invalid thermal image descriptor");

            m_images.push_back(ThermalImage(descriptor, payload + descriptor.offset));
            expectedOffset = imageEnd;
        }

        if (expectedOffset != payloadSize)
            throw std::runtime_error("Thermal image descriptors do not cover the payload");
    }

    const ThermalGroupHeader& header() const { return m_header; }
    std::size_t size() const { return m_images.size(); }
    const ThermalImage& at(std::size_t index) const { return m_images.at(index); }

private:
    ThermalGroupHeader m_header;
    std::vector<ThermalImage> m_images;
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
