/**
 * @file LibMultiSense/details/wire/Protocol.hh
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
 *   2013-05-07, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_details_wire_protocol
#define LibMultiSense_details_wire_protocol

#include <stdint.h>

#include "../utility/Portability.hh"

// Expose some configuration constants to the preprocessor (at least until C++17)
#ifndef MULTISENSE_WIRE_BITS_PER_PIXEL
#define MULTISENSE_WIRE_BITS_PER_PIXEL (12)
#endif

#ifndef MULTISENSE_API_BITS_PER_PIXEL
#define MULTISENSE_API_BITS_PER_PIXEL (16)
#endif

namespace crl {
namespace multisense {
namespace details {
namespace wire {

//
// Some message headers are directly used by sensor firmware

#ifdef SENSORPOD_FIRMWARE
#define WIRE_HEADER_ATTRIBS_ __attribute__ ((__packed__))
#else
#define WIRE_HEADER_ATTRIBS_
#endif // SENSORPOD_FIRMWARE

//
// The size of the combined headers

static CRL_CONSTEXPR uint8_t COMBINED_HEADER_LENGTH = 60;

//
// The magic number and version

static CRL_CONSTEXPR uint16_t HEADER_MAGIC   = 0xadad;
static CRL_CONSTEXPR uint16_t HEADER_VERSION = 0x0100;

//
// The protocol group (TODO: define CRL-wide)

static CRL_CONSTEXPR uint16_t HEADER_GROUP   = 0x0001;

//
// The packet header structure

#if defined (_MSC_VER)
#pragma pack(push, 1)
typedef struct {
#else
typedef struct __attribute__ ((__packed__)) {
#endif
    //
    // The magic number

    uint16_t magic;

    //
    // The protocol version

    uint16_t version;

    //
    // The protocol group

    uint16_t group;

    //
    // Protocol flags

    uint16_t flags;

    //
    // The message sequence identifier

    uint16_t sequenceIdentifier;

    //
    // The total size of the message

    uint32_t messageLength;

    //
    // Offset of this packet's payload

    uint32_t byteOffset;

} Header;
#if defined (_MSC_VER)
#pragma pack(pop)
#endif

//
// Types for message IDs and versions

typedef uint16_t IdType;
typedef uint16_t VersionType;

//
// Every command responsds with an ID_ACK message,
// regardless if a data message is also following.

//
// TODO: this message set is still awkward in places:
//       - Missing 1:1 get/set
//       - Some "Data" messages are also commands
//       - Duplicated information (CAM_GET_CONFIG, SYS_GET_CAMERA_CAL, etc.)

//
// [N]acks

static CRL_CONSTEXPR IdType ID_ACK = 0x0001;

//
// Commands

static CRL_CONSTEXPR IdType ID_CMD_GET_VERSION              = 0x0002;
static CRL_CONSTEXPR IdType ID_CMD_GET_STATUS               = 0x0003;
static CRL_CONSTEXPR IdType ID_CMD_CAM_GET_CONFIG           = 0x0004;
static CRL_CONSTEXPR IdType ID_CMD_CAM_CONTROL              = 0x0007;
static CRL_CONSTEXPR IdType ID_CMD_CAM_GET_HISTORY          = 0x0008;
static CRL_CONSTEXPR IdType ID_CMD_CAM_SET_HDR              = 0x000b;
static CRL_CONSTEXPR IdType ID_CMD_CAM_SET_RESOLUTION       = 0x000c;
static CRL_CONSTEXPR IdType ID_CMD_LIDAR_GET_CONFIG         = 0x000d;
static CRL_CONSTEXPR IdType ID_CMD_LIDAR_SET_MOTOR          = 0x0010;
static CRL_CONSTEXPR IdType ID_CMD_LED_GET_STATUS           = 0x0012;
static CRL_CONSTEXPR IdType ID_CMD_LED_SET                  = 0x0013;
static CRL_CONSTEXPR IdType ID_CMD_SYS_MTU                  = 0x0014;
static CRL_CONSTEXPR IdType ID_CMD_SYS_FLASH_OP             = 0x0015;
static CRL_CONSTEXPR IdType ID_CMD_SYS_SET_NETWORK          = 0x0016;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_DEVICE_INFO      = 0x0017;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_CAMERA_CAL       = 0x0018;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_LIDAR_CAL        = 0x0019;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_MTU              = 0x001a;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_NETWORK          = 0x001b;
static CRL_CONSTEXPR IdType ID_CMD_STREAM_CONTROL           = 0x001c;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_DEVICE_MODES     = 0x001d;
static CRL_CONSTEXPR IdType ID_CMD_CAM_SET_TRIGGER_SOURCE   = 0x001e;
static CRL_CONSTEXPR IdType ID_CMD_IMU_GET_INFO             = 0x001f;
static CRL_CONSTEXPR IdType ID_CMD_IMU_GET_CONFIG           = 0x0020;
static CRL_CONSTEXPR IdType ID_CMD_SYS_TEST_MTU             = 0x0021;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_DIRECTED_STREAMS = 0x0022;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_SENSOR_CAL       = 0x0023;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_EXTERNAL_CAL     = 0x0024;
static CRL_CONSTEXPR IdType ID_CMD_LED_GET_SENSOR_STATUS    = 0x0025;
static CRL_CONSTEXPR IdType ID_CMD_SYS_SET_TRANSMIT_DELAY   = 0x0026;
static CRL_CONSTEXPR IdType ID_CMD_SYS_GET_TRANSMIT_DELAY   = 0x0027;
static CRL_CONSTEXPR IdType ID_CMD_SYS_MOTOR_POLL           = 0x0028;
static CRL_CONSTEXPR IdType ID_CMD_SYS_SET_PTP              = 0x0029;
static CRL_CONSTEXPR IdType ID_CMD_CAM_AUX_CONTROL          = 0x002a;
static CRL_CONSTEXPR IdType ID_CMD_CAM_GET_AUX_CONFIG       = 0x002b;
static CRL_CONSTEXPR IdType ID_CMD_REMOTE_HEAD_GET_CONFIG   = 0x002c;
static CRL_CONSTEXPR IdType ID_CMD_REMOTE_HEAD_CONTROL      = 0x002d;

//
// Data

static CRL_CONSTEXPR IdType ID_DATA_VERSION                            = 0x0102;
static CRL_CONSTEXPR IdType ID_DATA_STATUS                             = 0x0103;
static CRL_CONSTEXPR IdType ID_DATA_CAM_CONFIG                         = 0x0104;
static CRL_CONSTEXPR IdType ID_DATA_CAM_HISTORY                        = 0x0105;
static CRL_CONSTEXPR IdType ID_DATA_LIDAR_CONFIG                       = 0x0108;
static CRL_CONSTEXPR IdType ID_DATA_LIDAR_SCAN                         = 0x0109;
static CRL_CONSTEXPR IdType ID_DATA_LED_STATUS                         = 0x010a;
static CRL_CONSTEXPR IdType ID_DATA_SYS_FLASH_RESPONSE                 = 0x010b;
static CRL_CONSTEXPR IdType ID_DATA_SYS_DEVICE_INFO                    = 0x010c;
static CRL_CONSTEXPR IdType ID_DATA_SYS_CAMERA_CAL                     = 0x010d;
static CRL_CONSTEXPR IdType ID_DATA_SYS_LIDAR_CAL                      = 0x010e;
static CRL_CONSTEXPR IdType ID_DATA_IMAGE_META                         = 0x010f;
static CRL_CONSTEXPR IdType ID_DATA_IMAGE                              = 0x0110;
static CRL_CONSTEXPR IdType ID_DATA_DISPARITY                          = 0x0111;
static CRL_CONSTEXPR IdType ID_DATA_SYS_DEVICE_MODES                   = 0x0112;
static CRL_CONSTEXPR IdType ID_DATA_SYS_PPS                            = 0x0113;
static CRL_CONSTEXPR IdType ID_DATA_IMU                                = 0x0114;
static CRL_CONSTEXPR IdType ID_DATA_IMU_INFO                           = 0x0115;
static CRL_CONSTEXPR IdType ID_DATA_IMU_CONFIG                         = 0x0116;
static CRL_CONSTEXPR IdType ID_DATA_SYS_TEST_MTU_RESPONSE              = 0x0117;
static CRL_CONSTEXPR IdType ID_DATA_JPEG_IMAGE                         = 0x0118;
static CRL_CONSTEXPR IdType ID_DATA_SYS_DIRECTED_STREAMS               = 0x0119;
static CRL_CONSTEXPR IdType ID_DATA_SYS_SENSOR_CAL                     = 0x011a;
static CRL_CONSTEXPR IdType ID_DATA_SYS_EXTERNAL_CAL                   = 0x011b;
static CRL_CONSTEXPR IdType ID_DATA_LED_SENSOR_STATUS                  = 0x011c;
static CRL_CONSTEXPR IdType ID_DATA_SYS_MOTOR_POLL                     = 0x011d;
static CRL_CONSTEXPR IdType ID_DATA_EXPOSURE_CONFIG                    = 0x011e;
static CRL_CONSTEXPR IdType ID_DATA_GROUND_SURFACE_SPLINE_DATA_MESSAGE = 0x011f;
static CRL_CONSTEXPR IdType ID_DATA_COMPRESSED_IMAGE                   = 0x0120;
static CRL_CONSTEXPR IdType ID_DATA_SYS_GROUND_SURFACE_PARAM           = 0x0121;
static CRL_CONSTEXPR IdType ID_DATA_APRILTAG_DETECTIONS_MESSAGE        = 0x0122;
static CRL_CONSTEXPR IdType ID_DATA_SYS_APRILTAG_PARAM                 = 0x0123;
static CRL_CONSTEXPR IdType ID_DATA_CAM_AUX_CONFIG                     = 0x0124;
static CRL_CONSTEXPR IdType ID_CMD_REMOTE_HEAD_CONFIG                  = 0x0125;

//
// Data sources

typedef uint32_t SourceType;

static CRL_CONSTEXPR SourceType SOURCE_UNKNOWN                     = 0;
static CRL_CONSTEXPR SourceType SOURCE_RAW_LEFT                    = (1U<<0);
static CRL_CONSTEXPR SourceType SOURCE_RAW_RIGHT                   = (1U<<1);
static CRL_CONSTEXPR SourceType SOURCE_LUMA_LEFT                   = (1U<<2);
static CRL_CONSTEXPR SourceType SOURCE_LUMA_RIGHT                  = (1U<<3);
static CRL_CONSTEXPR SourceType SOURCE_LUMA_RECT_LEFT              = (1U<<4);
static CRL_CONSTEXPR SourceType SOURCE_LUMA_RECT_RIGHT             = (1U<<5);
static CRL_CONSTEXPR SourceType SOURCE_CHROMA_LEFT                 = (1U<<6);
static CRL_CONSTEXPR SourceType SOURCE_CHROMA_RIGHT                = (1U<<7);
static CRL_CONSTEXPR SourceType SOURCE_CHROMA_RECT_AUX             = (1U<<8);
static CRL_CONSTEXPR SourceType SOURCE_DISPARITY                   = (1U<<10);
static CRL_CONSTEXPR SourceType SOURCE_DISPARITY_LEFT              = (1U<<10); // same as SOURCE_DISPARITY
static CRL_CONSTEXPR SourceType SOURCE_DISPARITY_RIGHT             = (1U<<11);
static CRL_CONSTEXPR SourceType SOURCE_DISPARITY_COST              = (1U<<12);
static CRL_CONSTEXPR SourceType SOURCE_JPEG_LEFT                   = (1U<<16);
static CRL_CONSTEXPR SourceType SOURCE_RGB_LEFT                    = (1U<<17);
static CRL_CONSTEXPR SourceType SOURCE_GROUND_SURFACE_SPLINE_DATA  = (1U<<20);
static CRL_CONSTEXPR SourceType SOURCE_GROUND_SURFACE_CLASS_IMAGE  = (1U<<22);
static CRL_CONSTEXPR SourceType SOURCE_APRILTAG_DETECTIONS         = (1U<<21);
static CRL_CONSTEXPR SourceType SOURCE_SLB_MOTOR                   = (1U<<23);
static CRL_CONSTEXPR SourceType SOURCE_LIDAR_SCAN                  = (1U<<24);
static CRL_CONSTEXPR SourceType SOURCE_IMU                         = (1U<<25);
static CRL_CONSTEXPR SourceType SOURCE_PPS                         = (1U<<26);
static CRL_CONSTEXPR SourceType SOURCE_RAW_AUX                     = (1U<<27);
static CRL_CONSTEXPR SourceType SOURCE_LUMA_AUX                    = (1U<<28);
static CRL_CONSTEXPR SourceType SOURCE_LUMA_RECT_AUX               = (1U<<29);
static CRL_CONSTEXPR SourceType SOURCE_CHROMA_AUX                  = (1U<<30);
static CRL_CONSTEXPR SourceType SOURCE_DISPARITY_AUX               = (1U<<31);
static CRL_CONSTEXPR SourceType SOURCE_COMPRESSED_LEFT             = (1U<<9);
static CRL_CONSTEXPR SourceType SOURCE_COMPRESSED_RIGHT            = (1U<<13);
static CRL_CONSTEXPR SourceType SOURCE_COMPRESSED_AUX              = (1U<<14);
static CRL_CONSTEXPR SourceType SOURCE_COMPRESSED_RECTIFIED_LEFT   = (1U<<15);
static CRL_CONSTEXPR SourceType SOURCE_COMPRESSED_RECTIFIED_RIGHT  = (1U<<16); // same as SOURCE_JPEG_LEFT
static CRL_CONSTEXPR SourceType SOURCE_COMPRESSED_RECTIFIED_AUX    = (1U<<17); // same as SOURCE_RGB_LEFT

static CRL_CONSTEXPR SourceType SOURCE_IMAGES            = (SOURCE_RAW_LEFT        |
                                                            SOURCE_RAW_RIGHT       |
                                                            SOURCE_RAW_AUX         |
                                                            SOURCE_LUMA_LEFT       |
                                                            SOURCE_LUMA_RIGHT      |
                                                            SOURCE_LUMA_AUX        |
                                                            SOURCE_LUMA_RECT_LEFT  |
                                                            SOURCE_LUMA_RECT_RIGHT |
                                                            SOURCE_LUMA_RECT_AUX   |
                                                            SOURCE_CHROMA_LEFT     |
                                                            SOURCE_CHROMA_RIGHT    |
                                                            SOURCE_CHROMA_AUX      |
                                                            SOURCE_CHROMA_RECT_AUX |
                                                            SOURCE_DISPARITY_LEFT  |
                                                            SOURCE_DISPARITY_RIGHT |
                                                            SOURCE_DISPARITY_COST  |
                                                            SOURCE_DISPARITY_AUX   |
                                                            SOURCE_JPEG_LEFT       |
                                                            SOURCE_RGB_LEFT        |
                                                            SOURCE_GROUND_SURFACE_CLASS_IMAGE|
                                                            SOURCE_APRILTAG_DETECTIONS       |
                                                            SOURCE_COMPRESSED_LEFT           |
                                                            SOURCE_COMPRESSED_RIGHT          |
                                                            SOURCE_COMPRESSED_AUX            |
                                                            SOURCE_COMPRESSED_RECTIFIED_LEFT |
                                                            SOURCE_COMPRESSED_RECTIFIED_RIGHT|
                                                            SOURCE_COMPRESSED_RECTIFIED_AUX
                                                          );

//
// Exposure config

static CRL_CONSTEXPR SourceType Default_Exposure_Source = SOURCE_LUMA_LEFT;
static CRL_CONSTEXPR float Default_Target_Intensity = 0.5f;
static CRL_CONSTEXPR float Default_Gamma = 2.0f;
static CRL_CONSTEXPR float Default_Gain = 1.0f;
//
// Some helper macros

#define MSG_ID(x)  ((wire::IdType)(x))
#define MSG_VER(x) ((wire::VersionType)(x))

#define SER_ARRAY_1(a_,n_)                    \
    for(uint32_t i_=0; i_<(n_); i_++)         \
        message & (a_)[i_];                   \

#define SER_ARRAY_2(a_,n_,m_)                 \
    for(uint32_t i_=0; i_<(n_); i_++)         \
        for(uint32_t j_=0; j_<(m_); j_++)     \
            message & (a_)[(i_)][(j_)];       \

#define CPY_ARRAY_1(d_,s_,n_)                   \
    for(uint32_t i_=0; i_<(n_); i_++)           \
        (d_)[i_] = (s_)[i_];                    \

#define CPY_ARRAY_2(d_,s_,n_,m_)                \
    for(uint32_t i_=0; i_<(n_); i_++)           \
        for(uint32_t j_=0; j_<(m_); j_++)       \
            (d_)[i_][j_] = (s_)[i_][j_];        \

}}}} // namespaces

#endif
