/**
 * @file LibMultiSense/details/channel.cc
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
 *   2013-04-25, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include "MultiSense/details/channel.hh"
#include "MultiSense/details/query.hh"

#include "MultiSense/details/wire/DisparityMessage.hh"
#include "MultiSense/details/wire/SysMtuMessage.hh"
#include "MultiSense/details/wire/SysGetMtuMessage.hh"
#include "MultiSense/details/wire/StatusRequestMessage.hh"
#include "MultiSense/details/wire/StatusResponseMessage.hh"
#include "MultiSense/details/wire/VersionRequestMessage.hh"
#include "MultiSense/details/wire/SysDeviceInfoMessage.hh"

#include "MultiSense/details/utility/Functional.hh"

#ifdef WIN32
#include <ws2tcpip.h>
#else
#include <netdb.h>
#endif
#include <errno.h>
#include <fcntl.h>

namespace crl {
namespace multisense {
namespace details {

//
// Implementation constructor

impl::impl(const std::string& address, const RemoteHeadChannel& cameraId, const std::string& ifName) :
    m_serverSocket(INVALID_SOCKET),
    m_serverSocketPort(0),
    m_sensorAddress(),
    m_sensorMtu(MAX_MTU_SIZE),
    m_incomingBuffer(MAX_MTU_SIZE),
    m_txSeqId(0),
    m_lastRxSeqId(-1),
    m_unWrappedRxSeqId(0),
    m_udpTrackerCache(UDP_TRACKER_CACHE_DEPTH),
    m_rxLargeBufferPool(),
    m_rxSmallBufferPool(),
    m_imageMetaCache(IMAGE_META_CACHE_DEPTH),
    m_udpAssemblerMap(),
    m_dispatchLock(),
    m_streamLock(),
    m_threadsRunning(false),
    m_rxThreadP(NULL),
    m_rxLock(),
    m_statusThreadP(NULL),
    m_imageListeners(),
    m_lidarListeners(),
    m_ppsListeners(),
    m_imuListeners(),
    m_compressedImageListeners(),
    m_watch(),
    m_messages(),
    m_streamsEnabled(0),
    m_timeLock(),
    m_timeOffsetInit(false),
    m_timeOffset(0),
    m_networkTimeSyncEnabled(true),
    m_ptpTimeSyncEnabled(false),
    m_sensorVersion()
{

#if WIN32
    WSADATA wsaData;
    int result = WSAStartup (MAKEWORD (0x02, 0x02), &wsaData);
    if (result != 0)
        CRL_EXCEPTION("WSAStartup() failed: %d", result);

#endif

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = 0;

    const int addrstatus = getaddrinfo(address.c_str(), NULL, &hints, &res);
    if (addrstatus != 0 || res == NULL)
        CRL_EXCEPTION("unable to resolve \"%s\": %s", address.c_str(), strerror(errno));

    in_addr addr;
    memcpy(&addr, &(((struct sockaddr_in *)(res->ai_addr))->sin_addr), sizeof(in_addr));

    memset(&m_sensorAddress, 0, sizeof(m_sensorAddress));

    m_sensorAddress.sin_family = AF_INET;
    m_sensorAddress.sin_port   = htons(DEFAULT_SENSOR_TX_PORT + static_cast<uint16_t>(cameraId + 1));
    m_sensorAddress.sin_addr   = addr;

    freeaddrinfo(res);

    //
    // Create a pool of RX buffers

    uint32_t largeBufferRetry = 0;
    for(uint32_t i=0; i<RX_POOL_LARGE_BUFFER_COUNT;)
    {
        try {
            m_rxLargeBufferPool.push_back(new utility::BufferStreamWriter(RX_POOL_LARGE_BUFFER_SIZE));
            i++;
            largeBufferRetry = 0;
        }
        catch (const std::exception &e) {
            CRL_DEBUG("Failed to allocate memory (will sleep and try again): %s", e.what());
            usleep(static_cast<unsigned int> (10000));
            largeBufferRetry++;

            if (largeBufferRetry >= MAX_BUFFER_ALLOCATION_RETRIES)
                throw e;
        }
    }

    uint32_t smallBufferRetry = 0;
    for(uint32_t i=0; i<RX_POOL_SMALL_BUFFER_COUNT;)
    {
        try {
            m_rxSmallBufferPool.push_back(new utility::BufferStreamWriter(RX_POOL_SMALL_BUFFER_SIZE));
            i++;
            smallBufferRetry = 0;
        }
        catch (const std::exception &e) {
            CRL_DEBUG("Failed to allocate memory (will sleep and try again): %s", e.what());
            usleep(static_cast<unsigned int> (10000));
            smallBufferRetry++;

            if (smallBufferRetry >= MAX_BUFFER_ALLOCATION_RETRIES)
                throw e;
        }
    }

    //
    // Bind to the port

    try {
        bind(ifName);
    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        cleanup();
        throw e;
    }

    //
    // Register any special UDP reassemblers

    m_udpAssemblerMap[MSG_ID(wire::Disparity::ID)] = wire::Disparity::assembler;

    //
    // Create UDP reception thread

    m_threadsRunning = true;
    m_rxThreadP      = new utility::Thread(rxThread, this);

    //
    // Request the current operating MTU of the device

    wire::SysMtu mtu;

    Status status = waitData(wire::SysGetMtu(), mtu);
    if (Status_Ok != status) {
        cleanup();
        CRL_EXCEPTION("failed to establish comms with the sensor at \"%s\", with remote head enum %d",
                      address.c_str(), cameraId);
    } else {

        //
        // Use the same MTU for TX

        m_sensorMtu = mtu.mtu;
    }

    //
    // Request version info from the device

    status = waitData(wire::VersionRequest(), m_sensorVersion);
    if (Status_Ok != status) {
        cleanup();
        CRL_EXCEPTION("failed to request version info from sensor at \"%s\"",
                      address.c_str());
    }

    //
    // Create status thread

    m_statusThreadP = new utility::Thread(statusThread, this);
}

//
// Implementation cleanup

void impl::cleanup()
{
    m_threadsRunning = false;

    if (m_rxThreadP)
        delete m_rxThreadP;
    if (m_statusThreadP)
        delete m_statusThreadP;

    std::list<ImageListener*>::const_iterator iti;
    for(iti  = m_imageListeners.begin();
        iti != m_imageListeners.end();
        iti ++)
        delete *iti;
    std::list<LidarListener*>::const_iterator itl;
    for(itl  = m_lidarListeners.begin();
        itl != m_lidarListeners.end();
        itl ++)
        delete *itl;
    std::list<PpsListener*>::const_iterator itp;
    for(itp  = m_ppsListeners.begin();
        itp != m_ppsListeners.end();
        itp ++)
        delete *itp;
    std::list<ImuListener*>::const_iterator itm;
    for(itm  = m_imuListeners.begin();
        itm != m_imuListeners.end();
        itm ++)
        delete *itm;
    std::list<CompressedImageListener*>::const_iterator itc;
    for(itc  = m_compressedImageListeners.begin();
        itc != m_compressedImageListeners.end();
        itc ++)
        delete *itc;

    BufferPool::const_iterator it;
    for(it  = m_rxLargeBufferPool.begin();
        it != m_rxLargeBufferPool.end();
        ++it)
        delete *it;
    for(it  = m_rxSmallBufferPool.begin();
        it != m_rxSmallBufferPool.end();
        ++it)
        delete *it;

    m_imageListeners.clear();
    m_lidarListeners.clear();
    m_ppsListeners.clear();
    m_imuListeners.clear();
    m_compressedImageListeners.clear();
    m_rxLargeBufferPool.clear();
    m_rxSmallBufferPool.clear();

    if (m_serverSocket > 0)
        closesocket(m_serverSocket);

#if WIN32
    WSACleanup ();
#endif
}

//
// Implementation destructor

impl::~impl()
{
    cleanup();
}

//
// Binds the communications channel, preparing it to send/receive data
// over the network.

void impl::bind(const std::string& ifName)
{
    //
    // Create the socket.

    m_serverSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_serverSocket < 0)
        CRL_EXCEPTION("failed to create the UDP socket: %s",
                      strerror(errno));

    #if __linux__
        //
        // Bind to spcific interface if specified
        if (!ifName.empty()){
            if (0 != setsockopt(m_serverSocket, SOL_SOCKET, SO_BINDTODEVICE,  ifName.c_str(), ifName.size())){
                CRL_EXCEPTION("Failed to bind to device %s. Error: %s", ifName.c_str(),
                              strerror(errno));
            }
        }
    #else
        if (!ifName.empty())
            CRL_DEBUG("User specified binding to adapter %s, but this feature is only supported under linux. Ignoring bind to specific adapter", ifName.c_str());
    #endif

    //
    // Turn non-blocking on.
#if WIN32
    u_long ioctl_arg = 1;
    if (0 != ioctlsocket(m_serverSocket, FIONBIO, &ioctl_arg))
        CRL_EXCEPTION("failed to make a socket non-blocking: %d",WSAGetLastError ());
#else
    const int flags = fcntl(m_serverSocket, F_GETFL, 0);

    if (0 != fcntl(m_serverSocket, F_SETFL, flags | O_NONBLOCK))
        CRL_EXCEPTION("failed to make a socket non-blocking: %s",
                      strerror(errno));
#endif

    //
    // Allow reusing sockets.

    int reuseSocket = 1;

    if (0 != setsockopt(m_serverSocket, SOL_SOCKET, SO_REUSEADDR, (char*) &reuseSocket,
                        sizeof(reuseSocket)))
        CRL_EXCEPTION("failed to turn on socket reuse flag: %s",
                      strerror(errno));

    //
    // We want very large buffers to store several images

#if __APPLE__
    // MacOS cannot reliably allocate a buffer larger than this
    int bufferSize = 4 * 1024 * 1024;
#else
    int bufferSize = 48 * 1024 * 1024;
#endif

    if (0 != setsockopt(m_serverSocket, SOL_SOCKET, SO_RCVBUF, (char*) &bufferSize,
                        sizeof(bufferSize)) ||
        0 != setsockopt(m_serverSocket, SOL_SOCKET, SO_SNDBUF, (char*) &bufferSize,
                        sizeof(bufferSize)))
        CRL_EXCEPTION("failed to adjust socket buffer sizes (%d bytes): %s",
                      bufferSize, strerror(errno));

    //
    // Bind the connection to the port.

    struct sockaddr_in address;

    address.sin_family      = AF_INET;
    address.sin_port        = htons(0); // system assigned
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (0 != ::bind(m_serverSocket, (struct sockaddr*) &address, sizeof(address)))
        CRL_EXCEPTION("failed to bind the server socket to system-assigned port: %s",
                      strerror(errno));

    //
    // Retrieve the system assigned local UDP port
#if WIN32
    int len = sizeof(address);
#else
    socklen_t len = sizeof(address);
#endif
    if (0 != getsockname(m_serverSocket, (struct sockaddr*) &address, &len))
        CRL_EXCEPTION("getsockname() failed: %s", strerror(errno));
    m_serverSocketPort = htons(address.sin_port);
}

//
// Publish a stream to the sensor

void impl::publish(const utility::BufferStreamWriter& stream)
{
    //
    // Install the header

    wire::Header& header = *(reinterpret_cast<wire::Header*>(stream.data()));

    header.magic              = wire::HEADER_MAGIC;
    header.version            = wire::HEADER_VERSION;
    header.group              = wire::HEADER_GROUP;
    header.flags              = 0;
#if WIN32
    // TBD: This returns the post-incremented value
    header.sequenceIdentifier = InterlockedIncrement16((short*)&m_txSeqId);
#else
    // TBD: This returns the pre-incremented value
    header.sequenceIdentifier = __sync_fetch_and_add(&m_txSeqId, 1);
#endif
    header.messageLength      = static_cast<uint32_t> (stream.tell() - sizeof(wire::Header));
    header.byteOffset         = 0;

    //
    // Send the packet along

// disable MSVC warning for narrowing conversion.
#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4267)
#endif
    const int32_t ret = sendto(m_serverSocket, (char*)stream.data(), stream.tell(), 0,
                               (struct sockaddr *) &m_sensorAddress,
                               sizeof(m_sensorAddress));
#ifdef WIN32
#pragma warning (pop)
#endif

    if (static_cast<size_t>(ret) != stream.tell())
        CRL_EXCEPTION("error sending data to sensor, %d/%d bytes written: %s",
                      ret, stream.tell(), strerror(errno));
}

//
// Convert data source types from wire<->API. These match 1:1 right now, but we
// want the freedom to change the wire protocol as we see fit.

wire::SourceType impl::sourceApiToWire(DataSource mask)
{
    wire::SourceType wire_mask = 0;

    if (mask & Source_Raw_Left)               wire_mask |= wire::SOURCE_RAW_LEFT;
    if (mask & Source_Raw_Right)              wire_mask |= wire::SOURCE_RAW_RIGHT;
    if (mask & Source_Raw_Aux)                wire_mask |= wire::SOURCE_RAW_AUX;
    if (mask & Source_Luma_Left)              wire_mask |= wire::SOURCE_LUMA_LEFT;
    if (mask & Source_Luma_Right)             wire_mask |= wire::SOURCE_LUMA_RIGHT;
    if (mask & Source_Luma_Aux)               wire_mask |= wire::SOURCE_LUMA_AUX;
    if (mask & Source_Luma_Rectified_Left)    wire_mask |= wire::SOURCE_LUMA_RECT_LEFT;
    if (mask & Source_Luma_Rectified_Right)   wire_mask |= wire::SOURCE_LUMA_RECT_RIGHT;
    if (mask & Source_Luma_Rectified_Aux)     wire_mask |= wire::SOURCE_LUMA_RECT_AUX;
    if (mask & Source_Chroma_Left)            wire_mask |= wire::SOURCE_CHROMA_LEFT;
    if (mask & Source_Chroma_Right)           wire_mask |= wire::SOURCE_CHROMA_RIGHT;
    if (mask & Source_Chroma_Rectified_Aux)   wire_mask |= wire::SOURCE_CHROMA_RECT_AUX;
    if (mask & Source_Chroma_Aux)             wire_mask |= wire::SOURCE_CHROMA_AUX;
    if (mask & Source_Disparity)              wire_mask |= wire::SOURCE_DISPARITY;
    if (mask & Source_Disparity_Right)        wire_mask |= wire::SOURCE_DISPARITY_RIGHT;
    if (mask & Source_Disparity_Aux)          wire_mask |= wire::SOURCE_DISPARITY_AUX;
    if (mask & Source_Disparity_Cost)         wire_mask |= wire::SOURCE_DISPARITY_COST;
    if (mask & Source_Jpeg_Left)              wire_mask |= wire::SOURCE_JPEG_LEFT;
    if (mask & Source_Rgb_Left)               wire_mask |= wire::SOURCE_RGB_LEFT;
    if (mask & Source_Lidar_Scan)             wire_mask |= wire::SOURCE_LIDAR_SCAN;
    if (mask & Source_Imu)                    wire_mask |= wire::SOURCE_IMU;
    if (mask & Source_Pps)                    wire_mask |= wire::SOURCE_PPS;
    if (mask & Source_Compressed_Left)        wire_mask |= wire::SOURCE_COMPRESSED_LEFT;
    if (mask & Source_Compressed_Rectified_Left)        wire_mask |= wire::SOURCE_COMPRESSED_RECTIFIED_LEFT;
    if (mask & Source_Compressed_Right)                 wire_mask |= wire::SOURCE_COMPRESSED_RIGHT;
    if (mask & Source_Compressed_Rectified_Right)       wire_mask |= wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT;
    if (mask & Source_Compressed_Aux)                   wire_mask |= wire::SOURCE_COMPRESSED_AUX;
    if (mask & Source_Compressed_Rectified_Aux)         wire_mask |= wire::SOURCE_COMPRESSED_RECTIFIED_AUX;
    if (mask & Source_Ground_Surface_Spline_Data)       wire_mask |= wire::SOURCE_GROUND_SURFACE_SPLINE_DATA;
    if (mask & Source_Ground_Surface_Class_Image)       wire_mask |= wire::SOURCE_GROUND_SURFACE_CLASS_IMAGE;
    if (mask & Source_AprilTag_Detections)              wire_mask |= wire::SOURCE_APRILTAG_DETECTIONS;

    return wire_mask;
}

DataSource impl::sourceWireToApi(wire::SourceType mask)
{
    DataSource api_mask = 0;

    if (mask & wire::SOURCE_RAW_LEFT)          api_mask |= Source_Raw_Left;
    if (mask & wire::SOURCE_RAW_RIGHT)         api_mask |= Source_Raw_Right;
    if (mask & wire::SOURCE_RAW_AUX)           api_mask |= Source_Raw_Aux;
    if (mask & wire::SOURCE_LUMA_LEFT)         api_mask |= Source_Luma_Left;
    if (mask & wire::SOURCE_LUMA_RIGHT)        api_mask |= Source_Luma_Right;
    if (mask & wire::SOURCE_LUMA_AUX)          api_mask |= Source_Luma_Aux;
    if (mask & wire::SOURCE_LUMA_RECT_LEFT)    api_mask |= Source_Luma_Rectified_Left;
    if (mask & wire::SOURCE_LUMA_RECT_RIGHT)   api_mask |= Source_Luma_Rectified_Right;
    if (mask & wire::SOURCE_LUMA_RECT_AUX)     api_mask |= Source_Luma_Rectified_Aux;
    if (mask & wire::SOURCE_CHROMA_LEFT)       api_mask |= Source_Chroma_Left;
    if (mask & wire::SOURCE_CHROMA_RIGHT)      api_mask |= Source_Chroma_Right;
    if (mask & wire::SOURCE_CHROMA_AUX)        api_mask |= Source_Chroma_Aux;
    if (mask & wire::SOURCE_CHROMA_RECT_AUX)   api_mask |= Source_Chroma_Rectified_Aux;
    if (mask & wire::SOURCE_DISPARITY)         api_mask |= Source_Disparity;
    if (mask & wire::SOURCE_DISPARITY_RIGHT)   api_mask |= Source_Disparity_Right;
    if (mask & wire::SOURCE_DISPARITY_AUX)     api_mask |= Source_Disparity_Aux;
    if (mask & wire::SOURCE_DISPARITY_COST)    api_mask |= Source_Disparity_Cost;
    if (mask & wire::SOURCE_JPEG_LEFT)         api_mask |= Source_Jpeg_Left;
    if (mask & wire::SOURCE_RGB_LEFT)          api_mask |= Source_Rgb_Left;
    if (mask & wire::SOURCE_LIDAR_SCAN)        api_mask |= Source_Lidar_Scan;
    if (mask & wire::SOURCE_IMU)               api_mask |= Source_Imu;
    if (mask & wire::SOURCE_PPS)               api_mask |= Source_Pps;
    if (mask & wire::SOURCE_GROUND_SURFACE_SPLINE_DATA)     api_mask |= Source_Ground_Surface_Spline_Data;
    if (mask & wire::SOURCE_GROUND_SURFACE_CLASS_IMAGE)     api_mask |= Source_Ground_Surface_Class_Image;
    if (mask & wire::SOURCE_APRILTAG_DETECTIONS)            api_mask |= Source_AprilTag_Detections;
    if (mask & wire::SOURCE_COMPRESSED_LEFT)                api_mask |= Source_Compressed_Left;
    if (mask & wire::SOURCE_COMPRESSED_RECTIFIED_LEFT)      api_mask |= Source_Compressed_Rectified_Left;
    if (mask & wire::SOURCE_COMPRESSED_RIGHT)               api_mask |= Source_Compressed_Right;
    if (mask & wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT)     api_mask |= Source_Compressed_Rectified_Right;
    if (mask & wire::SOURCE_COMPRESSED_AUX)                 api_mask |= Source_Compressed_Aux;
    if (mask & wire::SOURCE_COMPRESSED_RECTIFIED_AUX)       api_mask |= Source_Compressed_Rectified_Aux;

    return api_mask;
}

uint32_t impl::hardwareApiToWire(uint32_t a)
{
    switch(a) {
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_SL:                  return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_SL;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7:                  return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_M:                   return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_M;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7S:                 return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7S;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S21:                 return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21:                return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27:            return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30:                 return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S30;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7AR:                return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7AR;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21:                return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM:             return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB:     return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO:  return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM: return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM;
    case system::DeviceInfo::HARDWARE_REV_BCAM:                           return wire::SysDeviceInfo::HARDWARE_REV_BCAM;
    case system::DeviceInfo::HARDWARE_REV_MONO:                           return wire::SysDeviceInfo::HARDWARE_REV_MONO;
    default:
        CRL_DEBUG("unknown API hardware type \"%d\"\n", a);
        return a; // pass through
    }
}
uint32_t impl::hardwareWireToApi(uint32_t w)
{
    switch(w) {
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_SL:                  return system::DeviceInfo::HARDWARE_REV_MULTISENSE_SL;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7:                  return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_M:                   return system::DeviceInfo::HARDWARE_REV_MULTISENSE_M;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7S:                 return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7S;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21:                 return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S21;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21:                return system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27:            return system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S30:                 return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7AR:                return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7AR;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21:                return system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM:             return system::DeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB:     return system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO:  return system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM: return system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM;
    case wire::SysDeviceInfo::HARDWARE_REV_BCAM:                           return system::DeviceInfo::HARDWARE_REV_BCAM;
    case wire::SysDeviceInfo::HARDWARE_REV_MONO:                           return system::DeviceInfo::HARDWARE_REV_MONO;
    default:
        CRL_DEBUG("unknown WIRE hardware type \"%d\"\n", w);
        return w; // pass through
    }
}
uint32_t impl::imagerApiToWire(uint32_t a)
{
    switch(a) {
    case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:  return wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_GREY;
    case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR: return wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_COLOR;
    case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:  return wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_GREY;
    case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR: return wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_COLOR;
    case system::DeviceInfo::IMAGER_TYPE_IMX104_COLOR:  return wire::SysDeviceInfo::IMAGER_TYPE_IMX104_COLOR;
    case system::DeviceInfo::IMAGER_TYPE_AR0234_GREY:   return wire::SysDeviceInfo::IMAGER_TYPE_AR0234_GREY;
    case system::DeviceInfo::IMAGER_TYPE_AR0239_COLOR:  return wire::SysDeviceInfo::IMAGER_TYPE_AR0239_COLOR;
    case system::DeviceInfo::IMAGER_TYPE_FLIR_TAU2:  return wire::SysDeviceInfo::IMAGER_TYPE_FLIR_TAU2;
    default:
        CRL_DEBUG("unknown API imager type \"%d\"\n", a);
        return a; // pass through
    }
}
uint32_t impl::imagerWireToApi(uint32_t w)
{
    switch(w) {
    case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_GREY:  return system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY;
    case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_COLOR: return system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR;
    case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_GREY:  return system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY;
    case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_COLOR: return system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR;
    case wire::SysDeviceInfo::IMAGER_TYPE_IMX104_COLOR:  return system::DeviceInfo::IMAGER_TYPE_IMX104_COLOR;
    case wire::SysDeviceInfo::IMAGER_TYPE_AR0234_GREY:   return system::DeviceInfo::IMAGER_TYPE_AR0234_GREY;
    case wire::SysDeviceInfo::IMAGER_TYPE_AR0239_COLOR:  return system::DeviceInfo::IMAGER_TYPE_AR0239_COLOR;
    case wire::SysDeviceInfo::IMAGER_TYPE_FLIR_TAU2:  return system::DeviceInfo::IMAGER_TYPE_FLIR_TAU2;
    default:
        CRL_DEBUG("unknown WIRE imager type \"%d\"\n", w);
        return w; // pass through
    }
}

//
// Apply a time offset correction

void impl::applySensorTimeOffset(const utility::TimeStamp& offset)
{
    utility::ScopedLock lock(m_timeLock);

    //
    // Reseed on startup or if there is a large jump in time
    //
    CRL_CONSTEXPR int TIME_SYNC_THRESH_SECONDS = 100;
    const bool seed_offset = (false == m_timeOffsetInit) ||
                             (abs(m_timeOffset.getSeconds() - offset.getSeconds()) > TIME_SYNC_THRESH_SECONDS);

    if (seed_offset)
    {
        m_timeOffset = offset; // seed
        m_timeOffsetInit = true;
        return;
    }

    //
    // Use doubles to compute offsets to prevent overflow

    const double samples = static_cast<double>(TIME_SYNC_OFFSET_DECAY);

    const double currentOffset = m_timeOffset.getSeconds() + m_timeOffset.getMicroSeconds() * 1e-6;
    const double measuredOffset = offset.getSeconds() + offset.getMicroSeconds() * 1e-6;

    const double newOffset = utility::decayedAverage(currentOffset, samples, measuredOffset);

    const int32_t newOffsetSeconds = static_cast<int32_t>(newOffset);
    const int32_t newOffsetMicroSeconds = static_cast<int32_t>((newOffset - newOffsetSeconds) * 1e6);

    m_timeOffset = utility::TimeStamp(newOffsetSeconds, newOffsetMicroSeconds);
}

//
// Return the corrected time

utility::TimeStamp impl::sensorToLocalTime(const utility::TimeStamp& sensorTime)
{
    utility::ScopedLock lock(m_timeLock);
    return m_timeOffset + sensorTime;
}

//
// Correct the time, populate seconds/microseconds

void impl::sensorToLocalTime(const utility::TimeStamp& sensorTime,
                             uint32_t&     seconds,
                             uint32_t&     microseconds)
{
    const utility::TimeStamp corrected = sensorToLocalTime(sensorTime);
    seconds          = corrected.getSeconds();
    microseconds     = corrected.getMicroSeconds();
}

//
// An internal thread for status/time-synchronization

#ifdef WIN32
DWORD impl::statusThread(void *userDataP)
#else
void *impl::statusThread(void *userDataP)
#endif
{
    impl *selfP = reinterpret_cast<impl*>(userDataP);

    //
    // Loop until shutdown

    while(selfP->m_threadsRunning) {

        try {

            //
            // Setup handler for the status response

            ScopedWatch ack(wire::StatusResponse::ID, selfP->m_watch);

            //
            // Send the status request, recording the (approx) local time

            const utility::TimeStamp ping = utility::TimeStamp::getCurrentTime();
            selfP->publish(wire::StatusRequest());

            //
            // Wait for the response

            Status status;
            if (ack.wait(status, 0.010)) {

                //
                // Record (approx) time of response

                const utility::TimeStamp pong = utility::TimeStamp::getCurrentTime();

                //
                // Extract the response payload

                wire::StatusResponse msg;
                selfP->m_messages.extract(msg);


                //
                // Estimate 'msg.uptime' capture using half of the round trip period

                const utility::TimeStamp latency((pong.getNanoSeconds() - ping.getNanoSeconds()) / 2);

                //
                // Compute and apply the estimated time offset

                const utility::TimeStamp offset = ping + latency - msg.uptime;

                selfP->applySensorTimeOffset(offset);

                //
                // Cache the status message

                selfP->m_statusResponseMessage = msg;
                selfP->m_getStatusReturnStatus = Status_Ok;
            } else {
                selfP->m_getStatusReturnStatus = Status_TimedOut;
            }

        } catch (const std::exception& e) {

            CRL_DEBUG("exception: %s\n", e.what());

        } catch (...) {

            CRL_DEBUG_RAW("unknown exception\n");
        }

        //
        // Recompute offset at ~1Hz

        usleep(static_cast<unsigned int> (1e6));
    }

    return NULL;
}

} // namespace details

Channel* Channel::Create(const std::string& address)
{
    try {

        return new details::impl(address, Remote_Head_VPB, "");

    } catch (const std::exception& e) {

        CRL_DEBUG("exception: %s\n", e.what());
        return NULL;
    }
}

Channel* Channel::Create(const std::string& address, const RemoteHeadChannel &cameraId)
{
    try {

        return new details::impl(address, cameraId, "");

    } catch (const std::exception& e) {

        CRL_DEBUG("exception: %s\n", e.what());
        return NULL;
    }
}

Channel* Channel::Create(const std::string &address, const std::string& ifName)
    {
        try {
            return new details::impl(address, Remote_Head_VPB, ifName);
        } catch (const std::exception& e) {
            CRL_DEBUG("exception: %s\n", e.what());
            return NULL;
        }
    }

Channel* Channel::Create(const std::string &address, const RemoteHeadChannel &cameraId, const std::string &ifName)
    {
        try {
            return new details::impl(address, cameraId, ifName);
        } catch (const std::exception& e) {
            CRL_DEBUG("exception: %s\n", e.what());
            return NULL;
        }
    }

void Channel::Destroy(Channel *instanceP)
{
    try {

        if (instanceP)
            delete static_cast<details::impl*>(instanceP);

    } catch (const std::exception& e) {

        CRL_DEBUG("exception: %s\n", e.what());
    }
}

const char *Channel::statusString(Status status)
{
    switch(status) {
    case Status_Ok:          return "Ok";
    case Status_TimedOut:    return "Timed out";
    case Status_Error:       return "Error";
    case Status_Failed:      return "Failed";
    case Status_Unsupported: return "Unsupported";
    case Status_Unknown:     return "Unknown command";
    case Status_Exception:   return "Exception";
    }

    return "Unknown Error";
}

} // namespace multisense
} // namespace crl
