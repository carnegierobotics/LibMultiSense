/**
 * @file LibMultiSense/details/channel.cc
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
 *   2013-04-25, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include "details/channel.hh"
#include "details/query.hh"

#include "details/wire/DisparityMessage.h"
#include "details/wire/SysMtuMessage.h"
#include "details/wire/SysGetMtuMessage.h"
#include "details/wire/StatusRequestMessage.h"
#include "details/wire/StatusResponseMessage.h"
#include "details/wire/VersionRequestMessage.h"
#include "details/wire/SysDeviceInfoMessage.h"

#include "details/utility/Functional.hh"

#ifndef WIN32
#include <netdb.h>
#endif
#include <errno.h>
#include <fcntl.h>

namespace crl {
namespace multisense {
namespace details {

//
// Implementation constructor

impl::impl(const std::string& address) :
    m_serverSocket(-1),
    m_serverSocketPort(0),
    m_sensorAddress(),
    m_sensorMtu(MAX_MTU_SIZE),
    m_incomingBuffer(MAX_MTU_SIZE),
    m_txSeqId(0),
    m_lastRxSeqId(-1),
    m_unWrappedRxSeqId(0),
    m_udpTrackerCache(UDP_TRACKER_CACHE_DEPTH, 0),
    m_rxLargeBufferPool(),
    m_rxSmallBufferPool(),
    m_imageMetaCache(IMAGE_META_CACHE_DEPTH, 0),
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
    m_watch(),
    m_messages(),
    m_streamsEnabled(0),
    m_timeLock(),
    m_timeOffsetInit(false),
    m_timeOffset(0),
    m_networkTimeSyncEnabled(true),
    m_sensorVersion()
{
#if WIN32
    WSADATA wsaData;
    int result = WSAStartup (MAKEWORD (0x02, 0x02), &wsaData);
    if (result != 0)
        CRL_EXCEPTION("WSAStartup() failed: %d", result);
#endif

    //
    // Make sure the sensor address is sane

    struct hostent *hostP = gethostbyname(address.c_str());
    if (NULL == hostP)
        CRL_EXCEPTION("unable to resolve \"%s\": %s",
                      address.c_str(), strerror(errno));

    //
    // Set up the address for transmission

    in_addr addr;

    memcpy(&(addr.s_addr), hostP->h_addr, hostP->h_length);
    memset(&m_sensorAddress, 0, sizeof(m_sensorAddress));

    m_sensorAddress.sin_family = AF_INET;
    m_sensorAddress.sin_port   = htons(DEFAULT_SENSOR_TX_PORT);
    m_sensorAddress.sin_addr   = addr;

    //
    // Create a pool of RX buffers

    for(uint32_t i=0; i<RX_POOL_LARGE_BUFFER_COUNT; i++)
        m_rxLargeBufferPool.push_back(new utility::BufferStreamWriter(RX_POOL_LARGE_BUFFER_SIZE));
    for(uint32_t i=0; i<RX_POOL_SMALL_BUFFER_COUNT; i++)
        m_rxSmallBufferPool.push_back(new utility::BufferStreamWriter(RX_POOL_SMALL_BUFFER_SIZE));

    //
    // Bind to the port

    bind();

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
        CRL_EXCEPTION("failed to establish comms with the sensor at \"%s\"",
                      address.c_str());
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

    BufferPool::const_iterator it;
    for(it  = m_rxLargeBufferPool.begin();
        it != m_rxLargeBufferPool.end();
        ++it)
        delete *it;
    for(it  = m_rxSmallBufferPool.begin();
        it != m_rxSmallBufferPool.end();
        ++it)
        delete *it;

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

void impl::bind()
{
    //
    // Create the socket.

    m_serverSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_serverSocket < 0)
        CRL_EXCEPTION("failed to create the UDP socket: %s",
                      strerror(errno));

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

    int bufferSize = 48 * 1024 * 1024;

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
    header.messageLength      = stream.tell() - sizeof(wire::Header);
    header.byteOffset         = 0;

    //
    // Send the packet along

    const int32_t ret = sendto(m_serverSocket, (char*)stream.data(), stream.tell(), 0,
                               (struct sockaddr *) &m_sensorAddress,
                               sizeof(m_sensorAddress));        
    
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
    if (mask & Source_Luma_Left)              wire_mask |= wire::SOURCE_LUMA_LEFT;
    if (mask & Source_Luma_Right)             wire_mask |= wire::SOURCE_LUMA_RIGHT;
    if (mask & Source_Luma_Rectified_Left)    wire_mask |= wire::SOURCE_LUMA_RECT_LEFT;
    if (mask & Source_Luma_Rectified_Right)   wire_mask |= wire::SOURCE_LUMA_RECT_RIGHT;
    if (mask & Source_Chroma_Left)            wire_mask |= wire::SOURCE_CHROMA_LEFT;
    if (mask & Source_Chroma_Right)           wire_mask |= wire::SOURCE_CHROMA_RIGHT;
    if (mask & Source_Disparity)              wire_mask |= wire::SOURCE_DISPARITY;
    if (mask & Source_Disparity_Right)        wire_mask |= wire::SOURCE_DISPARITY_RIGHT;
    if (mask & Source_Disparity_Cost)         wire_mask |= wire::SOURCE_DISPARITY_COST;
    if (mask & Source_Jpeg_Left)              wire_mask |= wire::SOURCE_JPEG_LEFT;
    if (mask & Source_Rgb_Left)               wire_mask |= wire::SOURCE_RGB_LEFT;
    if (mask & Source_Lidar_Scan)             wire_mask |= wire::SOURCE_LIDAR_SCAN;
    if (mask & Source_Imu)                    wire_mask |= wire::SOURCE_IMU;
    if (mask & Source_Pps)                    wire_mask |= wire::SOURCE_PPS;

    return wire_mask;
};

DataSource impl::sourceWireToApi(wire::SourceType mask)
{
    DataSource api_mask = 0;

    if (mask & wire::SOURCE_RAW_LEFT)          api_mask |= Source_Raw_Left;
    if (mask & wire::SOURCE_RAW_RIGHT)         api_mask |= Source_Raw_Right;
    if (mask & wire::SOURCE_LUMA_LEFT)         api_mask |= Source_Luma_Left;
    if (mask & wire::SOURCE_LUMA_RIGHT)        api_mask |= Source_Luma_Right;
    if (mask & wire::SOURCE_LUMA_RECT_LEFT)    api_mask |= Source_Luma_Rectified_Left;
    if (mask & wire::SOURCE_LUMA_RECT_RIGHT)   api_mask |= Source_Luma_Rectified_Right;
    if (mask & wire::SOURCE_CHROMA_LEFT)       api_mask |= Source_Chroma_Left;
    if (mask & wire::SOURCE_CHROMA_RIGHT)      api_mask |= Source_Chroma_Right;
    if (mask & wire::SOURCE_DISPARITY)         api_mask |= Source_Disparity;
    if (mask & wire::SOURCE_DISPARITY_RIGHT)   api_mask |= Source_Disparity_Right;
    if (mask & wire::SOURCE_DISPARITY_COST)    api_mask |= Source_Disparity_Cost;
    if (mask & wire::SOURCE_JPEG_LEFT)         api_mask |= Source_Jpeg_Left;
    if (mask & wire::SOURCE_RGB_LEFT)          api_mask |= Source_Rgb_Left;
    if (mask & wire::SOURCE_LIDAR_SCAN)        api_mask |= Source_Lidar_Scan;
    if (mask & wire::SOURCE_IMU)               api_mask |= Source_Imu;
    if (mask & wire::SOURCE_PPS)               api_mask |= Source_Pps;

    return api_mask;
};

uint32_t impl::hardwareApiToWire(uint32_t a) 
{
    switch(a) {
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_SL:    return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_SL;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7:    return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_M:     return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_M;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7S:   return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7S;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_S21:   return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21;
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21:   return wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21;
    case system::DeviceInfo::HARDWARE_REV_BCAM:             return wire::SysDeviceInfo::HARDWARE_REV_BCAM;
    default:
        CRL_DEBUG("unknown API hardware type \"%d\"\n", a);
        return a; // pass through
    }
}
uint32_t impl::hardwareWireToApi(uint32_t w) 
{
    switch(w) {
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_SL:    return system::DeviceInfo::HARDWARE_REV_MULTISENSE_SL;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7:    return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_M:     return system::DeviceInfo::HARDWARE_REV_MULTISENSE_M;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7S:   return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S7S;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21:   return system::DeviceInfo::HARDWARE_REV_MULTISENSE_S21;
    case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21:  return system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21;
    case wire::SysDeviceInfo::HARDWARE_REV_BCAM:             return system::DeviceInfo::HARDWARE_REV_BCAM;
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
    default:
        CRL_DEBUG("unknown WIRE imager type \"%d\"\n", w);
        return w; // pass through
    }
}

//
// Apply a time offset correction

void impl::applySensorTimeOffset(const double& offset)
{
    utility::ScopedLock lock(m_timeLock);

    if (false == m_timeOffsetInit) {
        m_timeOffset     = offset; // seed
        m_timeOffsetInit = true;
        return;
    }
    
    const double samples = static_cast<double>(TIME_SYNC_OFFSET_DECAY);

    m_timeOffset = utility::decayedAverage(m_timeOffset, samples, offset);
}

//
// Return the corrected time

double impl::sensorToLocalTime(const double& sensorTime)
{
    utility::ScopedLock lock(m_timeLock);
    return m_timeOffset + sensorTime;
}

//
// Correct the time, populate seconds/microseconds

void impl::sensorToLocalTime(const double& sensorTime,
                             uint32_t&     seconds,
                             uint32_t&     microseconds)
{
    double corrected = sensorToLocalTime(sensorTime);
    seconds          = static_cast<uint32_t>(corrected);
    microseconds     = static_cast<uint32_t>(1e6 * (corrected - static_cast<double>(seconds)));
}

//
// An internal thread for status/time-synchroniziation

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

            const double ping = utility::TimeStamp::getCurrentTime();
            selfP->publish(wire::StatusRequest());

            //
            // Wait for the response

            Status status;
            if (ack.wait(status, 0.010)) {

                //
                // Record (approx) time of response

                const double pong = utility::TimeStamp::getCurrentTime();

                //
                // Extract the response payload

                wire::StatusResponse msg;
                selfP->m_messages.extract(msg);


                //
                // Estimate 'msg.uptime' capture using half of the round trip period

                const double latency = (pong - ping) / 2.0;

                //
                // Compute and apply the estimated time offset

                const double offset = (ping + latency) - static_cast<double>(msg.uptime);
                selfP->applySensorTimeOffset(offset);

                //
                // Cache the status message

                selfP->m_statusResponseMessage = msg;
            }
        
        } catch (const std::exception& e) {

            CRL_DEBUG("exception: %s\n", e.what());

        } catch (...) {

            CRL_DEBUG("unknown exception\n");
        }

        //
        // Recompute offset at ~1Hz

        usleep(static_cast<unsigned int> (1e6));
    }

    return NULL;
}

}; // namespace details

Channel* Channel::Create(const std::string& address)
{
    try {

        return new details::impl(address);

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

}; // namespace multisense
}; // namespace crl
