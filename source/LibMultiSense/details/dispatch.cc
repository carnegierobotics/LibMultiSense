/**
 * @file LibMultiSense/details/dispatch.cc
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
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include "details/channel.hh"

#include "details/wire/AckMessage.h"

#include "details/wire/VersionResponseMessage.h"
#include "details/wire/StatusResponseMessage.h"

#include "details/wire/CamConfigMessage.h"
#include "details/wire/ImageMessage.h"
#include "details/wire/JpegMessage.h"
#include "details/wire/ImageMetaMessage.h"
#include "details/wire/DisparityMessage.h"

#include "details/wire/CamHistoryMessage.h"

#include "details/wire/LidarDataMessage.h"

#include "details/wire/LedStatusMessage.h"

#include "details/wire/LedSensorStatusMessage.h"

#include "details/wire/SysMtuMessage.h"
#include "details/wire/SysNetworkMessage.h"
#include "details/wire/SysFlashResponseMessage.h"
#include "details/wire/SysDeviceInfoMessage.h"
#include "details/wire/SysCameraCalibrationMessage.h"
#include "details/wire/SysSensorCalibrationMessage.h"
#include "details/wire/SysLidarCalibrationMessage.h"
#include "details/wire/SysDeviceModesMessage.h"
#include "details/wire/SysExternalCalibrationMessage.h"

#include "details/wire/SysPpsMessage.h"

#include "details/wire/ImuDataMessage.h"
#include "details/wire/ImuConfigMessage.h"
#include "details/wire/ImuInfoMessage.h"

#include "details/wire/SysTestMtuResponseMessage.h"
#include "details/wire/SysDirectedStreamsMessage.h"

#include <limits>

namespace crl {
namespace multisense {
namespace details {
namespace {

//
// Default UDP assembler

void defaultUdpAssembler(utility::BufferStreamWriter& stream,
                         const uint8_t               *dataP,
                         uint32_t                     offset,
                         uint32_t                     length)
{
    stream.seek(offset);
    stream.write(dataP, length);
}

}; // anonymous

//
// Publish an image 

void impl::dispatchImage(utility::BufferStream& buffer,
                         image::Header&         header)
{
    utility::ScopedLock lock(m_dispatchLock);

    std::list<ImageListener*>::const_iterator it;

    for(it  = m_imageListeners.begin();
        it != m_imageListeners.end();
        it ++)
        (*it)->dispatch(buffer, header);
}

//
// Publish a laser scan

void impl::dispatchLidar(utility::BufferStream& buffer,
                         lidar::Header&         header)
{
    utility::ScopedLock lock(m_dispatchLock);

    std::list<LidarListener*>::const_iterator it;

    for(it  = m_lidarListeners.begin();
        it != m_lidarListeners.end();
        it ++)
        (*it)->dispatch(buffer, header);
}
//
// Publish a PPS event

void impl::dispatchPps(pps::Header& header)
{
    utility::ScopedLock lock(m_dispatchLock);

    std::list<PpsListener*>::const_iterator it;

    for(it  = m_ppsListeners.begin();
        it != m_ppsListeners.end();
        it ++)
        (*it)->dispatch(header);
}

//
// Publish an IMU event

void impl::dispatchImu(imu::Header& header)
{
    utility::ScopedLock lock(m_dispatchLock);

    std::list<ImuListener*>::const_iterator it;

    for(it  = m_imuListeners.begin();
        it != m_imuListeners.end();
        it ++)
        (*it)->dispatch(header);
}

//
// Dispatch incoming messages

void impl::dispatch(utility::BufferStreamWriter& buffer)
{
    utility::BufferStreamReader stream(buffer);

    //
    // Extract the type and version fields, which are stored
    // first in the stream

    wire::IdType      id;
    wire::VersionType version;

    stream & id;
    stream & version;
    
    //
    // Handle the message. 
    //
    // Simple, low-rate messages are dynamically stored 
    // off for possible presentation to the user in a signal 
    // handler. 
    // 
    // Larger data types use a threaded dispatch 
    // mechanism with a reference-counted buffer back-end.

    switch(id) {
    case MSG_ID(wire::LidarData::ID):
    {
        wire::LidarData scan(stream, version);
        lidar::Header   header;

	const int32_t  scanArc  = static_cast<int32_t> (utility::degreesToRadians(270.0) * 1e6); // microradians
	const uint32_t maxRange = static_cast<uint32_t> (30.0 * 1e3); // mm

        if (false == m_networkTimeSyncEnabled) {

            header.timeStartSeconds      = scan.timeStartSeconds;
            header.timeStartMicroSeconds = scan.timeStartMicroSeconds;
            header.timeEndSeconds        = scan.timeEndSeconds;
            header.timeEndMicroSeconds   = scan.timeEndMicroSeconds;

        } else {

            sensorToLocalTime(static_cast<double>(scan.timeStartSeconds) + 
                              1e-6 * static_cast<double>(scan.timeStartMicroSeconds),
                              header.timeStartSeconds, header.timeStartMicroSeconds);

            sensorToLocalTime(static_cast<double>(scan.timeEndSeconds) + 
                              1e-6 * static_cast<double>(scan.timeEndMicroSeconds),
                              header.timeEndSeconds, header.timeEndMicroSeconds);
        }

        header.scanId            = scan.scanCount;
        header.spindleAngleStart = scan.angleStart;
        header.spindleAngleEnd   = scan.angleEnd;
        header.scanArc           = scanArc;
        header.maxRange          = maxRange;
        header.pointCount        = scan.points;
        header.rangesP           = scan.distanceP;
        header.intensitiesP      = scan.intensityP;

        dispatchLidar(buffer, header);

        break;
    }
    case MSG_ID(wire::ImageMeta::ID):
    {
        wire::ImageMeta *metaP = new (std::nothrow) wire::ImageMeta(stream, version);

        if (NULL == metaP)
            CRL_EXCEPTION("unable to allocate metadata");

        m_imageMetaCache.insert(metaP->frameId, metaP); // destroys oldest

        break;
    }
    case MSG_ID(wire::JpegImage::ID):
    {
        wire::JpegImage image(stream, version);

        const wire::ImageMeta *metaP = m_imageMetaCache.find(image.frameId);
        if (NULL == metaP)
            break;
            //CRL_EXCEPTION("no meta cached for frameId %d", image.frameId);

        image::Header header;

        if (false == m_networkTimeSyncEnabled) {

            header.timeSeconds      = metaP->timeSeconds;
            header.timeMicroSeconds = metaP->timeMicroSeconds;

        } else
            sensorToLocalTime(static_cast<double>(metaP->timeSeconds) + 
                              1e-6 * static_cast<double>(metaP->timeMicroSeconds),
                              header.timeSeconds, header.timeMicroSeconds);

        header.source           = sourceWireToApi(image.source);
        header.bitsPerPixel     = 0;
        header.width            = image.width;
        header.height           = image.height;
        header.frameId          = image.frameId;
        header.exposure         = metaP->exposureTime;
        header.gain             = metaP->gain;
        header.framesPerSecond  = metaP->framesPerSecond;
        header.imageDataP       = image.dataP;
        header.imageLength      = image.length;
        
        dispatchImage(buffer, header);

        break;
    }
    case MSG_ID(wire::Image::ID):
    {
        wire::Image image(stream, version);

        const wire::ImageMeta *metaP = m_imageMetaCache.find(image.frameId);
        if (NULL == metaP)
            break;
            //CRL_EXCEPTION("no meta cached for frameId %d", image.frameId);

        image::Header header;

        if (false == m_networkTimeSyncEnabled) {

            header.timeSeconds      = metaP->timeSeconds;
            header.timeMicroSeconds = metaP->timeMicroSeconds;

        } else
            sensorToLocalTime(static_cast<double>(metaP->timeSeconds) + 
                              1e-6 * static_cast<double>(metaP->timeMicroSeconds),
                              header.timeSeconds, header.timeMicroSeconds);

        header.source           = sourceWireToApi(image.source);
        header.bitsPerPixel     = image.bitsPerPixel;
        header.width            = image.width;
        header.height           = image.height;
        header.frameId          = image.frameId;
        header.exposure         = metaP->exposureTime;
        header.gain             = metaP->gain;
        header.framesPerSecond  = metaP->framesPerSecond;
        header.imageDataP       = image.dataP;
        header.imageLength      = static_cast<uint32_t>(std::ceil(((double) image.bitsPerPixel / 8.0) * image.width * image.height));

        dispatchImage(buffer, header);

        break;
    }
    case MSG_ID(wire::Disparity::ID):
    {
        wire::Disparity image(stream, version);

        const wire::ImageMeta *metaP = m_imageMetaCache.find(image.frameId);
        if (NULL == metaP)
            break;
            //CRL_EXCEPTION("no meta cached for frameId %d", image.frameId);
        
        image::Header header;

        if (false == m_networkTimeSyncEnabled) {

            header.timeSeconds      = metaP->timeSeconds;
            header.timeMicroSeconds = metaP->timeMicroSeconds;

        } else
            sensorToLocalTime(static_cast<double>(metaP->timeSeconds) + 
                              1e-6 * static_cast<double>(metaP->timeMicroSeconds),
                              header.timeSeconds, header.timeMicroSeconds);
        
        header.source           = Source_Disparity;
        header.bitsPerPixel     = wire::Disparity::API_BITS_PER_PIXEL;
        header.width            = image.width;
        header.height           = image.height;
        header.frameId          = image.frameId;
        header.exposure         = metaP->exposureTime;
        header.gain             = metaP->gain;
        header.framesPerSecond  = metaP->framesPerSecond;
        header.imageDataP       = image.dataP;
        header.imageLength      = static_cast<uint32_t>(std::ceil(((double) wire::Disparity::API_BITS_PER_PIXEL / 8.0) * image.width * image.height));

        dispatchImage(buffer, header);

        break;
    }
    case MSG_ID(wire::SysPps::ID):
    {
        wire::SysPps pps(stream, version);

        pps::Header header;

        header.sensorTime = pps.ppsNanoSeconds;

        sensorToLocalTime(static_cast<double>(pps.ppsNanoSeconds) / 1e9,
                          header.timeSeconds, header.timeMicroSeconds);

        dispatchPps(header);

        break;
    }
    case MSG_ID(wire::ImuData::ID):
    {
        wire::ImuData imu(stream, version);

        imu::Header header;

        header.sequence = imu.sequence;
        header.samples.resize(imu.samples.size());
        
        for(uint32_t i=0; i<imu.samples.size(); i++) {

            const wire::ImuSample& w = imu.samples[i];
            imu::Sample&           a = header.samples[i];

            if (false == m_networkTimeSyncEnabled) {

                const int64_t oneBillion = static_cast<int64_t>(1e9);

                a.timeSeconds      = static_cast<uint32_t>(w.timeNanoSeconds / oneBillion);
                a.timeMicroSeconds = static_cast<uint32_t>((w.timeNanoSeconds % oneBillion) / 
                                                           static_cast<int64_t>(1000));

            } else
                sensorToLocalTime(static_cast<double>(w.timeNanoSeconds) / 1e9,
                                  a.timeSeconds, a.timeMicroSeconds);

            switch(w.type) {
            case wire::ImuSample::TYPE_ACCEL: a.type = imu::Sample::Type_Accelerometer; break;
            case wire::ImuSample::TYPE_GYRO : a.type = imu::Sample::Type_Gyroscope;     break;
            case wire::ImuSample::TYPE_MAG  : a.type = imu::Sample::Type_Magnetometer;  break;
            default: CRL_EXCEPTION("unknown wire IMU type: %d", w.type);
            }

            a.x = w.x; a.y = w.y; a.z = w.z;
        }

        dispatchImu(header);

        break;
    }
    case MSG_ID(wire::Ack::ID):
        break; // handle below
    case MSG_ID(wire::CamConfig::ID):
        m_messages.store(wire::CamConfig(stream, version));
        break;
    case MSG_ID(wire::CamHistory::ID):
        m_messages.store(wire::CamHistory(stream, version));
        break;
    case MSG_ID(wire::LedStatus::ID):
        m_messages.store(wire::LedStatus(stream, version));
        break;
    case MSG_ID(wire::LedSensorStatus::ID):
        m_messages.store(wire::LedSensorStatus(stream, version));
        break;
    case MSG_ID(wire::SysFlashResponse::ID):
        m_messages.store(wire::SysFlashResponse(stream, version));
        break;
    case MSG_ID(wire::SysDeviceInfo::ID):
        m_messages.store(wire::SysDeviceInfo(stream, version));
        break;
    case MSG_ID(wire::SysCameraCalibration::ID):
        m_messages.store(wire::SysCameraCalibration(stream, version));
        break;
    case MSG_ID(wire::SysSensorCalibration::ID):
        m_messages.store(wire::SysSensorCalibration(stream, version));
        break;
    case MSG_ID(wire::SysLidarCalibration::ID):
        m_messages.store(wire::SysLidarCalibration(stream, version));
        break;
    case MSG_ID(wire::SysMtu::ID):
        m_messages.store(wire::SysMtu(stream, version));
        break;
    case MSG_ID(wire::SysNetwork::ID):
        m_messages.store(wire::SysNetwork(stream, version));
        break;
    case MSG_ID(wire::SysDeviceModes::ID):
        m_messages.store(wire::SysDeviceModes(stream, version));
        break;
    case MSG_ID(wire::VersionResponse::ID):
        m_messages.store(wire::VersionResponse(stream, version));
        break;
    case MSG_ID(wire::StatusResponse::ID):
        m_messages.store(wire::StatusResponse(stream, version));   
        break;
    case MSG_ID(wire::ImuConfig::ID):
        m_messages.store(wire::ImuConfig(stream, version));
        break;
    case MSG_ID(wire::ImuInfo::ID):
        m_messages.store(wire::ImuInfo(stream, version));
        break;
    case MSG_ID(wire::SysTestMtuResponse::ID):
        m_messages.store(wire::SysTestMtuResponse(stream, version));
        break;
    case MSG_ID(wire::SysDirectedStreams::ID):
        m_messages.store(wire::SysDirectedStreams(stream, version));
        break;
    case MSG_ID(wire::SysExternalCalibration::ID):
        m_messages.store(wire::SysExternalCalibration(stream, version));
        break;
    default:

        CRL_DEBUG("unknown message received: id=%d, version=%d\n",
                  id, version);
        return;
    }

    //
    // Signal any listeners (_after_ the message is stored/dispatched)
    //
    // A [n]ack is a special case where we signal
    // the returned status code of the ack'd command, 
    // otherwise we signal valid reception of this message.

    switch(id) {
    case MSG_ID(wire::Ack::ID):
        m_watch.signal(wire::Ack(stream, version));
	break;
    default:	
	m_watch.signal(id);
	break;
    }
}

//
// Get a UDP assembler for this message type. We are given
// the first UDP packet in the stream

impl::UdpAssembler impl::getUdpAssembler(const uint8_t *firstDatagramP,
                                         uint32_t       length)
{
    //
    // Get the message type, it is stored after wire::Header

    utility::BufferStreamReader stream(firstDatagramP, length);
    stream.seek(sizeof(wire::Header));

    wire::IdType messageType; 
    stream & messageType;

    //
    // See if a custom handler has been registered

    UdpAssemblerMap::const_iterator it = m_udpAssemblerMap.find(messageType);

    if (m_udpAssemblerMap.end() != it)
        return it->second;
    else
        return defaultUdpAssembler;
}

//
// Find a suitably sized buffer for the incoming message

utility::BufferStreamWriter& impl::findFreeBuffer(uint32_t messageLength)
{    
    BufferPool *bP = NULL;
    
    if (messageLength <= RX_POOL_SMALL_BUFFER_SIZE)
        bP = &(m_rxSmallBufferPool);
    else if (messageLength <= RX_POOL_LARGE_BUFFER_SIZE)
        bP = &(m_rxLargeBufferPool);
    else
        CRL_EXCEPTION("message too large: %d bytes", messageLength);

    //
    // TODO: re-think the shared() mechanism of the buffers, so we do not
    //       have to walk this vector.

    BufferPool::const_iterator it = bP->begin();
    for(; it != bP->end(); it++)
        if (false == (*it)->shared())
            return *(*it);

    CRL_EXCEPTION("no free RX buffers (%d in use by consumers)\n", bP->size());
}

//
// Unwrap a 16-bit wire sequence ID into a unique 64-bit local ID

const int64_t& impl::unwrapSequenceId(uint16_t wireId)
{
    //
    // Look for a sequence change

    if (wireId != m_lastRxSeqId) {

        const uint16_t ID_MAX    = std::numeric_limits<uint16_t>::max();
        const uint16_t ID_CENTER = ID_MAX / 2;

        //
        // Seed

        if (-1 == m_lastRxSeqId)
			m_unWrappedRxSeqId = m_lastRxSeqId = wireId;

        //
        // Detect forward 16-bit wrap

        else if (wireId        < ID_CENTER   &&
                 m_lastRxSeqId > ID_CENTER) {

            m_unWrappedRxSeqId += 1 + (ID_MAX - m_lastRxSeqId) + wireId;

        //
        // Normal case

        } else
            m_unWrappedRxSeqId += wireId - m_lastRxSeqId;

        //
        // Remember change

        m_lastRxSeqId = wireId;
    }

    return m_unWrappedRxSeqId;
}

//
// Handles any incoming packets

void impl::handle()
{
    utility::ScopedLock lock(m_rxLock);

    for(;;) {
 
        //
        // Receive the packet
        
        const int bytesRead = recvfrom(m_serverSocket,
                                           (char*)m_incomingBuffer.data(),
                                           m_incomingBuffer.size(),
                                           0, NULL, NULL);
        //
        // Nothing left to read
        
        if (bytesRead < 0)
            break;

        //
        // Check for undersized packets

        else if (bytesRead < (int)sizeof(wire::Header))
            CRL_EXCEPTION("undersized packet: %d/%d bytes\n",
                          bytesRead, sizeof(wire::Header));

        //
        // For convenience below

        const uint8_t *inP = reinterpret_cast<const uint8_t*>(m_incomingBuffer.data());

        //
        // Validate the header

        const wire::Header& header = *(reinterpret_cast<const wire::Header*>(inP));

        if (wire::HEADER_MAGIC != header.magic)
            CRL_EXCEPTION("bad protocol magic: 0x%x, expecting 0x%x",
                          header.magic, wire::HEADER_MAGIC);
        else if (wire::HEADER_VERSION != header.version)
            CRL_EXCEPTION("bad protocol version: 0x%x, expecting 0x%x",
                          header.version, wire::HEADER_VERSION);
        else if (wire::HEADER_GROUP != header.group)
            CRL_EXCEPTION("bad protocol group: 0x%x, expecting 0x%x",
                          header.group, wire::HEADER_GROUP);

        //
        // Unwrap the sequence identifier

        const int64_t& sequence = unwrapSequenceId(header.sequenceIdentifier);

        //
        // See if we are already tracking this messge ID

        UdpTracker *trP = m_udpTrackerCache.find(sequence);
        if (NULL == trP) {

            //
            // If we drop first packet, we will drop entire message. Currently we 
            // require the first datagram in order to assign an assembler.
            // TODO: re-think this.

            if (0 != header.byteOffset)
                continue;
            else {

                //
                // Create a new tracker for this sequence id.

                trP = new UdpTracker(header.messageLength,
                                     getUdpAssembler(inP, bytesRead),
                                     findFreeBuffer(header.messageLength));
            }
        }
     
        //
        // Assemble the datagram into the message stream, returns true if the
        // assembly is complete.

        if (true == trP->assemble(bytesRead - sizeof(wire::Header),
                                  header.byteOffset,
                                  &(inP[sizeof(wire::Header)]))) {

            //
            // Dispatch to any listeners

            dispatch(trP->stream());

            //
            // Release the tracker

            if (1 == trP->packets())
                delete trP; // has not yet been cached
            else
                m_udpTrackerCache.remove(sequence);

        } else if (1 == trP->packets()) {

            //
            // Cache the tracker, as more UDP packets are
            // forthcoming for this message.

            m_udpTrackerCache.insert(sequence, trP);
        }
    }
}

//
// This thread waits for UDP packets

#if WIN32
DWORD impl::rxThread(void *userDataP)
#else
void *impl::rxThread(void *userDataP)
#endif
{
    impl     *selfP  = reinterpret_cast<impl*>(userDataP);
    const int server = selfP->m_serverSocket;
    fd_set    readSet;

    //
    // Loop until shutdown

    while(selfP->m_threadsRunning) {

        //
        // Add the server socket to the read set.

        FD_ZERO(&readSet);
        FD_SET(server, &readSet);

        //
        // Wait for a new packet to arrive, timing out every once in awhile

        struct timeval tv = {0, 200000}; // 5Hz
        const int result  = select(server+1, &readSet, NULL, NULL, &tv);
        if (result <= 0)
            continue;

        //
        // Let the comm object handle decoding

        try {

            selfP->handle();

        } catch (const std::exception& e) {
                    
            CRL_DEBUG("exception while decoding packet: %s\n", e.what());

        } catch ( ... ) {
            
            CRL_DEBUG("unknown exception while decoding packet\n");
        }
    }

    return NULL;
}

}}}; // namespaces
