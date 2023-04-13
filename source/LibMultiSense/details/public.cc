/**
 * @file LibMultiSense/details/public.cc
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
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include <stdlib.h>
#include <algorithm>

#include "MultiSense/details/utility/Functional.hh"

#include "MultiSense/details/channel.hh"
#include "MultiSense/details/query.hh"

#include "MultiSense/details/wire/VersionRequestMessage.hh"
#include "MultiSense/details/wire/VersionResponseMessage.hh"
#include "MultiSense/details/wire/StatusRequestMessage.hh"
#include "MultiSense/details/wire/StatusResponseMessage.hh"

#include "MultiSense/details/wire/StreamControlMessage.hh"
#include "MultiSense/details/wire/SysSetPtpMessage.hh"

#include "MultiSense/details/wire/CamControlMessage.hh"
#include "MultiSense/details/wire/AuxCamControlMessage.hh"
#include "MultiSense/details/wire/CamSetResolutionMessage.hh"
#include "MultiSense/details/wire/CamGetConfigMessage.hh"
#include "MultiSense/details/wire/AuxCamGetConfigMessage.hh"
#include "MultiSense/details/wire/CamConfigMessage.hh"
#include "MultiSense/details/wire/AuxCamConfigMessage.hh"
#include "MultiSense/details/wire/CamSetTriggerSourceMessage.hh"

#include "MultiSense/details/wire/RemoteHeadControlMessage.hh"
#include "MultiSense/details/wire/RemoteHeadGetConfigMessage.hh"
#include "MultiSense/details/wire/RemoteHeadConfigMessage.hh"

#include "MultiSense/details/wire/LidarSetMotorMessage.hh"

#include "MultiSense/details/wire/LedGetStatusMessage.hh"
#include "MultiSense/details/wire/LedSetMessage.hh"
#include "MultiSense/details/wire/LedStatusMessage.hh"

#include "MultiSense/details/wire/LedGetSensorStatusMessage.hh"
#include "MultiSense/details/wire/LedSensorStatusMessage.hh"

#include "MultiSense/details/wire/LidarPollMotorMessage.hh"
#include "MultiSense/details/wire/PollMotorInfoMessage.hh"

#include "MultiSense/details/wire/SysMtuMessage.hh"
#include "MultiSense/details/wire/SysGetMtuMessage.hh"
#include "MultiSense/details/wire/SysFlashOpMessage.hh"
#include "MultiSense/details/wire/SysGetNetworkMessage.hh"
#include "MultiSense/details/wire/SysNetworkMessage.hh"
#include "MultiSense/details/wire/SysGetDeviceInfoMessage.hh"
#include "MultiSense/details/wire/SysDeviceInfoMessage.hh"
#include "MultiSense/details/wire/SysGetCameraCalibrationMessage.hh"
#include "MultiSense/details/wire/SysCameraCalibrationMessage.hh"
#include "MultiSense/details/wire/SysGetSensorCalibrationMessage.hh"
#include "MultiSense/details/wire/SysGetTransmitDelayMessage.hh"
#include "MultiSense/details/wire/SysSensorCalibrationMessage.hh"
#include "MultiSense/details/wire/SysTransmitDelayMessage.hh"
#include "MultiSense/details/wire/SysGetLidarCalibrationMessage.hh"
#include "MultiSense/details/wire/SysLidarCalibrationMessage.hh"
#include "MultiSense/details/wire/SysGetDeviceModesMessage.hh"
#include "MultiSense/details/wire/SysDeviceModesMessage.hh"
#include "MultiSense/details/wire/SysGetExternalCalibrationMessage.hh"
#include "MultiSense/details/wire/SysExternalCalibrationMessage.hh"
#include "MultiSense/details/wire/SysGroundSurfaceParamsMessage.hh"
#include "MultiSense/details/wire/SysApriltagParamsMessage.hh"

#include "MultiSense/details/wire/ImuGetInfoMessage.hh"
#include "MultiSense/details/wire/ImuGetConfigMessage.hh"
#include "MultiSense/details/wire/ImuInfoMessage.hh"
#include "MultiSense/details/wire/ImuConfigMessage.hh"

#include "MultiSense/details/wire/SysTestMtuMessage.hh"
#include "MultiSense/details/wire/SysTestMtuResponseMessage.hh"

namespace crl {
namespace multisense {
namespace details {

//
// The user may "hold on" to the buffer back-end
// of a datum within a callback thread.

CRL_THREAD_LOCAL utility::BufferStream *dispatchBufferReferenceTP = NULL;

//
//
// Public API follows
//
//

//
// Adds a new image listener

Status impl::addIsolatedCallback(image::Callback callback,
                                 DataSource     imageSourceMask,
                                 void           *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_imageListeners.push_back(new ImageListener(callback,
                                                     imageSourceMask,
                                                     userDataP,
                                                     MAX_USER_IMAGE_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new laser listener

Status impl::addIsolatedCallback(lidar::Callback callback,
                                 void           *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_lidarListeners.push_back(new LidarListener(callback,
                                                     0,
                                                     userDataP,
                                                     MAX_USER_LASER_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new PPS listener

Status impl::addIsolatedCallback(pps::Callback callback,
                                 void         *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_ppsListeners.push_back(new PpsListener(callback,
                                                 0,
                                                 userDataP,
                                                 MAX_USER_PPS_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new IMU listener

Status impl::addIsolatedCallback(imu::Callback callback,
                                 void         *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_imuListeners.push_back(new ImuListener(callback,
                                                 0,
                                                 userDataP,
                                                 MAX_USER_IMU_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new Compressed Image listener

Status impl::addIsolatedCallback(compressed_image::Callback callback,
                                 DataSource     imageSourceMask,
                                 void         *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_compressedImageListeners.push_back(new CompressedImageListener(callback,
                                                                         imageSourceMask,
                                                                         userDataP,
                                                                         MAX_USER_COMPRESSED_IMAGE_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new Ground Surface listener

Status impl::addIsolatedCallback(ground_surface::Callback callback,
                                 void *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_groundSurfaceSplineListeners.push_back(new GroundSurfaceSplineListener(callback,
                                                 0,
                                                 userDataP,
                                                 MAX_USER_GROUND_SURFACE_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new april tag listener

Status impl::addIsolatedCallback(apriltag::Callback callback,
                                 void *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_aprilTagDetectionListeners.push_back(new AprilTagDetectionListener(callback,
                                               0,
                                               userDataP,
                                               MAX_USER_APRILTAG_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Removes an image listener

Status impl::removeIsolatedCallback(image::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<ImageListener*>::iterator it;
        for(it  = m_imageListeners.begin();
            it != m_imageListeners.end();
            it ++) {

            if ((*it)->callback() == callback) {
                delete *it;
                m_imageListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes a lidar listener

Status impl::removeIsolatedCallback(lidar::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<LidarListener*>::iterator it;
        for(it  = m_lidarListeners.begin();
            it != m_lidarListeners.end();
            it ++) {

            if ((*it)->callback() == callback) {
                delete *it;
                m_lidarListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes a PPS listener

Status impl::removeIsolatedCallback(pps::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<PpsListener*>::iterator it;
        for(it  = m_ppsListeners.begin();
            it != m_ppsListeners.end();
            it ++) {

            if ((*it)->callback() == callback) {
                delete *it;
                m_ppsListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes an IMU listener

Status impl::removeIsolatedCallback(imu::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<ImuListener*>::iterator it;
        for(it  = m_imuListeners.begin();
            it != m_imuListeners.end();
            it ++) {

            if ((*it)->callback() == callback) {
                delete *it;
                m_imuListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes a compressed image listener

Status impl::removeIsolatedCallback(compressed_image::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<CompressedImageListener*>::iterator it;
        for(it  = m_compressedImageListeners.begin();
            it != m_compressedImageListeners.end();
            it ++) {

            if ((*it)->callback() == callback) {
                delete *it;
                m_compressedImageListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes a ground surface listener

Status impl::removeIsolatedCallback(ground_surface::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<GroundSurfaceSplineListener*>::iterator it;
        for(it  = m_groundSurfaceSplineListeners.begin();
            it != m_groundSurfaceSplineListeners.end();
            it ++) {

            if ((*it)->callback() == callback) {
                delete *it;
                m_groundSurfaceSplineListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes a ground surface listener

Status impl::removeIsolatedCallback(apriltag::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<AprilTagDetectionListener*>::iterator it;
        for(it  = m_aprilTagDetectionListeners.begin();
            it != m_aprilTagDetectionListeners.end();
            it ++) {

            if ((*it)->callback() == callback) {
                delete *it;
                m_aprilTagDetectionListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Reserve the current callback buffer being used in a dispatch thread

void *impl::reserveCallbackBuffer()
{
    if (dispatchBufferReferenceTP)  {

        try {

            return reinterpret_cast<void*>(new utility::BufferStream(*dispatchBufferReferenceTP));

        } catch (const std::exception& e) {

            CRL_DEBUG("exception: %s\n", e.what());

        } catch (...) {

            CRL_DEBUG_RAW("unknown exception\n");
        }
    }

    return NULL;
}

//
// Release a user reserved buffer back to us

Status impl::releaseCallbackBuffer(void *referenceP)
{
    if (referenceP) {
        try {

            delete reinterpret_cast<utility::BufferStream*>(referenceP);
            return Status_Ok;

        } catch (const std::exception& e) {

            CRL_DEBUG("exception: %s\n", e.what());
            return Status_Exception;

        } catch (...) {

            CRL_DEBUG_RAW("unknown exception\n");
            return Status_Exception;
        }
    }

    return Status_Error;
}

//
// Get a copy of the histogram for a particular frame ID

Status impl::getImageHistogram(int64_t           frameId,
                               image::Histogram& histogram)
{
    try {

        utility::ScopedLock lock(m_imageMetaCache.mutex());

        const wire::ImageMeta *metaP = m_imageMetaCache.find_nolock(frameId);
        if (NULL == metaP) {
            CRL_DEBUG("no meta cached for frameId %ld",
                      static_cast<long int>(frameId));
            return Status_Failed;
        }

        histogram.channels = wire::ImageMeta::HISTOGRAM_CHANNELS;
        histogram.bins     = wire::ImageMeta::HISTOGRAM_BINS;

        const int entries   = histogram.channels * histogram.bins;
        const int sizeBytes = entries * sizeof(uint32_t);

        histogram.data.resize(entries);
        memcpy(&(histogram.data[0]), metaP->histogramP, sizeBytes);

        return Status_Ok;

    }
    catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    catch (...) {
        CRL_DEBUG ("%s\n", "unknown exception");
    }

    return Status_Error;
}

Status impl::getPtpStatus(int64_t frameId,
                          system::PtpStatus &ptpStatus)
{
    try {

        utility::ScopedLock lock(m_imageMetaCache.mutex());

        const wire::ImageMeta *metaP = m_imageMetaCache.find_nolock(frameId);
        if (NULL == metaP) {
            CRL_DEBUG("no meta cached for frameId %ld",
                      static_cast<long int>(frameId));
            return Status_Failed;
        }

        ptpStatus = system::PtpStatus();

        return Status_Ok;

    }
    catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    catch (...) {
        CRL_DEBUG ("%s\n", "unknown exception");
    }

    return Status_Error;
}

//
// Enable/disable network-based time synchronization

Status impl::networkTimeSynchronization(bool enabled)
{
    m_networkTimeSyncEnabled = enabled;
    return Status_Ok;
}

//
// Enable/disable PTP time synchronization

Status impl::ptpTimeSynchronization(bool enabled)
{
    Status status;

    wire::SysSetPtp cmd;
    cmd.enable = enabled ? 1 : 0;

    status = waitAck(cmd);

    if (Status_Ok == status)
        m_ptpTimeSyncEnabled = enabled;

    return status;
}

//
// Primary stream control

Status impl::startStreams(DataSource mask)
{
    utility::ScopedLock lock(m_streamLock);

    wire::StreamControl cmd;

    cmd.enable(sourceApiToWire(mask));

    Status status = waitAck(cmd);
    if (Status_Ok == status)
        m_streamsEnabled |= mask;

    return status;
}
Status impl::stopStreams(DataSource mask)
{
    utility::ScopedLock lock(m_streamLock);

    wire::StreamControl cmd;

    cmd.disable(sourceApiToWire(mask));

    Status status = waitAck(cmd);
    if (Status_Ok == status)
        m_streamsEnabled &= ~mask;

    return status;
}
Status impl::getEnabledStreams(DataSource& mask)
{
    utility::ScopedLock lock(m_streamLock);

    mask = m_streamsEnabled;

    return Status_Ok;
}

//
// Set the trigger source

Status impl::setTriggerSource(TriggerSource s)
{
    uint32_t wireSource;

    switch(s) {
    case Trigger_Internal:

        wireSource = wire::CamSetTriggerSource::SOURCE_INTERNAL;
        break;

    case Trigger_External:

        wireSource = wire::CamSetTriggerSource::SOURCE_EXTERNAL;
        break;

    case Trigger_External_Inverted:

        wireSource = wire::CamSetTriggerSource::SOURCE_EXTERNAL_INVERTED;
        break;

    case Trigger_PTP:

        wireSource = wire::CamSetTriggerSource::SOURCE_PTP;
        break;

    default:

        return Status_Error;
    }

    return waitAck(wire::CamSetTriggerSource(wireSource));
}

//
// Set the motor speed

Status impl::setMotorSpeed(float rpm)
{
    return waitAck(wire::LidarSetMotor(rpm));
}

//
// Get/set the lighting configuration

Status impl::getLightingConfig(lighting::Config& c)
{
    Status          status;
    wire::LedStatus data;

    status = waitData(wire::LedGetStatus(), data);
    if (Status_Ok != status)
        return status;

    for(uint32_t i=0; i<lighting::MAX_LIGHTS; i++) {
        float duty=0.0f;
        if ((1<<i) & data.available)
            duty = (data.intensity[i] * 100.0f) / 255;
        c.setDutyCycle(i, duty);
    }

    c.setFlash(data.flash != 0);

    c.setNumberOfPulses(data.number_of_pulses);

    c.setStartupTime(data.led_delay_us);

    c.setInvertPulse(data.invert_pulse != 0);

    c.enableRollingShutterLedSynchronization(data.rolling_shutter_led != 0);

    return Status_Ok;
}

Status impl::setLightingConfig(const lighting::Config& c)
{
    wire::LedSet msg;

    msg.flash = c.getFlash() ? 1 : 0;
    for(uint32_t i=0; i<lighting::MAX_LIGHTS; i++) {

        float duty = c.getDutyCycle(i);
        if (duty >= 0.0f) {  // less than zero == not set
            msg.mask |= (1<<i);
            msg.intensity[i] = static_cast<uint8_t> (255.0f * (utility::boundValue(duty, 0.0f, 100.0f) / 100.0f));
        }
    }

    msg.led_delay_us = c.getStartupTime();

    msg.number_of_pulses = c.getNumberOfPulses();

    msg.invert_pulse = c.getInvertPulse() ? 1 : 0;

    msg.rolling_shutter_led = c.getRollingShutterLedSynchronizationStatus() ? 1 : 0;

    return waitAck(msg);
}

Status impl::getLightingSensorStatus(lighting::SensorStatus& status)
{
    Status          requestStatus;
    wire::LedSensorStatus data;

    requestStatus = waitData(wire::LedGetSensorStatus(), data);
    if (Status_Ok != requestStatus)
        return requestStatus;

    status.ambientLightPercentage = data.ambientLightPercentage;

    return Status_Ok;
}

//
// Requests version from sensor

Status impl::getSensorVersion(VersionType& version)
{
    version = m_sensorVersion.firmwareVersion;
    return Status_Ok;
}

//
// Version of this API

Status impl::getApiVersion(VersionType& version)
{
    version = API_VERSION;
    return Status_Ok;
}

//
// Requests all versioning information

Status impl::getVersionInfo(system::VersionInfo& v)
{
    v.apiBuildDate            = std::string(__DATE__ ", " __TIME__);
    v.apiVersion              = API_VERSION;
    v.sensorFirmwareBuildDate = m_sensorVersion.firmwareBuildDate;
    v.sensorFirmwareVersion   = m_sensorVersion.firmwareVersion;
    v.sensorHardwareVersion   = m_sensorVersion.hardwareVersion;
    v.sensorHardwareMagic     = m_sensorVersion.hardwareMagic;
    v.sensorFpgaDna           = m_sensorVersion.fpgaDna;

    return Status_Ok;
}

//
// Query camera configuration

Status impl::getImageConfig(image::Config& config)
{
    Status          status;
    wire::CamConfig d;

    status = waitData(wire::CamGetConfig(), d);
    if (Status_Ok != status)
        return status;

    // for access to protected calibration members
    class ConfigAccess : public image::Config {
    public:
        void setCal(float fx, float fy, float cx, float cy,
                    float tx, float ty, float tz,
                    float r,  float p,  float w) {
            m_fx = fx; m_fy = fy; m_cx = cx; m_cy = cy;
            m_tx = tx; m_ty = ty; m_tz = tz;
            m_roll = r; m_pitch = p; m_yaw = w;
        };
    };

    // what is the proper c++ cast for this?
    ConfigAccess& a = *((ConfigAccess *) &config);

    a.setResolution(d.width, d.height);
    if (-1 == d.disparities) { // pre v2.3 firmware
        if (1024 == d.width)   // TODO: check for monocular
            d.disparities = 128;
        else
            d.disparities = 0;
    }
    a.setDisparities(d.disparities);
    a.setFps(d.framesPerSecond);
    a.setGain(d.gain);

    a.setExposure(d.exposure);
    a.setAutoExposure(d.autoExposure != 0);
    a.setAutoExposureMax(d.autoExposureMax);
    a.setAutoExposureDecay(d.autoExposureDecay);
    a.setAutoExposureTargetIntensity(d.autoExposureTargetIntensity);
    a.setAutoExposureThresh(d.autoExposureThresh);
    a.setGain(d.gain);

    a.setWhiteBalance(d.whiteBalanceRed, d.whiteBalanceBlue);
    a.setAutoWhiteBalance(d.autoWhiteBalance != 0);
    a.setAutoWhiteBalanceDecay(d.autoWhiteBalanceDecay);
    a.setAutoWhiteBalanceThresh(d.autoWhiteBalanceThresh);
    a.setStereoPostFilterStrength(d.stereoPostFilterStrength);
    a.setHdr(d.hdrEnabled);

    a.setAutoExposureRoi(d.autoExposureRoiX, d.autoExposureRoiY,
                         d.autoExposureRoiWidth, d.autoExposureRoiHeight);

    a.setCal(d.fx, d.fy, d.cx, d.cy,
             d.tx, d.ty, d.tz,
             d.roll, d.pitch, d.yaw);

    a.setCameraProfile(static_cast<CameraProfile>(d.cameraProfile));

    a.setGamma(d.gamma);

    return Status_Ok;
}

Status impl::getAuxImageConfig(image::AuxConfig& config)
{
    Status             status;
    wire::AuxCamConfig d;

    // for access to protected calibration members
    class ConfigAccess : public image::AuxConfig {
    public:
        void setCal(float fx, float fy, float cx, float cy) {
            m_fx = fx; m_fy = fy; m_cx = cx; m_cy = cy;
        };
    };

    // what is the proper c++ cast for this?
    ConfigAccess& a = *((ConfigAccess *) &config);

    status = waitData(wire::AuxCamGetConfig(), d);
    if (Status_Ok != status)
        return status;

    a.setGain(d.gain);

    a.setExposure(d.exposure);
    a.setAutoExposure(d.autoExposure != 0);
    a.setAutoExposureMax(d.autoExposureMax);
    a.setAutoExposureDecay(d.autoExposureDecay);
    a.setAutoExposureTargetIntensity(d.autoExposureTargetIntensity);
    a.setAutoExposureThresh(d.autoExposureThresh);

    a.setGain(d.gain);

    a.setWhiteBalance(d.whiteBalanceRed, d.whiteBalanceBlue);
    a.setAutoWhiteBalance(d.autoWhiteBalance != 0);
    a.setAutoWhiteBalanceDecay(d.autoWhiteBalanceDecay);
    a.setAutoWhiteBalanceThresh(d.autoWhiteBalanceThresh);
    a.setHdr(d.hdrEnabled);

    a.setAutoExposureRoi(d.autoExposureRoiX, d.autoExposureRoiY,
                         d.autoExposureRoiWidth, d.autoExposureRoiHeight);

    a.setCal(d.fx, d.fy, d.cx, d.cy);

    a.setCameraProfile(static_cast<CameraProfile>(d.cameraProfile));

    a.setGamma(d.gamma);

    a.enableSharpening(d.sharpeningEnable);
    a.setSharpeningPercentage(d.sharpeningPercentage);
    a.setSharpeningLimit(d.sharpeningLimit);

    return Status_Ok;
}

//
// Set camera configuration
//
// Currently several sensor messages are combined and presented
// to the user as one.

Status impl::setImageConfig(const image::Config& c)
{
    Status status;

    status = waitAck(wire::CamSetResolution(c.width(),
                                            c.height(),
                                            c.disparities()));
    if (Status_Ok != status)
        return status;

    wire::CamControl cmd;

    cmd.framesPerSecond = c.fps();
    cmd.gain            = c.gain();

    cmd.exposure                    = c.exposure();
    cmd.autoExposure                = c.autoExposure() ? 1 : 0;
    cmd.autoExposureMax             = c.autoExposureMax();
    cmd.autoExposureDecay           = c.autoExposureDecay();
    cmd.autoExposureThresh          = c.autoExposureThresh();
    cmd.autoExposureTargetIntensity = c.autoExposureTargetIntensity();

    cmd.whiteBalanceRed          = c.whiteBalanceRed();
    cmd.whiteBalanceBlue         = c.whiteBalanceBlue();
    cmd.autoWhiteBalance         = c.autoWhiteBalance() ? 1 : 0;
    cmd.autoWhiteBalanceDecay    = c.autoWhiteBalanceDecay();
    cmd.autoWhiteBalanceThresh   = c.autoWhiteBalanceThresh();
    cmd.stereoPostFilterStrength = c.stereoPostFilterStrength();
    cmd.hdrEnabled               = c.hdrEnabled();

    cmd.autoExposureRoiX         = c.autoExposureRoiX();
    cmd.autoExposureRoiY         = c.autoExposureRoiY();
    cmd.autoExposureRoiWidth     = c.autoExposureRoiWidth();
    cmd.autoExposureRoiHeight    = c.autoExposureRoiHeight();

    cmd.cameraProfile    = static_cast<uint32_t>(c.cameraProfile());

    cmd.gamma = c.gamma();

    return waitAck(cmd);
}

Status impl::setAuxImageConfig(const image::AuxConfig& c)
{
    wire::AuxCamControl cmd;

    cmd.gain            = c.gain();

    cmd.exposure                    = c.exposure();
    cmd.autoExposure                = c.autoExposure() ? 1 : 0;
    cmd.autoExposureMax             = c.autoExposureMax();
    cmd.autoExposureDecay           = c.autoExposureDecay();
    cmd.autoExposureThresh          = c.autoExposureThresh();
    cmd.autoExposureTargetIntensity = c.autoExposureTargetIntensity();

    cmd.whiteBalanceRed          = c.whiteBalanceRed();
    cmd.whiteBalanceBlue         = c.whiteBalanceBlue();
    cmd.autoWhiteBalance         = c.autoWhiteBalance() ? 1 : 0;
    cmd.autoWhiteBalanceDecay    = c.autoWhiteBalanceDecay();
    cmd.autoWhiteBalanceThresh   = c.autoWhiteBalanceThresh();
    cmd.hdrEnabled               = c.hdrEnabled();

    cmd.autoExposureRoiX         = c.autoExposureRoiX();
    cmd.autoExposureRoiY         = c.autoExposureRoiY();
    cmd.autoExposureRoiWidth     = c.autoExposureRoiWidth();
    cmd.autoExposureRoiHeight    = c.autoExposureRoiHeight();

    cmd.cameraProfile    = static_cast<uint32_t>(c.cameraProfile());

    cmd.gamma = c.gamma();
    cmd.sharpeningEnable = c.enableSharpening();
    cmd.sharpeningPercentage = c.sharpeningPercentage();
    cmd.sharpeningLimit = c.sharpeningLimit();

    return waitAck(cmd);
}

//
// Get Remote Head Configuration
Status impl::getRemoteHeadConfig(image::RemoteHeadConfig& c)
{
    Status                 status;
    wire::RemoteHeadConfig r;

    status = waitData(wire::RemoteHeadGetConfig(), r);
    if (Status_Ok != status)
        return status;

    c.m_syncPair1.controller = r.syncPair1.controller;
    c.m_syncPair1.responder  = r.syncPair1.responder;

    c.m_syncPair2.controller = r.syncPair2.controller;
    c.m_syncPair2.responder  = r.syncPair2.responder;

    return status;
}

//
// Set Remote Head Configuration
Status impl::setRemoteHeadConfig(const image::RemoteHeadConfig& c)
{
    wire::RemoteHeadControl cmd;

    cmd.syncPair1.controller = c.syncPair1Controller();
    cmd.syncPair1.responder  = c.syncPair1Responder();
    cmd.syncPair2.controller = c.syncPair2Controller();
    cmd.syncPair2.responder  = c.syncPair2Responder();

    return waitAck(cmd);
}



//
// Get camera calibration

Status impl::getImageCalibration(image::Calibration& c)
{
    wire::SysCameraCalibration d;

    Status status = waitData(wire::SysGetCameraCalibration(), d);
    if (Status_Ok != status)
        return status;

    CPY_ARRAY_2(c.left.M, d.left.M, 3, 3);
    CPY_ARRAY_1(c.left.D, d.left.D, 8);
    CPY_ARRAY_2(c.left.R, d.left.R, 3, 3);
    CPY_ARRAY_2(c.left.P, d.left.P, 3, 4);

    CPY_ARRAY_2(c.right.M, d.right.M, 3, 3);
    CPY_ARRAY_1(c.right.D, d.right.D, 8);
    CPY_ARRAY_2(c.right.R, d.right.R, 3, 3);
    CPY_ARRAY_2(c.right.P, d.right.P, 3, 4);

    CPY_ARRAY_2(c.aux.M, d.aux.M, 3, 3);
    CPY_ARRAY_1(c.aux.D, d.aux.D, 8);
    CPY_ARRAY_2(c.aux.R, d.aux.R, 3, 3);
    CPY_ARRAY_2(c.aux.P, d.aux.P, 3, 4);

    return Status_Ok;
}

//
// Set camera calibration

Status impl::setImageCalibration(const image::Calibration& c)
{
    wire::SysCameraCalibration d;

    CPY_ARRAY_2(d.left.M, c.left.M, 3, 3);
    CPY_ARRAY_1(d.left.D, c.left.D, 8);
    CPY_ARRAY_2(d.left.R, c.left.R, 3, 3);
    CPY_ARRAY_2(d.left.P, c.left.P, 3, 4);

    CPY_ARRAY_2(d.right.M, c.right.M, 3, 3);
    CPY_ARRAY_1(d.right.D, c.right.D, 8);
    CPY_ARRAY_2(d.right.R, c.right.R, 3, 3);
    CPY_ARRAY_2(d.right.P, c.right.P, 3, 4);

    CPY_ARRAY_2(d.aux.M, c.aux.M, 3, 3);
    CPY_ARRAY_1(d.aux.D, c.aux.D, 8);
    CPY_ARRAY_2(d.aux.R, c.aux.R, 3, 3);
    CPY_ARRAY_2(d.aux.P, c.aux.P, 3, 4);

    return waitAck(d);
}

//
// Get sensor calibration

Status impl::getTransmitDelay(image::TransmitDelay& c)
{
    wire::SysTransmitDelay d(0);

    Status status = waitData(wire::SysGetTransmitDelay(), d);
    if (Status_Ok != status)
        return status;
    c.delay = d.delay;

    return Status_Ok;
}

//
// Set sensor calibration

Status impl::setTransmitDelay(const image::TransmitDelay& c)
{
    wire::SysTransmitDelay d;

    d.delay = c.delay;;

    return waitAck(d);
}

//
// Get lidar calibration

Status impl::getLidarCalibration(lidar::Calibration& c)
{
    wire::SysLidarCalibration d;

    Status status = waitData(wire::SysGetLidarCalibration(), d);
    if (Status_Ok != status)
        return status;

    CPY_ARRAY_2(c.laserToSpindle, d.laserToSpindle, 4, 4);
    CPY_ARRAY_2(c.cameraToSpindleFixed, d.cameraToSpindleFixed, 4, 4);

    return Status_Ok;
}

//
// Set lidar calibration

Status impl::setLidarCalibration(const lidar::Calibration& c)
{
    wire::SysLidarCalibration d;

    CPY_ARRAY_2(d.laserToSpindle, c.laserToSpindle, 4, 4);
    CPY_ARRAY_2(d.cameraToSpindleFixed, c.cameraToSpindleFixed, 4, 4);

    return waitAck(d);
}

//
// Get a list of supported image formats / data sources

Status impl::getDeviceModes(std::vector<system::DeviceMode>& modes)
{
    wire::SysDeviceModes d;

    Status status = waitData(wire::SysGetDeviceModes(), d);
    if (Status_Ok != status)
        return Status_Error;

    modes.resize(d.modes.size());
    for(uint32_t i=0; i<d.modes.size(); i++) {

        system::DeviceMode&     a = modes[i];
        const wire::DeviceMode& w = d.modes[i];

        a.width                = w.width;
        a.height               = w.height;
        a.supportedDataSources = sourceWireToApi(w.supportedDataSources);
        if (m_sensorVersion.firmwareVersion >= 0x0203)
            a.disparities = w.disparities;
        else
            a.disparities = (a.width == 1024) ? 128 : 0;
    }

    return Status_Ok;
}

//
// Set/get the mtu

Status impl::setMtu(int32_t mtu)
{
    Status status = Status_Ok;

    //
    // Firmware v2.3 or better will send us an MTU-sized
    // response packet, which can be used to verify the
    // MTU setting before we actually make the change.

    if (m_sensorVersion.firmwareVersion <= 0x0202)
      status = waitAck(wire::SysMtu(mtu));
    else {
      wire::SysTestMtuResponse resp;
      status = waitData(wire::SysTestMtu(mtu), resp);
      if (Status_Ok == status)
        status = waitAck(wire::SysMtu(mtu));
    }

    if (Status_Ok == status)
        m_sensorMtu = mtu;

    return status;
}

Status impl::getMtu(int32_t& mtu)
{
    wire::SysMtu resp;

    Status status = waitData(wire::SysGetMtu(), resp);
    if (Status_Ok == status)
        mtu = resp.mtu;

    return status;
}

Status impl::getMotorPos(int32_t& pos)
{
    wire::MotorPoll resp;

    Status status = waitData(wire::LidarPollMotor(), resp);
    if (Status_Ok == status)
    	pos = resp.angleStart;

    return status;
}

//
// Set/get the network configuration

Status impl::setNetworkConfig(const system::NetworkConfig& c)
{
    return waitAck(wire::SysNetwork(c.ipv4Address,
                                    c.ipv4Gateway,
                                    c.ipv4Netmask));
}

Status impl::getNetworkConfig(system::NetworkConfig& c)
{
    wire::SysNetwork resp;

    Status status = waitData(wire::SysGetNetwork(), resp);
    if (Status_Ok == status) {
        c.ipv4Address = resp.address;
        c.ipv4Gateway = resp.gateway;
        c.ipv4Netmask = resp.netmask;
    }

    return status;
}

//
// Get device info from sensor

Status impl::getDeviceInfo(system::DeviceInfo& info)
{
    wire::SysDeviceInfo w;

    Status status = waitData(wire::SysGetDeviceInfo(), w);
    if (Status_Ok != status)
        return status;

    info.name             = w.name;
    info.buildDate        = w.buildDate;
    info.serialNumber     = w.serialNumber;
    info.hardwareRevision = hardwareWireToApi(w.hardwareRevision);
    info.pcbs.clear();

    for(uint8_t i=0; i<w.numberOfPcbs; i++) {
        system::PcbInfo pcb;

        pcb.name     = w.pcbs[i].name;
        pcb.revision = w.pcbs[i].revision;

        info.pcbs.push_back(pcb);
    }

    info.imagerName              = w.imagerName;
    info.imagerType              = imagerWireToApi(w.imagerType);
    info.imagerWidth             = w.imagerWidth;
    info.imagerHeight            = w.imagerHeight;
    info.lensName                = w.lensName;
    info.lensType                = w.lensType;
    info.nominalBaseline         = w.nominalBaseline;
    info.nominalFocalLength      = w.nominalFocalLength;
    info.nominalRelativeAperture = w.nominalRelativeAperture;
    info.lightingType            = w.lightingType;
    info.numberOfLights          = w.numberOfLights;
    info.laserName               = w.laserName;
    info.laserType               = w.laserType;
    info.motorName               = w.motorName;
    info.motorType               = w.motorType;
    info.motorGearReduction      = w.motorGearReduction;

    return Status_Ok;
}


Status impl::getDeviceStatus(system::StatusMessage& status)
{
    if (m_getStatusReturnStatus != Status_Ok){
        return m_getStatusReturnStatus;
    }

    status.uptime = static_cast<double>(m_statusResponseMessage.uptime.getNanoSeconds()) * 1e-9;

    status.systemOk = (m_statusResponseMessage.status & wire::StatusResponse::STATUS_GENERAL_OK) ==
        wire::StatusResponse::STATUS_GENERAL_OK;
    status.laserOk = (m_statusResponseMessage.status & wire::StatusResponse::STATUS_LASER_OK) ==
        wire::StatusResponse::STATUS_LASER_OK;
    status.laserMotorOk = (m_statusResponseMessage.status & wire::StatusResponse::STATUS_LASER_MOTOR_OK) ==
        wire::StatusResponse::STATUS_LASER_MOTOR_OK;
    status.camerasOk = (m_statusResponseMessage.status & wire::StatusResponse::STATUS_CAMERAS_OK) ==
        wire::StatusResponse::STATUS_CAMERAS_OK;
    status.imuOk = (m_statusResponseMessage.status & wire::StatusResponse::STATUS_IMU_OK) ==
        wire::StatusResponse::STATUS_IMU_OK;
    status.externalLedsOk = (m_statusResponseMessage.status & wire::StatusResponse::STATUS_EXTERNAL_LED_OK) ==
        wire::StatusResponse::STATUS_EXTERNAL_LED_OK;
    status.processingPipelineOk = (m_statusResponseMessage.status & wire::StatusResponse::STATUS_PIPELINE_OK) ==
        wire::StatusResponse::STATUS_PIPELINE_OK;

    status.powerSupplyTemperature = m_statusResponseMessage.temperature0;
    status.fpgaTemperature = m_statusResponseMessage.temperature1;
    status.leftImagerTemperature = m_statusResponseMessage.temperature2;
    status.rightImagerTemperature = m_statusResponseMessage.temperature3;

    status.inputVoltage = m_statusResponseMessage.inputVolts;
    status.inputCurrent = m_statusResponseMessage.inputCurrent;
    status.fpgaPower = m_statusResponseMessage.fpgaPower;
    status.logicPower = m_statusResponseMessage.logicPower;
    status.imagerPower = m_statusResponseMessage.imagerPower;

    return Status_Ok;
}

Status impl::getExternalCalibration(system::ExternalCalibration& calibration)
{
    wire::SysExternalCalibration d;

    Status status = waitData(wire::SysGetExternalCalibration(), d);
    if (Status_Ok != status)
        return status;

    calibration.x = d.calibration[0];
    calibration.y = d.calibration[1];
    calibration.z = d.calibration[2];
    calibration.roll = d.calibration[3];
    calibration.pitch = d.calibration[4];
    calibration.yaw = d.calibration[5];

    return Status_Ok;
}

Status impl::setExternalCalibration(const system::ExternalCalibration& calibration)
{
    wire::SysExternalCalibration w;

    w.calibration[0] = calibration.x;
    w.calibration[1] = calibration.y;
    w.calibration[2] = calibration.z;
    w.calibration[3] = calibration.roll;
    w.calibration[4] = calibration.pitch;
    w.calibration[5] = calibration.yaw;

    return waitAck(w);
}

Status impl::setGroundSurfaceParams (const system::GroundSurfaceParams& params)
{
    wire::SysGroundSurfaceParams w;

    w.ground_surface_number_of_levels_x = params.ground_surface_number_of_levels_x;
    w.ground_surface_number_of_levels_z = params.ground_surface_number_of_levels_z;
    w.ground_surface_base_model = params.ground_surface_base_model;
    w.ground_surface_pointcloud_grid_size = params.ground_surface_pointcloud_grid_size;
    w.ground_surface_min_points_per_grid = params.ground_surface_min_points_per_grid;
    w.ground_surface_pointcloud_decimation = params.ground_surface_pointcloud_decimation;
    w.ground_surface_pointcloud_max_range_m = params.ground_surface_pointcloud_max_range_m;
    w.ground_surface_pointcloud_min_range_m = params.ground_surface_pointcloud_min_range_m;
    w.ground_surface_pointcloud_max_width_m = params.ground_surface_pointcloud_max_width_m;
    w.ground_surface_pointcloud_min_width_m = params.ground_surface_pointcloud_min_width_m;
    w.ground_surface_pointcloud_max_height_m = params.ground_surface_pointcloud_max_height_m;
    w.ground_surface_pointcloud_min_height_m = params.ground_surface_pointcloud_min_height_m;
    w.ground_surface_obstacle_height_thresh_m = params.ground_surface_obstacle_height_thresh_m;
    w.ground_surface_obstacle_percentage_thresh = params.ground_surface_obstacle_percentage_thresh;
    w.ground_surface_max_fitting_iterations = params.ground_surface_max_fitting_iterations;
    w.ground_surface_adjacent_cell_search_size_m = params.ground_surface_adjacent_cell_search_size_m;

    return waitAck(w);
}

Status impl::setApriltagParams (const system::ApriltagParams& params)
{
    (void)params;
    wire::SysApriltagParams w;

    w.family = params.family;
    w.max_hamming = params.max_hamming;
    w.quad_detection_blur_sigma = params.quad_detection_blur_sigma;
    w.quad_detection_decimate = params.quad_detection_decimate;
    w.min_border_width = params.min_border_width;
    w.refine_quad_edges = params.refine_quad_edges;
    w.decode_sharpening = params.decode_sharpening;

    return waitAck(w);
}
//
// Sets the device info

Status impl::setDeviceInfo(const std::string& key,
                           const system::DeviceInfo& info)
{
    wire::SysDeviceInfo w;

    w.key              = key; // must match device firmware key
    w.name             = info.name;
    w.buildDate        = info.buildDate;
    w.serialNumber     = info.serialNumber;
    w.hardwareRevision = hardwareApiToWire(info.hardwareRevision);
    w.numberOfPcbs     = std::min((uint8_t) info.pcbs.size(),
                                            wire::SysDeviceInfo::maxPcbs());
    for(uint32_t i=0; i<w.numberOfPcbs; i++) {
        w.pcbs[i].name     = info.pcbs[i].name;
        w.pcbs[i].revision = info.pcbs[i].revision;
    }

    w.imagerName              = info.imagerName;
    w.imagerType              = imagerApiToWire(info.imagerType);
    w.imagerWidth             = info.imagerWidth;
    w.imagerHeight            = info.imagerHeight;
    w.lensName                = info.lensName;
    w.lensType                = info.lensType;
    w.nominalBaseline         = info.nominalBaseline;
    w.nominalFocalLength      = info.nominalFocalLength;
    w.nominalRelativeAperture = info.nominalRelativeAperture;
    w.lightingType            = info.lightingType;
    w.numberOfLights          = info.numberOfLights;
    w.laserName               = info.laserName;
    w.laserType               = info.laserType;
    w.motorName               = info.motorName;
    w.motorType               = info.motorType;
    w.motorGearReduction      = info.motorGearReduction;

    return waitAck(w);
}

//
// Flash the bitstream file (dangerous!)

Status impl::flashBitstream(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_PROGRAM,
                     wire::SysFlashOp::RGN_BITSTREAM);
}

//
// Flash the firmware file (dangerous!)

Status impl::flashFirmware(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_PROGRAM,
                     wire::SysFlashOp::RGN_FIRMWARE);
}

//
// Verify the bitstream file

Status impl::verifyBitstream(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_VERIFY,
                     wire::SysFlashOp::RGN_BITSTREAM);
}

//
// Verify the firmware file

Status impl::verifyFirmware(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_VERIFY,
                     wire::SysFlashOp::RGN_FIRMWARE);
}

//
// Get IMU information

Status impl::getImuInfo(uint32_t& maxSamplesPerMessage,
                        std::vector<imu::Info>& info)
{
    wire::ImuInfo w;

    Status status = waitData(wire::ImuGetInfo(), w);
    if (Status_Ok != status)
        return status;

    //
    // Wire --> API

    maxSamplesPerMessage = w.maxSamplesPerMessage;
    info.resize(w.details.size());
    for(uint32_t i=0; i<w.details.size(); i++) {

        const wire::imu::Details& d = w.details[i];

        info[i].name   = d.name;
        info[i].device = d.device;
        info[i].units  = d.units;

        info[i].rates.resize(d.rates.size());
        for(uint32_t j=0; j<d.rates.size(); j++) {
            info[i].rates[j].sampleRate      = d.rates[j].sampleRate;
            info[i].rates[j].bandwidthCutoff = d.rates[j].bandwidthCutoff;
        }
        info[i].ranges.resize(d.ranges.size());
        for(uint32_t j=0; j<d.ranges.size(); j++) {
            info[i].ranges[j].range      = d.ranges[j].range;
            info[i].ranges[j].resolution = d.ranges[j].resolution;
        }
    }

    return Status_Ok;
}

//
// Get IMU configuration

Status impl::getImuConfig(uint32_t& samplesPerMessage,
                          std::vector<imu::Config>& c)
{
    wire::ImuConfig w;

    Status status = waitData(wire::ImuGetConfig(), w);
    if (Status_Ok != status)
        return status;

    //
    // Wire --> API

    samplesPerMessage = w.samplesPerMessage;
    c.resize(w.configs.size());
    for(uint32_t i=0; i<w.configs.size(); i++) {
        c[i].name            = w.configs[i].name;
        c[i].enabled         = (w.configs[i].flags & wire::imu::Config::FLAGS_ENABLED);
        c[i].rateTableIndex  = w.configs[i].rateTableIndex;
        c[i].rangeTableIndex = w.configs[i].rangeTableIndex;
    }

    return Status_Ok;
}

//
// Set IMU configuration

Status impl::setImuConfig(bool storeSettingsInFlash,
                          uint32_t samplesPerMessage,
                          const std::vector<imu::Config>& c)
{
    wire::ImuConfig w;

    //
    // API --> wire

    w.storeSettingsInFlash = storeSettingsInFlash ? 1 : 0;
    w.samplesPerMessage    = samplesPerMessage;
    w.configs.resize(c.size());
    for(uint32_t i=0; i<c.size(); i++) {
        w.configs[i].name            = c[i].name;
        w.configs[i].flags           = c[i].enabled ? wire::imu::Config::FLAGS_ENABLED : 0;
        w.configs[i].rateTableIndex  = c[i].rateTableIndex;
        w.configs[i].rangeTableIndex = c[i].rangeTableIndex;
    }

    return waitAck(w);
}

//
// Get recommended large buffer pool count/size

Status impl::getLargeBufferDetails(uint32_t& bufferCount,
                                   uint32_t& bufferSize)
{
    bufferCount = RX_POOL_LARGE_BUFFER_COUNT;
    bufferSize  = RX_POOL_LARGE_BUFFER_SIZE;

    return Status_Ok;
}

//
// Replace internal large buffers with user supplied

Status impl::setLargeBuffers(const std::vector<uint8_t*>& buffers,
                             uint32_t                     bufferSize)
{
    if (buffers.size() < RX_POOL_LARGE_BUFFER_COUNT)
        CRL_DEBUG("WARNING: supplying less than recommended number of large buffers: %ld/%ld\n",
                  static_cast<long int>(buffers.size()),
                  static_cast<long int>(RX_POOL_LARGE_BUFFER_COUNT));
    if (bufferSize < RX_POOL_LARGE_BUFFER_SIZE)
        CRL_DEBUG("WARNING: supplying smaller than recommended large buffers: %ld/%ld bytes\n",
                  static_cast<long int>(bufferSize),
                  static_cast<long int>(RX_POOL_LARGE_BUFFER_SIZE));

    try {

        utility::ScopedLock lock(m_rxLock); // halt potential pool traversal

        //
        // Deletion is safe even if the buffer is in use elsewhere
        // (BufferStream is reference counted.)

        BufferPool::const_iterator it;
        for(it  = m_rxLargeBufferPool.begin();
            it != m_rxLargeBufferPool.end();
            ++it)
            delete *it;

        m_rxLargeBufferPool.clear();

        for(uint32_t i=0; i<buffers.size(); i++)
            m_rxLargeBufferPool.push_back(new utility::BufferStreamWriter(buffers[i], bufferSize));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Ok;
}

//
// Retrieve the system-assigned local UDP port

Status impl::getLocalUdpPort(uint16_t& port)
{
    port = m_serverSocketPort;
    return Status_Ok;
}
}}} // namespaces
