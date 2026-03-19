/**
 * @file LibMultiSense/details/constants.cc
 *
 * Copyright 2021-2025
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
 *   2021-05-11, wdouglass@carnegierobotics.com, 8000, Created file.
 **/

#include "utility/Portability.hh"
#include "MultiSense/MultiSenseTypes.hh"

#include <wire/Protocol.hh>
#include <utility/BufferStream.hh>
#include <wire/AckMessage.hh>
#include <wire/CamConfigMessage.hh>
#include <wire/CamControlMessage.hh>
#include <wire/CamGetConfigMessage.hh>
#include <wire/AuxCamConfigMessage.hh>
#include <wire/AuxCamControlMessage.hh>
#include <wire/AuxCamGetConfigMessage.hh>
#include <wire/RemoteHeadConfigMessage.hh>
#include <wire/RemoteHeadControlMessage.hh>
#include <wire/RemoteHeadGetConfigMessage.hh>
#include <wire/CamGetHistoryMessage.hh>
#include <wire/CamHistoryMessage.hh>
#include <wire/CamSetResolutionMessage.hh>
#include <wire/CamSetTriggerSourceMessage.hh>
#include <wire/CompressedImageMessage.hh>
#include <wire/DisparityMessage.hh>
#include <wire/ExposureConfigMessage.hh>
#include <wire/ImageMessage.hh>
#include <wire/ImageMetaMessage.hh>
#include <wire/ImuConfigMessage.hh>
#include <wire/ImuDataMessage.hh>
#include <wire/ImuGetConfigMessage.hh>
#include <wire/ImuGetInfoMessage.hh>
#include <wire/ImuInfoMessage.hh>
#include <wire/JpegMessage.hh>
#include <wire/LedGetSensorStatusMessage.hh>
#include <wire/LedGetStatusMessage.hh>
#include <wire/LedSensorStatusMessage.hh>
#include <wire/LedSetMessage.hh>
#include <wire/LedStatusMessage.hh>
#include <wire/LidarDataMessage.hh>
#include <wire/LidarPollMotorMessage.hh>
#include <wire/LidarSetMotorMessage.hh>
#include <wire/PollMotorInfoMessage.hh>
#include <wire/PtpStatusRequestMessage.hh>
#include <wire/PtpStatusResponseMessage.hh>
#include <wire/StatusRequestMessage.hh>
#include <wire/StatusResponseMessage.hh>
#include <wire/StreamControlMessage.hh>
#include <wire/SysCameraCalibrationMessage.hh>
#include <wire/SysDeviceInfoMessage.hh>
#include <wire/SysDeviceModesMessage.hh>
#include <wire/SysDirectedStreamsMessage.hh>
#include <wire/SysExternalCalibrationMessage.hh>
#include <wire/SysFlashOpMessage.hh>
#include <wire/SysFlashResponseMessage.hh>
#include <wire/SysGetCameraCalibrationMessage.hh>
#include <wire/SysGetDeviceInfoMessage.hh>
#include <wire/SysGetDeviceModesMessage.hh>
#include <wire/SysGetDirectedStreamsMessage.hh>
#include <wire/SysGetExternalCalibrationMessage.hh>
#include <wire/SysGetLidarCalibrationMessage.hh>
#include <wire/SysGetMtuMessage.hh>
#include <wire/SysGetNetworkMessage.hh>
#include <wire/SysGetSensorCalibrationMessage.hh>
#include <wire/SysGetTransmitDelayMessage.hh>
#include <wire/SysGetPacketDelayMessage.hh>
#include <wire/SysLidarCalibrationMessage.hh>
#include <wire/SysMtuMessage.hh>
#include <wire/SysNetworkMessage.hh>
#include <wire/SysPpsMessage.hh>
#include <wire/SysSensorCalibrationMessage.hh>
#include <wire/SysSetPtpMessage.hh>
#include <wire/SysTestMtuMessage.hh>
#include <wire/SysTestMtuResponseMessage.hh>
#include <wire/SysTransmitDelayMessage.hh>
#include <wire/SysPacketDelayMessage.hh>
#include <wire/VersionRequestMessage.hh>
#include <wire/VersionResponseMessage.hh>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

    CRL_CONSTEXPR IdType Ack::ID;
    CRL_CONSTEXPR IdType CamConfig::ID;
    CRL_CONSTEXPR IdType CamControl::ID;
    CRL_CONSTEXPR IdType CamGetConfig::ID;
    CRL_CONSTEXPR IdType AuxCamConfig::ID;
    CRL_CONSTEXPR IdType AuxCamControl::ID;
    CRL_CONSTEXPR IdType AuxCamGetConfig::ID;
    CRL_CONSTEXPR IdType RemoteHeadConfig::ID;
    CRL_CONSTEXPR IdType RemoteHeadControl::ID;
    CRL_CONSTEXPR IdType RemoteHeadGetConfig::ID;
    CRL_CONSTEXPR IdType CamGetHistory::ID;
    CRL_CONSTEXPR IdType CamHistory::ID;
    CRL_CONSTEXPR IdType CamSetResolution::ID;
    CRL_CONSTEXPR IdType CamSetTriggerSource::ID;
    CRL_CONSTEXPR IdType CompressedImageHeader::ID;
    CRL_CONSTEXPR IdType DisparityHeader::ID;
    CRL_CONSTEXPR IdType ExposureConfig::ID;
    CRL_CONSTEXPR IdType ImageHeader::ID;
    CRL_CONSTEXPR IdType ImageMetaHeader::ID;
    CRL_CONSTEXPR IdType ImuConfig::ID;
    CRL_CONSTEXPR IdType ImuData::ID;
    CRL_CONSTEXPR IdType ImuGetConfig::ID;
    CRL_CONSTEXPR IdType ImuGetInfo::ID;
    CRL_CONSTEXPR IdType ImuInfo::ID;
    CRL_CONSTEXPR IdType JpegImageHeader::ID;
    CRL_CONSTEXPR IdType LedGetSensorStatus::ID;
    CRL_CONSTEXPR IdType LedGetStatus::ID;
    CRL_CONSTEXPR IdType LedSensorStatus::ID;
    CRL_CONSTEXPR IdType LedSet::ID;
    CRL_CONSTEXPR IdType LedStatus::ID;
    CRL_CONSTEXPR IdType LidarDataHeader::ID;
    CRL_CONSTEXPR IdType LidarPollMotor::ID;
    CRL_CONSTEXPR IdType LidarSetMotor::ID;
    CRL_CONSTEXPR IdType MotorPoll::ID;
    CRL_CONSTEXPR IdType StatusRequest::ID;
    CRL_CONSTEXPR IdType StatusResponse::ID;
    CRL_CONSTEXPR IdType StreamControl::ID;
    CRL_CONSTEXPR IdType SysCameraCalibration::ID;
    CRL_CONSTEXPR IdType SysDeviceInfo::ID;
    CRL_CONSTEXPR IdType SysDeviceModes::ID;
    CRL_CONSTEXPR IdType SysDirectedStreams::ID;
    CRL_CONSTEXPR IdType SysExternalCalibration::ID;
    CRL_CONSTEXPR IdType SysFlashOp::ID;
    CRL_CONSTEXPR IdType SysFlashResponse::ID;
    CRL_CONSTEXPR IdType SysGetCameraCalibration::ID;
    CRL_CONSTEXPR IdType SysGetDeviceInfo::ID;
    CRL_CONSTEXPR IdType SysGetDeviceModes::ID;
    CRL_CONSTEXPR IdType SysGetDirectedStreams::ID;
    CRL_CONSTEXPR IdType SysGetExternalCalibration::ID;
    CRL_CONSTEXPR IdType SysGetLidarCalibration::ID;
    CRL_CONSTEXPR IdType SysGetMtu::ID;
    CRL_CONSTEXPR IdType SysGetNetwork::ID;
    CRL_CONSTEXPR IdType SysGetSensorCalibration::ID;
    CRL_CONSTEXPR IdType SysGetTransmitDelay::ID;
    CRL_CONSTEXPR IdType SysGetPacketDelay::ID;
    CRL_CONSTEXPR IdType SysLidarCalibration::ID;
    CRL_CONSTEXPR IdType SysMtu::ID;
    CRL_CONSTEXPR IdType SysNetwork::ID;
    CRL_CONSTEXPR IdType SysPps::ID;
    CRL_CONSTEXPR IdType SysSensorCalibration::ID;
    CRL_CONSTEXPR IdType SysSetPtp::ID;
    CRL_CONSTEXPR IdType SysTestMtu::ID;
    CRL_CONSTEXPR IdType SysTestMtuResponse::ID;
    CRL_CONSTEXPR IdType SysTransmitDelay::ID;
    CRL_CONSTEXPR IdType SysPacketDelay::ID;
    CRL_CONSTEXPR IdType VersionRequest::ID;
    CRL_CONSTEXPR IdType VersionResponse::ID;

    CRL_CONSTEXPR VersionType Ack::VERSION;
    CRL_CONSTEXPR VersionType CamConfig::VERSION;
    CRL_CONSTEXPR VersionType CamControl::VERSION;
    CRL_CONSTEXPR VersionType CamGetConfig::VERSION;
    CRL_CONSTEXPR VersionType CamGetHistory::VERSION;
    CRL_CONSTEXPR VersionType CamHistory::VERSION;
    CRL_CONSTEXPR VersionType CamSetResolution::VERSION;
    CRL_CONSTEXPR VersionType CamSetTriggerSource::VERSION;
    CRL_CONSTEXPR VersionType CompressedImageHeader::VERSION;
    CRL_CONSTEXPR VersionType DisparityHeader::VERSION;
    CRL_CONSTEXPR VersionType ExposureConfig::VERSION;
    CRL_CONSTEXPR VersionType ImageHeader::VERSION;
    CRL_CONSTEXPR VersionType ImageMetaHeader::VERSION;
    CRL_CONSTEXPR VersionType ImuConfig::VERSION;
    CRL_CONSTEXPR VersionType ImuData::VERSION;
    CRL_CONSTEXPR VersionType ImuGetConfig::VERSION;
    CRL_CONSTEXPR VersionType ImuGetInfo::VERSION;
    CRL_CONSTEXPR VersionType ImuInfo::VERSION;
    CRL_CONSTEXPR VersionType JpegImageHeader::VERSION;
    CRL_CONSTEXPR VersionType LedGetSensorStatus::VERSION;
    CRL_CONSTEXPR VersionType LedGetStatus::VERSION;
    CRL_CONSTEXPR VersionType LedSensorStatus::VERSION;
    CRL_CONSTEXPR VersionType LedSet::VERSION;
    CRL_CONSTEXPR VersionType LedStatus::VERSION;
    CRL_CONSTEXPR VersionType LidarDataHeader::VERSION;
    CRL_CONSTEXPR VersionType LidarPollMotor::VERSION;
    CRL_CONSTEXPR VersionType LidarSetMotor::VERSION;
    CRL_CONSTEXPR VersionType MotorPoll::VERSION;
    CRL_CONSTEXPR VersionType StatusRequest::VERSION;
    CRL_CONSTEXPR VersionType StatusResponse::VERSION;
    CRL_CONSTEXPR VersionType StreamControl::VERSION;
    CRL_CONSTEXPR VersionType SysCameraCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysDeviceInfo::VERSION;
    CRL_CONSTEXPR VersionType SysDeviceModes::VERSION;
    CRL_CONSTEXPR VersionType SysDirectedStreams::VERSION;
    CRL_CONSTEXPR VersionType SysExternalCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysFlashOp::VERSION;
    CRL_CONSTEXPR VersionType SysFlashResponse::VERSION;
    CRL_CONSTEXPR VersionType SysGetCameraCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysGetDeviceInfo::VERSION;
    CRL_CONSTEXPR VersionType SysGetDeviceModes::VERSION;
    CRL_CONSTEXPR VersionType SysGetDirectedStreams::VERSION;
    CRL_CONSTEXPR VersionType SysGetExternalCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysGetLidarCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysGetMtu::VERSION;
    CRL_CONSTEXPR VersionType SysGetNetwork::VERSION;
    CRL_CONSTEXPR VersionType SysGetSensorCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysGetTransmitDelay::VERSION;
    CRL_CONSTEXPR VersionType SysGetPacketDelay::VERSION;
    CRL_CONSTEXPR VersionType SysLidarCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysMtu::VERSION;
    CRL_CONSTEXPR VersionType SysNetwork::VERSION;
    CRL_CONSTEXPR VersionType SysPps::VERSION;
    CRL_CONSTEXPR VersionType SysSensorCalibration::VERSION;
    CRL_CONSTEXPR VersionType SysSetPtp::VERSION;
    CRL_CONSTEXPR VersionType SysTestMtu::VERSION;
    CRL_CONSTEXPR VersionType SysTestMtuResponse::VERSION;
    CRL_CONSTEXPR VersionType SysTransmitDelay::VERSION;
    CRL_CONSTEXPR VersionType SysPacketDelay::VERSION;
    CRL_CONSTEXPR VersionType VersionRequest::VERSION;
    CRL_CONSTEXPR VersionType VersionResponse::VERSION;
}}}}
