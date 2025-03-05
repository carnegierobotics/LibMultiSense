 /**
 * @file FeatureDetectorUtility/FeatureDetectorUtility.cc
 *
 * Copyright 2024-2025
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
 *   2024-25-01, patrick.smith@carnegierobotics.com, IRAD, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h> // htons
#endif

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <string>
#include <map>
#include <chrono>
#include <ctime>

#include <getopt/getopt.h>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

#include "FeatureDetectorUtilities.hh"

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG = false;

volatile int affineCalib = 0;
volatile uint16_t affineCalCount0 = 0;
volatile bool affineCalCount0Set = false;

struct featureDetectionTime
{
  std::chrono::time_point<std::chrono::system_clock> imageTime;
  std::chrono::time_point<std::chrono::system_clock> featureTime;
};

struct UserData
{
    Channel *channelP;
    system::VersionInfo version;
    system::DeviceInfo deviceInfo;
    image::Calibration calibration;
    std::map<int64_t, featureDetectionTime> elapsedTime;
    uint16_t observerStatus;
    uint16_t observerNum;
    uint16_t observerIndex;
    int16_t observerDy;
    int16_t observerTheta;
    uint16_t affineCalCount;
};

void usage(const char *programNameP)
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>                : MTU to set the camera to (default=1500)" << std::endl;
    std::cerr << "\t-r <head_id>            : remote head ID (default=0)" << std::endl;
    std::cerr << "\t-c                      : Perform affine calibration (from scratch, discard previous affine calibration)" << std::endl;
    std::cerr << "\t-i                      : Perform affine calibration (incremental, keep previous affine calibration)" << std::endl;
    exit(1);
}

#ifdef WIN32
BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
    CRL_UNUSED (dwCtrlType);
    doneG = true;
    return TRUE;
}
#else
void signalHandler(int sig)
{
    (void) sig;
    doneG = true;
}
#endif

RemoteHeadChannel getRemoteHeadIdFromString(const std::string &head_str)
{
  if (head_str == "VPB")
  {
      return Remote_Head_VPB;
  }
  else if (head_str == "0")
  {
      return Remote_Head_0;
  }
  else if (head_str == "1")
  {
      return Remote_Head_1;
  }
  else if (head_str == "2")
  {
      return Remote_Head_2;
  }
  else if (head_str == "3")
  {
      return Remote_Head_3;
  }

  fprintf(stderr, "Error: Unrecognized remote head\n");
  fprintf(stderr, "Please use one of the following:\n");
  fprintf(stderr, "\tVPB\n");
  fprintf(stderr, "\t0'\n");
  fprintf(stderr, "\t1\n");
  fprintf(stderr, "\t2\n");
  fprintf(stderr, "\t3\n");
  exit(EXIT_FAILURE);
}

system::DeviceMode getOperatingMode(const std::vector<system::DeviceMode> &modes)
{
    //
    // Get the full resolution image

    system::DeviceMode best_mode(0, 0, 0, -1);
    for (size_t i = 0 ; i < modes.size() ; ++i)
    {
        const system::DeviceMode &mode = modes[i];
        if (mode.width >= best_mode.width && mode.height >= best_mode.height && mode.disparities >= best_mode.disparities) {
            best_mode = mode;
        }
    }

    //
    // Target 1/4 of the full res setting. Check to see if there is a setting with a larger number of disparities

    system::DeviceMode target_mode(best_mode.width / 2, best_mode.height / 2, 0, best_mode.disparities);

    for (size_t i = 0 ; i < modes.size() ; ++i)
    {
        const system::DeviceMode &mode = modes[i];
        if (mode.width == target_mode.width && mode.height == target_mode.height && mode.disparities >= target_mode.disparities) {
            target_mode = mode;
        }
    }

    //
    // The camera does not support our targeted 1/4 res setting

    if (target_mode.supportedDataSources == 0) {
        return best_mode;
    }

    return target_mode;
}


std::string writeMatrix(const float* data, size_t width, size_t height)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);

    for (size_t h = 0 ; h < height ; ++h) {
        for (size_t w = 0 ; w < width ; ++w) {
            ss << data[w + width * h] << ",";
        }
    }

    return ss.str();
}

std::string assembledInfoString(const image::Header&       header,
                                const system::DeviceInfo&  info,
                                const system::VersionInfo& version,
                                const image::Calibration&  calibration)
{
    std::stringstream ss;

    bool hasAuxCamera = info.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 ||
                        info.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 ||
                        info.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM ||
                        info.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21i;

    ss << "SN," << info.serialNumber << ",";
    ss << "HWRev," << info.hardwareRevision << ",";
    ss << "APIVersion," << version.apiVersion << ",";
    ss << "FirmwareVersion," << version.sensorFirmwareVersion << ",";
    ss << "timeSec," << header.timeSeconds << ",";
    ss << "timeMiroSec," << header.timeMicroSeconds << ",";
    ss << "exposure," << header.exposure << ",";
    ss << "gain," << header.gain << ",";
    ss << "fps," << header.framesPerSecond << ",";
    ss << "leftM," << writeMatrix(&calibration.left.M[0][0], 3, 3);
    ss << "leftD," << writeMatrix(calibration.left.D, 8, 1);
    ss << "leftR," << writeMatrix(&calibration.left.R[0][0], 3, 3);
    ss << "leftP," << writeMatrix(&calibration.left.P[0][0], 4, 3);
    ss << "rightM," << writeMatrix(&calibration.right.M[0][0], 3, 3);
    ss << "rightD," << writeMatrix(calibration.right.D, 8, 1);
    ss << "rightR," << writeMatrix(&calibration.right.R[0][0], 3, 3);
    ss << "rightP," << writeMatrix(&calibration.right.P[0][0], 4, 3);
    if (hasAuxCamera) {
        ss << "auxM," << writeMatrix(&calibration.aux.M[0][0], 3, 3);
        ss << "auxD," << writeMatrix(calibration.aux.D, 8, 1);
        ss << "auxR," << writeMatrix(&calibration.aux.R[0][0], 3, 3);
        ss << "auxP," << writeMatrix(&calibration.aux.P[0][0], 4, 3);
    }

    return ss.str();
}

bool savePgm(const std::string& fileName,
             uint32_t           width,
             uint32_t           height,
             uint32_t           bitsPerPixel,
             const std::string& comment,
             const void         *dataP)
{
    std::ofstream outputStream(fileName.c_str(), std::ios::binary | std::ios::out);

    if (false == outputStream.good()) {
		std::cerr << "Failed to open \"" << fileName << "\"" << std::endl;
        return false;
    }

    const uint32_t imageSize = height * width;

    switch(bitsPerPixel) {
    case 8:
    {

        outputStream << "P5\n"
                     << "#" << comment << "\n"
                     << width << " " << height << "\n"
                     << 0xFF << "\n";

        outputStream.write(reinterpret_cast<const char*>(dataP), imageSize);

        break;
    }
    case 16:
    {
        outputStream << "P5\n"
                     << "#" << comment << "\n"
                     << width << " " << height << "\n"
                     << 0xFFFF << "\n";

        const uint16_t *imageP = reinterpret_cast<const uint16_t*>(dataP);

        for (uint32_t i=0; i<imageSize; ++i) {
            uint16_t o = htons(imageP[i]);
            outputStream.write(reinterpret_cast<const char*>(&o), sizeof(uint16_t));
        }

        break;
    }
    }

    outputStream.close();
    return true;
}

bool savePgm(const std::string&         fileName,
             const image::Header&       header,
             const system::DeviceInfo&  info,
             const system::VersionInfo& version,
             const image::Calibration&  calibration)
{
    const std::string comment = assembledInfoString(header, info, version, calibration);

    return savePgm(fileName,
                   header.width,
                   header.height,
                   header.bitsPerPixel,
                   comment,
                   header.imageDataP);
}

void imageCallback(const image::Header& header,
                   void                *userDataP)
{
    UserData *userData = reinterpret_cast<UserData*>(userDataP);

    static int64_t lastFrameId = -1;

    if (-1 == lastFrameId)
    {
        savePgm("test.pgm",
                header,
                userData->deviceInfo,
                userData->version,
                userData->calibration);
    }

    if (header.source == Source_Luma_Left) {

        auto it = userData->elapsedTime.find(header.frameId);
        if (it == userData->elapsedTime.end()) {
          featureDetectionTime t;
          t.imageTime = std::chrono::system_clock::now();
          userData->elapsedTime.insert(std::pair<int64_t, featureDetectionTime>(header.frameId, t));
        }
        else
        {
          it->second.imageTime = std::chrono::system_clock::now();
          std::cout <<" Image received after feature: " << header.frameId
                    <<" Delta: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(it->second.featureTime - it->second.imageTime).count()
                    << "ms\n";
          userData->elapsedTime.erase(it);
        }

    }
    lastFrameId = header.frameId;

    image::Histogram histogram;

    if (Status_Ok != userData->channelP->getImageHistogram(header.frameId, histogram))
        std::cerr << "failed to get histogram for frame " << header.frameId << std::endl;
}


const char *observerStatus(uint16_t x)
{
    switch(x)
    {
    case    0: return "IDLE";
    case    10: return "NOT ENOUGH MATCH";
    case    11: return "SVD FAILED";
    case    12: return "BAD DISTRIBUTION";
    case    13: return "IDLE 0";
    case    14: return "IDLE 1";
    case    15: return "IDLE 01";
    case    0xffff: return "IN PROGRESS";
    }
    return "unknown";
}

void featureDetectorCallback(const secondary_app::Header& header,
                   void                *userDataP)
{
    feature_detector::Header fHeader;
    UserData *userData = reinterpret_cast<UserData*>(userDataP);
    void * buffer = userData->channelP->reserveCallbackBuffer();
    Status s = feature_detector::secondaryAppDataExtract(fHeader, header);

    if (s != Status_Ok)
    {
      fprintf(stderr, "%s Error Secondary App Data extraction failed\n", __func__ );
      userData->channelP->releaseCallbackBuffer(buffer);
      return;
    }

    
    userData->channelP->releaseCallbackBuffer(buffer);

    if(affineCalib)
    {
        if(affineCalCount0Set)
        {
            std::cout << "\033[H";
            std::cout << "Frame:  " << fHeader.frameId << " \n";
            std::cout << "Motion X: " << fHeader.averageXMotion << " \n";
            std::cout << "Motion Y: " << fHeader.averageYMotion << " \n";
            std::cout << "Number of features:     " << fHeader.numFeatures    << " \n";
            std::cout << "Number of descriptors:  " << fHeader.numDescriptors << " \n";
            std::cout << "Octave Width:    " << fHeader.octaveWidth << " \n";
            std::cout << "Octave Height:   " << fHeader.octaveHeight << " \n";
            std::cout << "timeSeconds:     " << fHeader.timeSeconds << " \n";
            std::cout << "timeNanoSeconds: " << fHeader.timeNanoSeconds << " \n";
            std::cout << "ptpNanoSeconds:  " << fHeader.ptpNanoSeconds << " \n";

            if(fHeader.observerStatus==0xffff)
            {
                std::cout << "observerStatus:  " << observerStatus(fHeader.observerStatus) << "                     \n";
                std::cout << "observerIndex:   " << fHeader.observerIndex << "        \n";
                std::cout << "observerNum:     " << fHeader.observerNum << "        \n";
                std::cout << "observerDy:      " << fHeader.observerDy * 0.001 << "        \n";
                std::cout << "observerTheta:   " << fHeader.observerTheta * 0.001 << "        \n";
                std::cout << "affineCalCount:  " << fHeader.affineCalCount << "        \n";
                userData->observerStatus = fHeader.observerStatus;
                userData->observerIndex = fHeader.observerIndex;
                userData->observerNum = fHeader.observerNum;
                userData->observerDy = fHeader.observerDy;
                userData->observerTheta = fHeader.observerTheta;
                userData->affineCalCount = fHeader.affineCalCount;
            }
            else if(fHeader.observerStatus==0)
            {
                std::cout << "observerStatus:  IDLE             ";
            }
            else if(fHeader.observerStatus==9)
            {
                std::cout << "observerStatus:  INITIALIZING " << fHeader.observerNum << "   \n";
            }
            else
            {
                if(userData->observerStatus==0xffff)
                {
                    std::cout << "observerStatus:  " << observerStatus(userData->observerStatus) << " (" << observerStatus(fHeader.observerStatus) << ")                     \n";
                    std::cout << "observerIndex:   " << userData->observerIndex <<         "        \n";
                    std::cout << "observerNum:     " << userData->observerNum <<           "        \n";
                    std::cout << "observerDy:      " << userData->observerDy * 0.001 <<    "        \n";
                    std::cout << "observerTheta:   " << userData->observerTheta * 0.001 << "        \n";
                    std::cout << "affineCalCount:  " << userData->affineCalCount << "\n";
                }
            }
            
            if(affineCalCount0 != fHeader.affineCalCount)
            {
                doneG = true;
                std::cout << std::endl << "affine calibration is finished" << std::endl << std::endl;
            }
        }
        else
        {
            affineCalCount0 = fHeader.affineCalCount;
            affineCalCount0Set = true;
            std::cout << "\033[2J\033[H";
        }
    }
    else
    {
        if ((fHeader.source == feature_detector::Source_Feature_Left)
        || (fHeader.source == feature_detector::Source_Feature_Rectified_Left))
         {
            auto it = userData->elapsedTime.find(fHeader.frameId);
            if (it == userData->elapsedTime.end()) {
                std::cout << "Unexpected result, image not yet received for frame: " << fHeader.frameId << "\n";
                featureDetectionTime t;
                t.featureTime = std::chrono::system_clock::now();
                userData->elapsedTime.insert(std::pair<int64_t, featureDetectionTime>(fHeader.frameId, t));
            }
            else
            {
                it->second.featureTime = std::chrono::system_clock::now();
                std::cout << "Feature received after image " << fHeader.frameId
                << " Delta: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(it->second.featureTime - it->second.imageTime).count()
                << "ms\n";
                userData->elapsedTime.erase(it);
            }
        }
        
        std::cout << "Source: " << fHeader.source << " \n";
        std::cout << "Frame:  " << fHeader.frameId << " \n";
        std::cout << "Motion X: " << fHeader.averageXMotion << " \n";
        std::cout << "Motion Y: " << fHeader.averageYMotion << " \n";
        std::cout << "Number of features:     " << fHeader.numFeatures    << " \n";
        std::cout << "Number of descriptors:  " << fHeader.numDescriptors << " \n";
        std::cout << "Octave Width:    " << fHeader.octaveWidth << " \n";
        std::cout << "Octave Height:   " << fHeader.octaveHeight << " \n";
        std::cout << "timeSeconds:     " << fHeader.timeSeconds << " \n";
        std::cout << "timeNanoSeconds: " << fHeader.timeNanoSeconds << " \n";
        std::cout << "ptpNanoSeconds:  " << fHeader.ptpNanoSeconds << " \n";

        std::cout << "observerStatus:  " << fHeader.observerStatus << " \n";
        std::cout << "observerIndex:   " << fHeader.observerIndex << " \n";
        std::cout << "observerNum:     " << fHeader.observerNum << " \n";
        std::cout << "observerDy:      " << fHeader.observerDy * 0.001 << " \n";
        std::cout << "observerTheta:   " << fHeader.observerTheta * 0.001 << " \n";
        std::cout << "affineCalCount:  " << fHeader.affineCalCount << "\n";
    }
}

} // anonymous

int main(int    argc,
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    int32_t mtu = 0;
    RemoteHeadChannel head_id = Remote_Head_VPB;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int opt;

    while(-1 != (opt = getopt(argc, argvPP, "a:m:r:ci")))
        switch(opt) {
        case 'a': currentAddress = std::string(optarg);               break;
        case 'm': mtu            = atoi(optarg);                      break;
        case 'r': head_id        = getRemoteHeadIdFromString(optarg); break;
        case 'c': affineCalib    = 1;                                 break;
        case 'i': affineCalib    = 2;                                 break;
        default: usage(*argvPP);                                      break;
        }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(currentAddress, head_id);
    if (NULL == channelP) {
		std::cerr << "Failed to establish communications with \"" << currentAddress << "\"" << std::endl;
        return -1;
    }

    //
    // Query version

    Status status;
    system::VersionInfo v;
    VersionType version;
    std::vector<system::DeviceMode> deviceModes;
    system::DeviceMode operatingMode;
    image::Calibration calibration;
    system::DeviceInfo info;
    UserData userData;
    bool quarter_res = false;

    userData.observerStatus=0;

    status = channelP->getSensorVersion(version);
    status = channelP->getVersionInfo(v);

    if (Status_Ok != status) {
		std::cerr << "Failed to query sensor version: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    std::cout << "API build date      :  " << v.apiBuildDate << "\n";
    std::cout << "API version         :  0x" << std::hex << std::setw(4) << std::setfill('0') << v.apiVersion << "\n";
    std::cout << "Firmware build date :  " << v.sensorFirmwareBuildDate << "\n";
    std::cout << "Firmware version    :  0x" << std::hex << std::setw(4) << std::setfill('0') << v.sensorFirmwareVersion << "\n";
    std::cout << "Hardware version    :  0x" << std::hex << v.sensorHardwareVersion << "\n";
    std::cout << "Hardware magic      :  0x" << std::hex << v.sensorHardwareMagic << "\n";
    std::cout << "FPGA DNA            :  0x" << std::hex << v.sensorFpgaDna << "\n";
    std::cout << std::dec;

    status = channelP->getDeviceModes(deviceModes);
    if (Status_Ok != status) {
        std::cerr << "Failed to get device modes: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    operatingMode = getOperatingMode(deviceModes);

    //
    // Change framerate

    {
        image::Config cfg;

        status = channelP->getImageConfig(cfg);
        if (Status_Ok != status) {
            std::cerr << "Failed to get image config: " << Channel::statusString(status) << std::endl;
            goto clean_out;
        } else {
            if(affineCalib)
            {
                operatingMode.width = 1920;
                operatingMode.height = 1200;
                operatingMode.disparities = 256;
                std::cout << "Affine calibration uses full-res" << std::endl;    
            }

            std::cout << "Setting resolution to: " << operatingMode.width << "x" <<
                                                      operatingMode.height << "x" <<
                                                      operatingMode.disparities << std::endl;

            cfg.setResolution(operatingMode.width, operatingMode.height);
            cfg.setDisparities(operatingMode.disparities);
            if (operatingMode.width == 960) {
                quarter_res = true;
                cfg.setFps(15.0);
            }
            else
            {
                cfg.setFps(5.0);
            }

            status = channelP->setImageConfig(cfg);
            if (Status_Ok != status) {
                std::cerr << "Failed to configure sensor: " << Channel::statusString(status) << std::endl;
                goto clean_out;
            }
        }
    }

    {

        system::SecondaryAppRegisteredApps s;
        status = channelP->getRegisteredApps(s);
        if (Status_Ok != status)
        {
          std::cerr << "Error failed to get registered apps\n";
          return -2;
        }

        status = channelP->secondaryAppActivate(s.apps[0].appName);
        if (Status_Ok != status)
        {
          std::cerr << "Error failed to activate app " << s.apps[0].appName;
          return -2;
        }

        fprintf(stderr, "%s got registered app: %s activated\n", __func__, s.apps[0].appName.c_str() );

        feature_detector::FeatureDetectorConfig fcfg;
        
        if (quarter_res)
            fcfg.setNumberOfFeatures(1500);
        else
            fcfg.setNumberOfFeatures(5000);
        fcfg.setGrouping(true);
        fcfg.setMotion(1);

        uint32_t fd_opts = 0;   // USE HLS
        if(affineCalib)
        {
            fd_opts |= FeatureDetectorConfigParams::OPT_USE_OBSERVER | FeatureDetectorConfigParams::OPT_AUTO_AFFINE_CAL;
            if(affineCalib>1)
            {
                fd_opts |= FeatureDetectorConfigParams::OPT_OBSERVER_INCREMENTAL;
            }
        }
        fcfg.setOptions(fd_opts);

        status = channelP->setSecondaryAppConfig(fcfg);
        if (Status_Ok != status)
        {
          std::cerr << "Error failed to set featureDetectorConfig apps\n";
          return -2;
        }

        std::cout << "Successfully Configured Feature Detector\n";


        status = channelP->getSecondaryAppConfig(fcfg);
        if (Status_Ok != status) {
          std::cerr << "Failed to get feature detector config: " << Channel::statusString(status) << std::endl;
                goto clean_out;
        }

        std::cout << "Current feature detector settings: "
          << fcfg.numberOfFeatures() << " : "
          << fcfg.grouping() << " : "
          << fcfg.motion() << " : " 
          << fcfg.options() << "\n";        
    }

    //
    // Change MTU

    if (mtu >= 1500)
        status = channelP->setMtu(mtu);
    else
        status = channelP->setBestMtu();
    if (Status_Ok != status) {
		std::cerr << "Failed to set MTU: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Change trigger source

    status = channelP->setTriggerSource(Trigger_Internal);
    if (Status_Ok != status) {
		std::cerr << "Failed to set trigger source: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Get image Calibration

    status = channelP->getImageCalibration(calibration);
    if (Status_Ok != status) {
		std::cerr << "Failed to get image calibration: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    //
    // Get device info

    status = channelP->getDeviceInfo(info);
    if (Status_Ok != status) {
		std::cerr << "Failed to get device info: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    userData.channelP = channelP;
    userData.version = v;
    userData.deviceInfo = info;
    userData.calibration = calibration;

    //
    // Add callbacks

    if(!affineCalib)
    {
        channelP->addIsolatedCallback(imageCallback, Source_All, &userData);
    }
    channelP->addIsolatedCallback(featureDetectorCallback, &userData);

    //
    // Start streaming
    

    if(!affineCalib)
    {
        status = channelP->startStreams((operatingMode.supportedDataSources & Source_Luma_Left)  |
                                        (operatingMode.supportedDataSources & Source_Luma_Right) |
                                        feature_detector::Source_Feature_Left|feature_detector::Source_Feature_Right);
    }
    else
    {
        status = channelP->startStreams(feature_detector::Source_Feature_Left|feature_detector::Source_Feature_Right);
    }


    if (Status_Ok != status) {
		std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    while(!doneG)
    {
        system::StatusMessage statusMessage;
        status = channelP->getDeviceStatus(statusMessage);

        // if (Status_Ok == status) {
        //     std::cout << "Uptime: " << statusMessage.uptime << ", " <<
        //     "SystemOk: " << statusMessage.systemOk << ", " <<
        //     "FPGA Temp: " << statusMessage.fpgaTemperature << ", " <<
        //     "Left Imager Temp: " << statusMessage.leftImagerTemperature << ", " <<
        //     "Right Imager Temp: " << statusMessage.rightImagerTemperature << ", " <<
        //     "Input Voltage: " << statusMessage.inputVoltage << ", " <<
        //     "Input Current: " << statusMessage.inputCurrent << ", " <<
        //     "FPGA Power: " << statusMessage.fpgaPower << ", " <<
        //     "Logic Power: " << statusMessage.logicPower << ", " <<
        //     "Imager Power: " << statusMessage.imagerPower << std::endl;
        // }

        usleep(1000000);
    }

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status)
    {
		std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
