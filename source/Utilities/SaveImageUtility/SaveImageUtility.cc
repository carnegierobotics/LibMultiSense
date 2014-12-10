/**
 * @file SaveImageUtility/SaveImageUtility.cc
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
 *   2013-06-14, ekratzer@carnegierobotics.com, PR1044, Created file.
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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <Utilities/portability/getopt/getopt.h>

#include <LibMultiSense/details/utility/Portability.hh>
#include <LibMultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG = false;

void usage(const char *programNameP) 
{
	std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
	std::cerr << "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    
    exit(-1);
}

#ifdef WIN32
BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
	std::cerr << "Shutting down on signal: CTRL-C" << std::endl;
    doneG = true;
    return TRUE;
}
#else
void signalHandler(int sig)
{
	std::cerr << "Shutting down on signal: " << strsignal(sig) << std::endl;
    doneG = true;
}
#endif

bool savePgm(const std::string& fileName,
             uint32_t           width,
             uint32_t           height,
             uint32_t           bitsPerPixel,
             const void        *dataP)
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
                     << width << " " << height << "\n"
                     << 0xFF << "\n";
        
        outputStream.write(reinterpret_cast<const char*>(dataP), imageSize);

        break;
    }
    case 16:
    {
        outputStream << "P5\n"
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

void ppsCallback(const pps::Header& header,
                 void              *userDataP)
{
	std::cerr << "PPS: " << header.sensorTime << " ns" << std::endl;
}                

void laserCallback(const lidar::Header& header,
                   void                *userDataP)
{
// 	std::cerr << "lidar: " << header.pointCount << std::endl;
}

void imageCallback(const image::Header& header,
                   void                *userDataP)
{
    Channel *channelP = reinterpret_cast<Channel*>(userDataP);
    
//    double timeStamp = header.timeSeconds + 1e-6 * header.timeMicroSeconds;
    
    static int64_t lastFrameId = -1;

    if (-1 == lastFrameId)
        savePgm("test.pgm",
                header.width,
                header.height,
                header.bitsPerPixel,
                header.imageDataP);

    lastFrameId = header.frameId;

    image::Histogram histogram;

	if (Status_Ok != channelP->getImageHistogram(header.frameId, histogram))
		std::cerr << "failed to get histogram for frame " << header.frameId << std::endl;
}

}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    int32_t mtu = 7200;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:m:")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'm': mtu            = atoi(optarg);           break;
        default: usage(*argvPP);                           break;
        }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(currentAddress);
    if (NULL == channelP) {
		std::cerr << "Failed to establish communications with \"" << currentAddress << "\"" << std::endl;
        return -1;
    }

    //
    // Query version

    Status status;
    system::VersionInfo v;

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

    //
    // Change framerate

    {
        image::Config cfg;

        status = channelP->getImageConfig(cfg);
        if (Status_Ok != status) {
			std::cerr << "Failed to get image config: " << Channel::statusString(status) << std::endl;
            goto clean_out;
        } else {

            cfg.setResolution(1024, 544);
            cfg.setFps(30.0);
        
            status = channelP->setImageConfig(cfg);
            if (Status_Ok != status) {
				std::cerr << "Failed to configure sensor: " << Channel::statusString(status) << std::endl;
                goto clean_out;
            }
        }
    }

    //
    // Change MTU

    status = channelP->setMtu(mtu);
    if (Status_Ok != status) {
		std::cerr << "Failed to set MTU to " << mtu << ": " << Channel::statusString(status) << std::endl;
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
    // Add callbacks

    channelP->addIsolatedCallback(imageCallback, Source_All, channelP);
    channelP->addIsolatedCallback(laserCallback, channelP);
    channelP->addIsolatedCallback(ppsCallback, channelP);

    //
    // Start streaming

    status = channelP->startStreams(Source_Luma_Rectified_Left | Source_Lidar_Scan);
    if (Status_Ok != status) {
		std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    while(!doneG)
        usleep(100000);

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
		std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
