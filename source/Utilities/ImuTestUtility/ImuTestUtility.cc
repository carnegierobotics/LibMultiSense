/**
 * @file ImuTestUtility/ImuTestUtility.cc
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
 *   2013-11-12, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <iomanip>

#include <LibMultiSense/details/utility/Portability.hh>
#include <LibMultiSense/MultiSenseChannel.hh>

#include <Utilities/portability/getopt/getopt.h>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG         = false;
FILE*         logFileP      = stdout;
uint32_t      accel_samples = 0;
uint32_t      gyro_samples  = 0;
uint32_t      mag_samples   = 0;
int64_t       sequence      = -1;
uint32_t      messages      = 0;
uint32_t      dropped       = 0;

void usage(const char *programNameP) 
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <ip_address>    : IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>           : default=7200" << std::endl;
    std::cerr << "\t-f <log_file>      : FILE to log IMU data (stdout by default)" << std::endl;
    
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

void imuCallback(const imu::Header& header,
                 void              *userDataP)
{
    std::vector<imu::Sample>::const_iterator it = header.samples.begin();

    for(; it!=header.samples.end(); ++it) {

        const imu::Sample& s = *it;

        switch(s.type) {
        case imu::Sample::Type_Accelerometer: accel_samples ++; break;
        case imu::Sample::Type_Gyroscope:     gyro_samples ++;  break;
        case imu::Sample::Type_Magnetometer:  mag_samples ++;   break;
        }

        if (logFileP)
            fprintf(logFileP, "%d %.6f %.6f %.6f %.6f\n",
                    s.type, 
                    s.timeSeconds + 1e-6 * s.timeMicroSeconds,
                    s.x, s.y, s.z);
    }

    if (-1 == sequence)
        sequence = header.sequence;
    else if ((sequence + 1) != header.sequence) {
        const int32_t d = static_cast<int32_t> (header.sequence - (sequence + 1));
        dropped += d;
    }

    sequence = header.sequence;
    messages ++;
}

}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    const char *logFileNameP   = NULL;
    uint32_t    mtu            = 7200;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:f:m:v")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'f': logFileNameP   = optarg;                 break;
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
    // Query firmware version

    system::VersionInfo v;

    Status status = channelP->getVersionInfo(v);
    if (Status_Ok != status) {
        std::cerr << "Failed to query sensor version: " << Channel::statusString(status) << std::endl; 
        goto clean_out;
    }

    //
    // Make sure firmware supports IMU

    if (v.sensorFirmwareVersion <= 0x0202) {
        std::cerr << "IMU support requires sensor firmware version v2.3 or greater, sensor is " <<
                "running v" << (v.sensorFirmwareVersion >> 8) << "." << (v.sensorFirmwareVersion & 0xff) << std::endl;
        goto clean_out;
    }

    //
    // Turn off all streams by default

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl; 
        goto clean_out;
    }

    //
    // Was logging requested ?

    if (NULL != logFileNameP) {
        
        //
        // Open the log file

        logFileP = fopen(logFileNameP, "w+");
        if (NULL == logFileP) {
            std::cerr << "Failed to open \"" << logFileNameP << "\" for writing: " << strerror(errno) << std::endl;
            goto clean_out;
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
    // Add callbacks

    channelP->addIsolatedCallback(imuCallback);

    //
    // Start streaming

    status = channelP->startStreams(Source_Imu);
    if (Status_Ok != status) {
        std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl; 
        goto clean_out;
    }

    while(!doneG)
        usleep(100000);
        
    //
    // Stop streaming

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
    }

    //
    // Report simple stats

    {
        int64_t imu_total = accel_samples + gyro_samples + mag_samples;
        if (imu_total > 0) {
            std::cerr << "IMU samples : " <<
                         "total: " << imu_total << ", " <<
                         "accel: " << std::fixed << std::setprecision(1) << (100.0 * static_cast<double>(accel_samples) / static_cast<double>(imu_total)) << "%, " <<
                         "gyro: " << std::fixed << std::setprecision(1) << (100.0 * static_cast<double>(gyro_samples) / static_cast<double>(imu_total)) << "%, " <<
                         "mag: " << std::fixed << std::setprecision(1) << (100.0 * static_cast<double>(mag_samples) / static_cast<double>(imu_total)) << "%" << std::endl;
        }
        
        if (messages > 0) 
            std::cerr << "IMU messages: total: " << messages << ", " <<
                         "dropped: " << dropped << "(" << std::fixed << std::setprecision(6) << (100* static_cast<double>(dropped) / static_cast<double>(messages+dropped)) << "%)" << std::endl;
    }

clean_out:
        
    if (logFileNameP)
        fclose(logFileP);

    Channel::Destroy(channelP);
    return 0;
}
