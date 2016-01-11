/**
 * @file SensorCalUtility/SensorCalUtility.cc
 *
 * Copyright 2015
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
 *   2015-09-29, kcarpenter@carnegierobotics.com, Created file.
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>
#include <map>
#include <string.h>
#include <sstream>

#include <Utilities/portability/getopt/getopt.h>
#include <LibMultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

std::string byte_to_binary(uint8_t x)
{
    std::stringstream b;

    uint8_t z;
    for (z = 128; z > 0; z >>= 1)
    {
       b << (((x & z) == z) ? "1" : "0");
    }

    return b.str();
}

std::string bytes_to_binary(uint64_t x, int bits)
{
    std::string b;
    uint8_t currentByte;
    for(currentByte = 0; currentByte*8<bits; currentByte++)
    {
        b = byte_to_binary((x >> currentByte*8)) + " " + b;
    }

    return b.erase(0,(currentByte*8 - bits));
}

void usage(const char *programNameP) 
{
    fprintf(stderr, 
            "USAGE: %s [<options>]\n", 
            programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>      : ip address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-s                   : set the calibration (default is query)\n");
    fprintf(stderr, "\t-l                   : set the left calibration \n");
    fprintf(stderr, "\t-r                   : set the right calibration \n");
    fprintf(stderr, "\t-e                   : set the right black level \n");
    fprintf(stderr, "\t-k                   : set the left black level \n");
    
    exit(-1);
}

bool fileExists(const std::string& name)
{
    struct stat sbuf;
    return (0 == stat(name.c_str(), &sbuf));
}

}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    Status status = Status_Ok;
    std::string ipAddress = "10.66.171.21";
    std::string leftIn = "50";
    std::string rightIn = "50";
    std::string leftBlackIn = "-61";
    std::string rightBlackIn = "-61";
	uint8_t left;
	uint8_t right;
    int16_t leftBlack;
    int16_t rightBlack;
    bool        setCal=false;
	bool        setLeft=false;
	bool        setRight=false;
	bool        setLeftBlack=false;
	bool        setRightBlack=false;

    //
    // Parse args

    int inArgument;

    while(-1 != (inArgument = getopt(argc, argvPP, "l:r:k:e:a:sy")))
        switch(inArgument) {
        case 'a': ipAddress      = std::string(optarg);                      break;
        case 'l': leftIn         = std::string(optarg); setLeft=true;        break;
        case 'r': rightIn        = std::string(optarg); setRight=true;       break;
        case 'k': leftBlackIn    = std::string(optarg); setLeftBlack=true;    break;
        case 'e': rightBlackIn   = std::string(optarg); setRightBlack=true;   break;
        case 's': setCal         = true;                   break;
        default: usage(*argvPP);                           break;
    }

    //
    // Verify options

    if (setCal && (!setLeft && !setRight && !setLeftBlack && !setRightBlack)) {
        fprintf(stderr, "Please specify a value to set using -l, -r, -e, or -k\n");
        usage(*argvPP);
    }

    //
    // Initialize communications.
    fprintf(stdout, "Attempting to establish communications with \"%s\"\n",
		ipAddress.c_str());

    Channel *channelP = Channel::Create(ipAddress);
    if (NULL == channelP) {
        fprintf(stderr, "Failed to establish communications with \"%s\"\n",
            ipAddress.c_str());
        return -1;
    }

    //
    // Query the current device calibration
    image::SensorCalibration sensorCalibration;

    status = channelP->getSensorCalibration(sensorCalibration);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query sensor calibration: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }

    //
    // Modify the elements of the queried calibration depending on which
    // parameters will be set
    if (setCal && setLeft) {
        left = atoi(leftIn.c_str());

        if (left > 255 || left < 0)
        {
            fprintf(stderr, "Left sensor gain range is 0-255\n");
            usage(*argvPP);
            goto clean_out;
        }

        sensorCalibration.adc_gain[0] = left;
    }

    if (setCal && setRight) {
        right = atoi(rightIn.c_str());

        if (right > 255 || right < 0)
        {
            fprintf(stderr, "Right sensor gain range is 0-255\n");
            usage(*argvPP);
            goto clean_out;
        }

        sensorCalibration.adc_gain[1] = right;
    }

    if (setCal && setLeftBlack)
    {
        leftBlack = atoi(leftBlackIn.c_str());
        sensorCalibration.bl_offset[0] = leftBlack;
    }

    if (setCal && setRightBlack)
    {
        rightBlack = atoi(rightBlackIn.c_str());
        sensorCalibration.bl_offset[1] = rightBlack;
    }


    if (false == setCal) {

        fprintf(stdout,"left sensor gain: %hu\nright sensor gain %hu\nleft sensor black level: %d\nright sensor black level %d\n", 
                    sensorCalibration.adc_gain[0], sensorCalibration.adc_gain[1],
                    sensorCalibration.bl_offset[0], sensorCalibration.bl_offset[1]);

    } else {

        fprintf(stdout,"Setting :\nleft sensor gain: %5hu binary: %s\n"
                                   "right sensor gain %5hu binary: %s\n"
                                   "left sensor black level: %d binary: %s\n"
                                   "right sensor black level %d binary: %s\n", 
                    sensorCalibration.adc_gain[0],byte_to_binary(sensorCalibration.adc_gain[0]).c_str(),
                    sensorCalibration.adc_gain[1],byte_to_binary(sensorCalibration.adc_gain[1]).c_str(),
                    sensorCalibration.bl_offset[0],bytes_to_binary(sensorCalibration.bl_offset[0], 14).c_str(),
                    sensorCalibration.bl_offset[1],bytes_to_binary(sensorCalibration.bl_offset[1], 14).c_str());

        status = channelP->setSensorCalibration(sensorCalibration);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to set sensor calibration: %s\n", 
                    Channel::statusString(status));
            goto clean_out;
        }

        fprintf(stdout, "Sensor calibration successfully updated\n");
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
