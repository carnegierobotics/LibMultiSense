/**
 * @file SensorCalUtility/SensorCalUtility.cc
 *
 * Copyright 2015-2022
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
 *
 * References:
 *   CMV2000 Data Sheet (https://ams.com/documents/20143/36005/CMV2000_DS000733_3-00.pdf)
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

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <Utilities/portability/getopt/getopt.h>
#include <MultiSense/MultiSenseChannel.hh>

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
        b = byte_to_binary(static_cast<uint8_t> (x >> (currentByte*8))) + " " + b;
    }

    return b.erase(0,(currentByte*8 - bits));
}

void usage(const char *programNameP)
{
    std::cerr << "USAGE: " << programNameP << "  [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <ip_address>      : ip address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-s                   : set the calibration (default is query)" << std::endl;
    std::cerr << "\t-l                   : set the left calibration" << std::endl;
    std::cerr << "\t-r                   : set the right calibration" << std::endl;
    std::cerr << "\t-e                   : set the right black level" << std::endl;
    std::cerr << "\t-k                   : set the left black level" << std::endl;

    exit(-1);
}

} // anonymous

int main(int    argc,
         char **argvPP)
{
    Status status = Status_Ok;
    std::string ipAddress = "10.66.171.21";
    std::string leftIn = "50";
    std::string rightIn = "50";

    // The default value of the dark level offset as given in section 5.12.1 of the CMV2000 data sheet.
    std::string leftBlackIn = "-61";
    std::string rightBlackIn = "-61";
    int left;
    int right;
    int leftBlack;
    int rightBlack;
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
        std::cerr << "Please specify a value to set using -l, -r, -e, or -k" << std::endl;
        usage(*argvPP);
    }

    //
    // Initialize communications.
    std::cout << "Attempting to establish communications with: " << ipAddress << std::endl;

    Channel *channelP = Channel::Create(ipAddress);
    if (NULL == channelP) {
        std::cerr << "Failed to establish communications with: " << ipAddress << std::endl;
        return -1;
    }

    //
    // Query the current device calibration
    image::SensorCalibration sensorCalibration;

    status = channelP->getSensorCalibration(sensorCalibration);
    if (Status_Ok != status) {
        std::cerr << "failed to query sensor calibration: " << Channel::statusString(status);
        goto clean_out;
    }

    //
    // Modify the elements of the queried calibration depending on which
    // parameters will be set
    if (setCal && setLeft) {
        left = atoi(leftIn.c_str());

        if (left > 255 || left < 0)
        {
            std::cerr << "Left sensor gain range is 0-255" << std::endl;
            usage(*argvPP);
            goto clean_out;
        }

        sensorCalibration.adc_gain[0] = static_cast<uint8_t> (left);
    }

    if (setCal && setRight) {
        right = atoi(rightIn.c_str());

        if (right > 255 || right < 0)
        {
            std::cerr << "Right sensor gain range is 0-255" << std::endl;
            usage(*argvPP);
            goto clean_out;
        }

        sensorCalibration.adc_gain[1] = static_cast<uint8_t> (right);
    }

    if (setCal && setLeftBlack)
    {
        leftBlack = atoi(leftBlackIn.c_str());

        if (leftBlack > 8191 || leftBlack < -8192)
        {
            std::cerr << "Left black offset range is -8192 to 8191" << std::endl;
            usage (*argvPP);
            goto clean_out;
        }

        // Per the CMV2000 data sheet, bl_offset is a 14-bit 2-complement integer 
        sensorCalibration.bl_offset[0] = static_cast<uint16_t> (leftBlack) & 0x7FFF;
    }

    if (setCal && setRightBlack)
    {
        rightBlack = atoi(rightBlackIn.c_str());

        if (rightBlack > 8191 || rightBlack < -8192)
        {
            std::cerr << "Right black offset range is -8192 to 8191" << std::endl;
            usage (*argvPP);
            goto clean_out;
        }

        // Per the CMV2000 data sheet, bl_offset is a 14-bit 2-complement integer
        sensorCalibration.bl_offset[1] = static_cast<uint16_t> (rightBlack) & 0x7FFF;
    }


    if (false == setCal) {

        std::cout << "left sensor gain: " << static_cast<uint32_t>(sensorCalibration.adc_gain[0]) << std::endl;
        std::cout << "right sensor gain: " << static_cast<uint32_t>(sensorCalibration.adc_gain[1]) << std::endl;
        std::cout << "left sensor black level: " << static_cast<uint32_t>(sensorCalibration.bl_offset[0]) << std::endl;
        std::cout << "right sensor black level: " << static_cast<uint32_t>(sensorCalibration.bl_offset[1]) << std::endl;
        std::cout << "left sensor vramp: "  << static_cast<uint32_t>(sensorCalibration.vramp[0]) << std::endl;
        std::cout << "right sensor vramp: "  << static_cast<uint32_t>(sensorCalibration.vramp[1]) << std::endl;
    } else {
        std::cout << "Setting :" << std::endl;
        std::cout << "left sensor gain: " << static_cast<uint32_t>(sensorCalibration.adc_gain[0]) << " binary: " << byte_to_binary(sensorCalibration.adc_gain[0]) << std::endl;
        std::cout << "right sensor gain: " << static_cast<uint32_t>(sensorCalibration.adc_gain[1]) << " binary: " << byte_to_binary(sensorCalibration.adc_gain[1]) << std::endl;
        std::cout << "left sensor black level: " << static_cast<uint32_t>(sensorCalibration.bl_offset[0]) << " binary: " << bytes_to_binary(sensorCalibration.bl_offset[0], 14) << std::endl;
        std::cout << "right sensor black level: " << static_cast<uint32_t>(sensorCalibration.bl_offset[1]) << " binary: " << bytes_to_binary(sensorCalibration.bl_offset[1], 14) << std::endl;

        status = channelP->setSensorCalibration(sensorCalibration);
        if (Status_Ok != status) {
            std::cerr << "failed to set sensor calibration: " << Channel::statusString(status) << std::endl;
            goto clean_out;
        }

        std::cout <<  "Sensor calibration successfully updated" << std::endl;
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
