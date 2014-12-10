/**
 * @file ImuConfigUtility/ImuConfigUtility.cc
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
#include <string.h>
#include <errno.h>
#include <iostream>
#include <iomanip>

#include <LibMultiSense/MultiSenseChannel.hh>

#include <Utilities/portability/getopt/getopt.h>

#ifdef WIN32
#define strcasecmp _stricmp
#endif

using namespace crl::multisense;

namespace {  // anonymous

std::vector<imu::Info>   sensor_infos;
std::vector<imu::Config> sensor_configs;
uint32_t                 sensor_samplesPerMessage    = 0;
uint32_t                 sensor_maxSamplesPerMessage = 0;

void usage(const char *programNameP) 
{
	std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
	std::cerr << "Where <options> are:" << std::endl;
	std::cerr << "\t-a <ip_address>        : IPV4 address (default=10.66.171.21)" << std::endl;
	std::cerr << "\t-q                     : query and report IMU configuration" << std::endl;
	std::cerr << "\t-f                     : store IMU configuration in non-volatile flash" << std::endl;
	std::cerr << "\t-s <samples>           : set IMU samples-per-message" << std::endl;
	std::cerr << "\t-c \"<sensor_config>\"   : IMU sensor configuration string\n" << std::endl;
	std::cerr << "And \"<sensor_config>\" is of the following form:" << std::endl;
	std::cerr << "\t\"<sensor_name> <enabled> <rate_table_index> <range_table_index>\"\n" << std::endl;
	std::cerr << "For example, to enable the accelerometer, and have it use rate index 1 and range index 2:\n" << std::endl;
	std::cerr << "\t-c \"accelerometer true 1 2\"\n" << std::endl;
	std::cerr << "Multiple \"-c\" options may be specified to configure more than 1 sensor\n" << std::endl;
	std::cerr << "Please note that small values for samples-per-message combined with high IMU sensor rates" << std::endl;
	std::cerr << "may interfere with the acquisition and transmission of image and lidar data, if applicable" << std::endl;

    exit(-1);
}

bool imuInfoByName(const std::string& name, imu::Info& info)
{
    std::vector<imu::Info>::const_iterator it = sensor_infos.begin();

    for(; it != sensor_infos.end(); ++it) 
        if (name == (*it).name) {
            info = (*it);
            return true;
        }

    return false;
}

}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    std::string              currentAddress         = "10.66.171.21";
    int32_t                  user_samplesPerMessage = 0;
    bool                     query                  = false;
    bool                     storeInFlash           = false;
    std::vector<std::string> cli_configs;
    Status                   status;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:s:c:qf")))
        switch(c) {
        case 'a': currentAddress         = std::string(optarg);    break;
        case 's': user_samplesPerMessage = atoi(optarg);           break;
        case 'q': query                  = true;                   break;
        case 'f': storeInFlash           = true;                   break;
        case 'c': cli_configs.push_back(optarg);                   break;
        default: usage(*argvPP);                                   break;
        }

    //
    // A setting of zero here tells the firmware to leave it alone, useful
    // if you do not want to change the default.

    if (user_samplesPerMessage < 0) {
		std::cerr << "Invalid samples-per-message: " << user_samplesPerMessage << std::endl;
        exit(-2);
    }

    //
    // If no setting changes are requested, perform a query

    if (0 == cli_configs.size() && 0 == user_samplesPerMessage)
        query = true;

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

    status = channelP->getVersionInfo(v);
    if (Status_Ok != status) {
		std::cerr << "Failed to query sensor version: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    if (query) {
		std::cout << "Version information:\n";
		std::cout << "\tAPI build date      :  " << v.apiBuildDate << "\n";
        std::cout << "\tAPI version         :  0x" << std::hex << std::setw(4) << std::setfill('0') << v.apiVersion << "\n";
		std::cout << "\tFirmware build date :  " << v.sensorFirmwareBuildDate << "\n";
		std::cout << "\tFirmware version    :  0x" << std::hex << std::setw(4) << std::setfill('0') << v.sensorFirmwareVersion << "\n";
		std::cout << "\tHardware version    :  0x" << std::hex << v.sensorHardwareVersion << "\n";
		std::cout << "\tHardware magic      :  0x" << std::hex << v.sensorHardwareMagic << "\n";
		std::cout << "\tFPGA DNA            :  0x" << std::hex << v.sensorFpgaDna << "\n";
		std::cout << std::dec;
    }

    //
    // Make sure firmware supports IMU

    if (v.sensorFirmwareVersion <= 0x0202) {
		std::cerr << "IMU support requires sensor firmware version v2.3 or greater, sensor is " <<
			"running v" << (v.sensorFirmwareVersion >> 8) << "." << (v.sensorFirmwareVersion & 0xff) << std::endl;
            goto clean_out;
    }

    //
    // Query IMU info / configuration

    status = channelP->getImuInfo(sensor_maxSamplesPerMessage,
                                  sensor_infos);
    if (Status_Ok != status) {
		std::cerr << "Failed to query imu info: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }
    status = channelP->getImuConfig(sensor_samplesPerMessage, sensor_configs);
    if (Status_Ok != status) {
		std::cerr << "Failed to query imu config: " << Channel::statusString(status) << std::endl;
        goto clean_out;
    }

    if (query) {
		std::cout << "\nMax IMU samples-per-message: " << sensor_maxSamplesPerMessage << "\n";
        std::cout << sensor_infos.size () << " IMU sensors:\n";
        for(uint32_t i=0; i<sensor_infos.size(); i++) {

        const imu::Info& m = sensor_infos[i];

		std::cout << "\t" << m.name << ":\n";
		std::cout << "\t\tdevice:   " << m.device << "\n";
		std::cout << "\t\tunits :   " << m.units << "\n";
		std::cout << "\t\trates :   " << m.rates.size() << ": rate (Hz), bandwidthCutoff (Hz)\n";
		for (uint32_t j = 0; j < m.rates.size(); j++)
			std::cout << "\t\t\t\t" << j << ": " << std::fixed << std::setprecision(1) << m.rates[j].sampleRate << "," <<
                                 			      std::fixed << std::setprecision(3) << m.rates[j].bandwidthCutoff << "\n";
		std::cout << "\t\tranges:   " << m.ranges.size() << ": range (+/- " << m.units << "), resolution (" << m.units << ")\n";
		for (uint32_t j = 0; j < m.ranges.size(); j++)
			std::cout << "\t\t\t\t" << j << ": " << std::fixed << std::setprecision(1) << m.ranges[j].range << ", " <<
  			                                        std::fixed << std::setprecision(6) << m.ranges[j].resolution << "\n";
        }

        std::cout << "\nCurrent IMU configuration:\n";
		std::cout << "\t-s " << sensor_samplesPerMessage << " ";
        for(uint32_t i=0; i<sensor_configs.size(); i++) {
            
            const imu::Config& c = sensor_configs[i];
            
			std::cout << "-c \"" << c.name << " " << (c.enabled ? "true" : "false") << " " <<
				c.rateTableIndex << " " << c.rangeTableIndex << "\" ";
        }
        std::cout << "\n";
    }

    //
    // Send current configuration

    if (user_samplesPerMessage > 0 || cli_configs.size() > 0) {

        std::vector<imu::Config> user_configs;
        bool                     configValid = true;

        if (user_samplesPerMessage > static_cast<int32_t>(sensor_maxSamplesPerMessage)) {
			std::cerr << "Invalid samples-per-message " << user_samplesPerMessage << ", valid values are in [1," << sensor_maxSamplesPerMessage << "]" << std::endl;
            configValid = false;
        }

        //
        // Validate each command line config option against IMU information from the head

        for(uint32_t i=0; i<cli_configs.size(); i++) {
        
            char nameP[32]    = {0};
            char enabledP[32] = {0};
            int  rate         = -1;
            int range         = -1;

            if (4 != sscanf(cli_configs[i].c_str(), "%31s %31s %d %d",
                            nameP, enabledP, &rate, &range)) {
				std::cerr << "Malformed IMU config: \"" << cli_configs[i] << "\"" << std::endl;
                configValid = false;
                continue;  // keep parsing for maximum feedback
            }

            if (0 != strcasecmp(enabledP, "true") &&
                0 != strcasecmp(enabledP, "false")) {
				std::cerr << "Malformed <enabled> string \"" << enabledP << "\", must be one of \"true\" or \"false\"" << std::endl;
                configValid = false;
                continue;
            }

            //
            // Find the IMU information for this name

            imu::Info info;
            if (false == imuInfoByName(nameP, info)) {
				std::cerr << "Unknown <sensor_name> \"" << nameP << "\", query config for a list of valid names" << std::endl;
                configValid = false;
                continue;
            }

            //
            // Validate the rate/range indices

            if (rate < 0 || rate >= static_cast<int32_t>(info.rates.size())) {
				std::cerr << "Invalid rate table index " << rate << " for \"" << nameP << "\", valid indices are in [0," << (info.rates.size() - 1) << "]" << std::endl;
                configValid = false;
            }
            if (range < 0 || range >= static_cast<int32_t>(info.ranges.size())) {
				std::cerr << "Invalid range table index " << range << " for \"" << nameP << "\", " <<
					         "valid indices are in [0," << (info.ranges.size() - 1) << "]" << std::endl;
                configValid = false;
            }

            if (false == configValid)
                continue;

            //
            // We have a valid config, store it

            imu::Config c;
            c.name            = std::string(nameP);
            c.enabled         = (0 == strcasecmp(enabledP, "true"));
            c.rateTableIndex  = rate;
            c.rangeTableIndex = range;

            user_configs.push_back(c);
        }

		if (false == configValid)
			std::cerr << "Errors exist in configuration, aborting" << std::endl;
        else {
            status = channelP->setImuConfig(storeInFlash,
                                            user_samplesPerMessage,
                                            user_configs); // can be empty
			if (Status_Ok != status)
				std::cerr << "Failed to set IMU configuration: " << Channel::statusString(status) << std::endl;
			else
				std::cout << "IMU configuration updated successfully\n";
        }
    }

clean_out:
        
    Channel::Destroy(channelP);
    return 0;
}
