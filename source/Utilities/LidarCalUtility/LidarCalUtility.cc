/**
 * @file LidarCalUtility/LidarCalUtility.cc
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
 *   2013-05-23, ekratzer@carnegierobotics.com, PR1044, Created file.
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

#include <Utilities/portability/getopt/getopt.h>
#include <Utilities/shared/CalibrationYaml.hh>
#include <LibMultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

void usage(const char *programNameP) 
{
    fprintf(stderr, 
            "USAGE: %s -f <calibration_file> [<options>]\n", 
            programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>      : ip address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-s                   : set the calibration (default is query)\n");
    fprintf(stderr, "\t-y                   : disable confirmation prompts\n");
    
    exit(-1);
}

bool fileExists(const std::string& name)
{
    struct stat sbuf;
    return (0 == stat(name.c_str(), &sbuf));
}

const char *laserToSpindleNameP       = "laser_T_spindle";
const char *cameraToSpindleFixedNameP = "camera_T_spindle_fixed";


std::ostream& writeLaserCal (std::ostream& stream, lidar::Calibration const& calibration)
{
    stream << "%YAML:1.0\n";
    writeMatrix (stream, laserToSpindleNameP, 4, 4, &calibration.laserToSpindle[0][0]);
    writeMatrix (stream, cameraToSpindleFixedNameP, 4, 4, &calibration.cameraToSpindleFixed[0][0]);
    return stream;
}


}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    std::string        ipAddress = "10.66.171.21";
    std::string        calFile;
    bool               setCal=false;
    bool               prompt=true;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:f:sy")))
        switch(c) {
        case 'a': ipAddress = std::string(optarg);    break;
        case 'f': calFile   = std::string(optarg);    break;
        case 's': setCal    = true;                   break;
        case 'y': prompt    = false;                  break;
        default: usage(*argvPP);                      break;
        }

    //
    // Verify options

    if (calFile.empty()) {
        fprintf(stderr, "Must provide a file argument\n");
        usage(*argvPP);
    }

    if (true == setCal && false == fileExists(calFile)) {
        
        fprintf(stderr, "file not found: \"%s\"\n", calFile.c_str());
        usage(*argvPP);
    }
    
    if (false == setCal && true == prompt &&
        true == fileExists(calFile)) {
        
        fprintf(stdout, 
                "\"%s\" already exists.\n\n"
                "Really overwrite this file? (y/n): ",
                calFile.c_str());
        fflush(stdout);

        int c = getchar();
        if ('Y' != c && 'y' != c) {
            fprintf(stdout, "Aborting\n");
            return 0;
        }
    }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(ipAddress);
    if (NULL == channelP) {
	fprintf(stderr, "Failed to establish communications with \"%s\"\n",
		ipAddress.c_str());
	return -1;
    }

    //
    // Query version

    Status status;
    VersionType version;

    status = channelP->getSensorVersion(version);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query sensor version: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }
    
    //
    // Query

    if (false == setCal) {

        lidar::Calibration c;

        status = channelP->getLidarCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to query lidar calibration: %s\n", 
                    Channel::statusString(status));
            goto clean_out;
        }

        std::ofstream cvFile (calFile.c_str (), std::ios_base::out | std::ios_base::trunc);

        if (!cvFile) {
            fprintf(stderr, "failed to open '%s' for writing\n", 
                    calFile.c_str());
            goto clean_out;
        }

        writeLaserCal (cvFile, c);

        cvFile.flush ();

    } else {

        std::map<std::string, std::vector<float> > data;
        std::ifstream cvFile (calFile.c_str ());

        if (!cvFile) {
            fprintf(stderr, "failed to open '%s' for reading\n", 
                    calFile.c_str());
            goto clean_out;
        }

        parseYaml (cvFile, data);

        cvFile.close ();

        if (data[laserToSpindleNameP].size () != 4 * 4 ||
            data[cameraToSpindleFixedNameP].size () != 4 * 4) {

            fprintf(stderr, "calibration matrices incomplete in %s, "
                    "expecting two 4x4 matrices\n", calFile.c_str());
            goto clean_out;
        }

        lidar::Calibration c;

        memcpy (&c.laserToSpindle[0][0], &data[laserToSpindleNameP].front (), data[laserToSpindleNameP].size () * sizeof (float));
        memcpy (&c.cameraToSpindleFixed[0][0], &data[cameraToSpindleFixedNameP].front (), data[cameraToSpindleFixedNameP].size () * sizeof (float));
        
        status = channelP->setLidarCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to set lidar calibration: %s\n", 
                    Channel::statusString(status));
            goto clean_out;
        }

        fprintf(stdout, "Lidar calibration successfully updated\n");
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
