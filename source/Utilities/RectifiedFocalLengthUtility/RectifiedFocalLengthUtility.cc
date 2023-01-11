/**
 * @file RectifiedFocalLengthUtility/RectifiedFocalLengthUtility.cc
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
 *   2022-03-05, malvarado@carnegierobotics.com, IRAD, Created file.
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
#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseTypes.hh>

using namespace crl::multisense;

namespace {  // anonymous

void usage(const char *programNameP)
{
    fprintf(stderr,
            "USAGE: %s -f <rectified_focal_length> [<options>]\n",
            programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>      : ip address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-s                   : set the rectified focal length (default is query)\n");

    exit(-1);
}


} // anonymous

int main(int    argc,
         char **argvPP)
{
    std::string ipAddress = "10.66.171.21";
    double rectifiedFocalLength = -1.0;
    bool set = false;
    bool hasAuxCamera = false;
    double currentFocalLength = 0.0;
    double tx = 0.0;
    double auxTx = 0.0;
    double auxTy = 0.0;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:f:s")))
        switch(c) {
        case 'a': ipAddress            = std::string(optarg);    break;
        case 'f': rectifiedFocalLength = atof(optarg);           break;
        case 's': set                  = true;                   break;
        default: usage(*argvPP);                                 break;
        }

    //
    // Verify options. New focal lengths should be positive and large enough for the onboard
    // stereo pipeline

    if (rectifiedFocalLength < 200.0) {
        fprintf(stderr, "Invalid rectified focal length\n");
        usage(*argvPP);
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
    // Query Device Info

    Status status;

    system::DeviceInfo info;
    status = channelP->getDeviceInfo(info);
    if (Status_Ok != status)
    {
        fprintf(stderr, "Failed to query device info: %s\n",
                Channel::statusString(status));
        goto clean_out;
    }

    hasAuxCamera = info.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 ||
                   info.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 ||
                   info.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM;

    image::Calibration calibration;

    status = channelP->getImageCalibration(calibration);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query image calibration: %s\n",
                Channel::statusString(status));
        goto clean_out;
    }

    currentFocalLength = calibration.left.P[0][0];
    fprintf(stdout, "updating focal length from %f to %f\n",
            currentFocalLength, rectifiedFocalLength);

    if (set) {

        tx = calibration.right.P[0][3] / calibration.right.P[0][0];

        calibration.left.P[0][0]  = static_cast<float> (rectifiedFocalLength);
        calibration.left.P[1][1]  = static_cast<float> (rectifiedFocalLength);
        calibration.right.P[0][0] = static_cast<float> (rectifiedFocalLength);
        calibration.right.P[1][1] = static_cast<float> (rectifiedFocalLength);
        calibration.right.P[0][3] = static_cast<float> (rectifiedFocalLength * tx);

        if (hasAuxCamera) {

            auxTx = calibration.aux.P[0][3] / calibration.aux.P[0][0];
            auxTy = calibration.aux.P[1][3] / calibration.aux.P[1][1];

            calibration.aux.P[0][0] = static_cast<float> (rectifiedFocalLength);
            calibration.aux.P[1][1] = static_cast<float> (rectifiedFocalLength);
            calibration.aux.P[0][3] = static_cast<float> (rectifiedFocalLength * auxTx);
            calibration.aux.P[1][3] = static_cast<float> (rectifiedFocalLength * auxTy);
        }

        status = channelP->setImageCalibration(calibration);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to set image calibration: %s\n",
                    Channel::statusString(status));
            goto clean_out;
        }

        fprintf(stdout, "Rectified focal length successfully updated to %f\n",
                rectifiedFocalLength);
    } else {

        fprintf(stdout, "Please specify -s to update the rectified focal length to %f\n",
                rectifiedFocalLength);
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
