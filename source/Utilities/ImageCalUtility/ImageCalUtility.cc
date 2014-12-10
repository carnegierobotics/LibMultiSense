/**
 * @file ImageCalUtility/ImageCalUtility.cc
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
            "USAGE: %s -e <extrinisics_file> -i <intrinsics_file> [<options>]\n", 
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

std::ostream& writeImageIntrinics (std::ostream& stream, image::Calibration const& calibration)
{
    stream << "%YAML:1.0\n";
    writeMatrix (stream, "M1", 3, 3, &calibration.left.M[0][0]);
    writeMatrix (stream, "D1", 1, 8, &calibration.left.D[0]);
    writeMatrix (stream, "M2", 3, 3, &calibration.right.M[0][0]);
    writeMatrix (stream, "D2", 1, 8, &calibration.right.D[0]);
    return stream;
}

std::ostream& writeImageExtrinics (std::ostream& stream, image::Calibration const& calibration)
{
    stream << "%YAML:1.0\n";
    writeMatrix (stream, "R1", 3, 3, &calibration.left.R[0][0]);
    writeMatrix (stream, "P1", 3, 4, &calibration.left.P[0][0]);
    writeMatrix (stream, "R2", 3, 3, &calibration.right.R[0][0]);
    writeMatrix (stream, "P2", 3, 4, &calibration.right.P[0][0]);
    return stream;
}

}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    std::string ipAddress = "10.66.171.21";
    std::string intrinsicsFile;
    std::string extrinsicsFile;
    bool        setCal=false;
    bool        prompt=true;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:e:i:sy")))
        switch(c) {
        case 'a': ipAddress      = std::string(optarg);    break;
        case 'i': intrinsicsFile = std::string(optarg);    break;
        case 'e': extrinsicsFile = std::string(optarg);    break;
        case 's': setCal         = true;                   break;
        case 'y': prompt         = false;                  break;
        default: usage(*argvPP);                           break;
        }

    //
    // Verify options

    if (intrinsicsFile.empty() || extrinsicsFile.empty()) {
        fprintf(stderr, "Both intrinsics and extrinsics files must be set\n");
        usage(*argvPP);
    }

    if (true == setCal &&
        (false == fileExists(intrinsicsFile) ||
         false == fileExists(extrinsicsFile))) {
        
        fprintf(stderr, "intrinsics or extrinsics file not found\n");
        usage(*argvPP);
    }

    if (false == setCal && true == prompt &&
        (true == fileExists(intrinsicsFile) ||
         true == fileExists(extrinsicsFile))) {
        
        fprintf(stdout, 
                "One or both of \"%s\" and \"%s\" already exists.\n\n"
                "Really overwrite these files? (y/n): ",
                intrinsicsFile.c_str(),
                extrinsicsFile.c_str());
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

        image::Calibration c;

        status = channelP->getImageCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to query image calibration: %s\n", 
                    Channel::statusString(status));
            goto clean_out;
        }

        std::ofstream inFile, exFile;

        inFile.open (intrinsicsFile.c_str (), std::ios_base::out | std::ios_base::trunc);

        if (!inFile) {
            fprintf(stderr, "failed to open '%s' for writing\n", 
                    intrinsicsFile.c_str());
            goto clean_out;
        }

        exFile.open (extrinsicsFile.c_str (), std::ios_base::out | std::ios_base::trunc);

        if (!exFile) {
            fprintf(stderr, "failed to open '%s' for writing\n", 
                    extrinsicsFile.c_str());
            goto clean_out;
        }

        writeImageIntrinics (inFile, c);
        writeImageExtrinics (exFile, c);

        inFile.flush ();
        exFile.flush ();

    } else {

        std::ifstream inFile, exFile;
        std::map<std::string, std::vector<float> > data;

        inFile.open (intrinsicsFile.c_str ());
        
        if (!inFile) {
            fprintf(stderr, "failed to open '%s' for reading\n", 
                    intrinsicsFile.c_str());
            goto clean_out;
        }

        parseYaml (inFile, data);

        inFile.close ();

        if (data["M1"].size () != 3 * 3 ||
            (data["D1"].size () != 5 && data["D1"].size () != 8) ||
            data["M2"].size () != 3 * 3 ||
            (data["D2"].size () != 5 && data["D2"].size () != 8)) {
            fprintf(stderr, "intrinsic matrices incomplete in %s\n",
                    intrinsicsFile.c_str());
            goto clean_out;
        }

        exFile.open (extrinsicsFile.c_str ());

        if (!exFile) {
            fprintf(stderr, "failed to open '%s' for reading\n", 
                    extrinsicsFile.c_str());
            goto clean_out;
        }

        parseYaml (exFile, data);

        exFile.close ();

        if (data["R1"].size () != 3 * 3 ||
            data["P1"].size () != 3 * 4 ||
            data["R2"].size () != 3 * 3 ||
            data["P2"].size () != 3 * 4) {
            fprintf(stderr, "extrinsic matrices incomplete in %s\n",
                    extrinsicsFile.c_str());
            goto clean_out;
        }

        image::Calibration c;

        memcpy (&c.left.M[0][0], &data["M1"].front (), data["M1"].size () * sizeof (float));
        memset (&c.left.D[0], 0, sizeof (c.left.D));
        memcpy (&c.left.D[0], &data["D1"].front (), data["D1"].size () * sizeof (float));
        memcpy (&c.left.R[0][0], &data["R1"].front (), data["R1"].size () * sizeof (float));
        memcpy (&c.left.P[0][0], &data["P1"].front (), data["P1"].size () * sizeof (float));

        memcpy (&c.right.M[0][0], &data["M2"].front (), data["M2"].size () * sizeof (float));
        memset (&c.right.D[0], 0, sizeof (c.right.D));
        memcpy (&c.right.D[0], &data["D2"].front (), data["D2"].size () * sizeof (float));
        memcpy (&c.right.R[0][0], &data["R2"].front (), data["R2"].size () * sizeof (float));
        memcpy (&c.right.P[0][0], &data["P2"].front (), data["P2"].size () * sizeof (float));

        status = channelP->setImageCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to set image calibration: %s\n", 
                    Channel::statusString(status));
            goto clean_out;
        }

        fprintf(stdout, "Image calibration successfully updated\n");
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
