/**
 * @file FlashUtility/FlashUtility.cc
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
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
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
#include <string>
#include <LibMultiSense/MultiSenseChannel.hh>

#include <fstream>

#include <Utilities/portability/getopt/getopt.h>

namespace {  // anonymous

void usage(const char *programNameP) 
{
    fprintf(stderr, "USAGE: %s <options>\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>        : IP address of device (default=10.66.171.21)\n");
    fprintf(stderr, "\t-p                     : Perform flash operation\n");
    fprintf(stderr, "\t-v                     : Perform verify operation\n");
    fprintf(stderr, "\t-b <bitstream_file>    : The bitstream (.bin) file\n");
    fprintf(stderr, "\t-f <firmware_file>     : The firmware (.srec) file\n");
}

bool verifyFileWithExtension(const std::string& description,
                             const std::string& fileName,
                             const std::string& extension)
{
    try {
        std::ifstream file(fileName.c_str(),
                           std::ios::in | std::ios::binary);

        if (false == file.good()) {
            fprintf(stderr, "Cannot open %s file \"%s\" for reading, aborting.\n",
                    description.c_str(), fileName.c_str());
            return false;
        }

    } catch (const std::exception& e) {
        fprintf(stderr, "Exception accessing %s file \"%s\" for reading: %s.\n",
                description.c_str(), fileName.c_str(), e.what());
        return false;
    }

    std::string::size_type idx = fileName.rfind('.');

    if (std::string::npos != idx &&
        fileName.substr(idx+1) == extension)
        return true;

    fprintf(stderr, "%s file \"%s\" is not a \".%s\" file, aborting.\n",
            description.c_str(), fileName.c_str(), extension.c_str());

    return false;
}

}; // anonymous

using namespace crl::multisense;

int main(int    argc, 
         char **argvPP)
{
    std::string ipAddress  = "10.66.171.21";
    bool        programOp  = false;
    bool        verifyOp   = false;
    int         returnCode = 0;
    std::string bitstreamFile;
    std::string firmwareFile;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:epvb:f:")))
        switch(c) {
        case 'a': ipAddress     = std::string(optarg);    break;
        case 'p': programOp     = true;                   break;
        case 'v': verifyOp      = true;                   break;
        case 'b': bitstreamFile = std::string(optarg);    break;
        case 'f': firmwareFile  = std::string(optarg);    break;
        default: usage(*argvPP); exit(-1);                break;
        }

    //
    // Verify that the bitstream/firmware filenames are sane, and that the files exist

    if (false == bitstreamFile.empty() && 
        false == verifyFileWithExtension("Bitstream", bitstreamFile, "bin"))
        exit(-2);
    if (false == firmwareFile.empty() && 
        false == verifyFileWithExtension("Firmware", firmwareFile, "srec"))
        exit(-3);

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(ipAddress);
    if (NULL == channelP) {
	fprintf(stderr, "Failed to establish communications with \"%s\"\n",
		ipAddress.c_str());
        exit(-4);
    }
    
    //
    // Perform any programming operations first

    Status status=Status_Ok;

    if (programOp) {

        if (!bitstreamFile.empty()) {
            
            fprintf(stderr, "Programming bitstream: %s\n",
                    bitstreamFile.c_str());

            status = channelP->flashBitstream(bitstreamFile); 
            if (Status_Ok != status) {
                fprintf(stderr, "Programming bitstream failed: %s\n",
                        Channel::statusString(status));
                returnCode = -6; 
                goto clean_out;
            }
        }

        if (!firmwareFile.empty()) {
            
            fprintf(stderr, "Programming firmware: %s\n",
                    firmwareFile.c_str());

            status = channelP->flashFirmware(firmwareFile);
            if (Status_Ok != status) {
                fprintf(stderr, "Programming firmware failed: %s\n",
                        Channel::statusString(status));
                returnCode = -7;
                goto clean_out;
            }
        }
    }

    //
    // Perform any verification operations

    if (verifyOp) {

        if (!bitstreamFile.empty()) {
            
            fprintf(stderr, "Verifying bitstream: %s\n",
                    bitstreamFile.c_str());

            status = channelP->verifyBitstream(bitstreamFile);
            if (Status_Ok != status) {
                fprintf(stderr, "Verify bitstream failed: %s\n",
                        Channel::statusString(status));
                returnCode = -8;
                goto clean_out;
            }
        }

        if (!firmwareFile.empty()) {
            
            fprintf(stderr, "Verifying firmware: %s\n",
                    firmwareFile.c_str());

            status = channelP->verifyFirmware(firmwareFile);
            if (Status_Ok != status) {
                fprintf(stderr, "Verify firmware failed: %s\n",
                        Channel::statusString(status));
                returnCode = -9;
                goto clean_out;
            }
        }
    }

clean_out:

    Channel::Destroy(channelP);
    return returnCode;
}
