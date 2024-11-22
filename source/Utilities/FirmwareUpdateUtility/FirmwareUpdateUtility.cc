/**
 * @file FirmwareUpdateUtility/FirmwareUpdateUtility.cc
 *
 * Copyright 2013-2024
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
 *   2024-27-08, patrick.smith@carnegierobotics.com, IRAD, Created file.
 **/

#include <signal.h>
#include <limits.h>
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <atomic>

#include <Utilities/portability/getopt/getopt.h>

#include "Messages.hh"
#include "Updater.hh"
#include "Util.hh"

static void usage() {
    std::cout << "Usage\n";
    std::cout << "\n";
    std::cout << "FirmwareUpdateUtility\n";
    std::cout << "\n";
    std::cout << "FirmwareUpdateUtility <options>\n";
    std::cout << "-a Ip Address of camera (Default 10.66.171.21)\n";
    std::cout << "-f Path of firmware update (Default update.img)\n";
    std::cout << "-h Print this menu and exit.\n\n";
}

static void signal_handler(int signal);
std::atomic<int> ExitReceived(0);

#ifndef PATH_MAX
#define PATH_MAX MAX_PATH
#endif


int main(int argc, char *argv[]) {

    char IpAddress[INET_ADDRSTRLEN] = {};
    char FilePath[PATH_MAX]         = {};
    bool UpdateComplete = false;
    int  ret = 0;
    int  opt;

    while ((opt = getopt(argc, argv, "a:f:h")) != -1) {
        switch (opt) {
        case 'a':
            strncpy(IpAddress, optarg, INET_ADDRSTRLEN - 1);
            break;
        case 'f':
            strncpy(FilePath, optarg, PATH_MAX - 1);
            break;
        case 'h':
        default: /* '?' */
            usage();
            exit(1);
        }
    }

    signal(SIGINT, signal_handler);

    if (!strcmp(IpAddress, "")) {
        strcpy(IpAddress, "10.66.171.21");
    }

    if (!strcmp(FilePath, "")) {
        strcpy(FilePath, "update.img");
    }

    if (!Util::FileExists(FilePath)) {
        std::cerr << "Error: File " << FilePath << " Does not exist\n";
        usage();
        return -1;
    }

    Ip i;
    if (i.Setup(IpAddress) < 0) {
        std::cerr << "Error Failed to setup Network\n";
        return -1;
    }

    if (i.Bind() < 0) {
        std::cerr << "Error Failed to bind to camera server\n";
        return -1;
    }

    Updater u(&i);

    if (u.SendFile(FilePath) < 0) {
        std::cerr << "Error failed to send the file " << FilePath << " to the camera!";
        return -1;
    }

    while (!UpdateComplete || !ExitReceived)
    {
        Messages::MessageUpdateStatus UpdateStatus;
        long int MsgLen = 0;

        ret = u.Receive((uint8_t *)&UpdateStatus, sizeof(UpdateStatus), &MsgLen);
        if (ret < 0)
        {
            if (ret == -CRL_EAGAIN)
            {
                usleep(100);
                continue;
            }
            std::cerr << "Error: Failed to get message from camera\n";
            return -1;
        }
        else if (ret == 0)
        {
            std::cerr << "Error: Connection closed by camera\n";
            return -1;
        }

        if (UpdateStatus.State & Messages::Status_Error)
        {
            std::cerr << "Error: Received an error from Camera\n";
        }
        else
        {
            Messages::PrintStatus(UpdateStatus.State);
            if (UpdateStatus.State & Messages::Status_UpdateComplete)
            {
                std::cout << "Notice: Update is complete, camera will power cycle!\n";
                UpdateStatus.Id = Messages::UpdateStatusId_ACK;
                UpdateStatus.Status = Messages::UpdateStatus_Done;
                ret = u.Send((uint8_t*)&UpdateStatus, sizeof(UpdateStatus));
                break;
            }
            else
            {
                UpdateStatus.Id = Messages::UpdateStatusId_ACK;
                UpdateStatus.Status = Messages::UpdateStatus_Ok;
                ret = u.Send((uint8_t *)&UpdateStatus, sizeof(UpdateStatus));
                if (ret < 0)
                {
                    std::cerr << "Error Failed to send ack to camera\n";
                }

            }

        }
    }

    return ret;
}

static void signal_handler(int signal)
{
    (void) signal;
    std::cout << "Exit Signal Received Cancelling Update\n";
    ExitReceived = 1;
}
