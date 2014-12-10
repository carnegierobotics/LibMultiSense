/**
 * @file LibMultiSense/details/flash.cc
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

#include "details/channel.hh"
#include "details/query.hh"

#include "details/wire/AckMessage.h"
#include "details/wire/SysFlashOpMessage.h"
#include "details/wire/SysFlashResponseMessage.h"

namespace crl {
namespace multisense {
namespace details {

//
// Erase a flash region

void impl::eraseFlashRegion(uint32_t region)
{
    wire::SysFlashResponse response;

    //
    // Start the erase operation

    Status status = waitData(wire::SysFlashOp(wire::SysFlashOp::OP_ERASE, region),
                             response);
    if (Status_Ok != status)
        CRL_EXCEPTION("OP_ERASE failed: %d", status);

    //
    // Check for success, or flash in progress

    switch(response.status) {
    case wire::SysFlashResponse::STATUS_SUCCESS:
    case wire::SysFlashResponse::STATUS_ERASE_IN_PROGRESS:
        break; // ok, erase is happening
    default:
        CRL_EXCEPTION("OP_ERASE ack'd, but failed: %d\n", response.status);
    }

    //
    // Wait for the erase to complete

    const double ERASE_TIMEOUT = 210.0; // seconds

    utility::TimeStamp start = utility::TimeStamp::getCurrentTime();

    int prevProgress = -1;

    while((utility::TimeStamp::getCurrentTime() - start) < ERASE_TIMEOUT) {

        //
        // Request current progress

        status = waitData(wire::SysFlashOp(), response);
        if (Status_Ok != status)
            CRL_EXCEPTION("failed to request flash erase status");

        //
        // IDLE means the flash has been erased

        if (wire::SysFlashResponse::STATUS_IDLE == response.status)
            return; // success

        //
        // Prompt and delay a bit

        if (response.erase_progress != prevProgress &&
            0 == (response.erase_progress % 5))
            CRL_DEBUG("erasing... %3d%%\n", response.erase_progress);
        usleep(100000);

        prevProgress = response.erase_progress;
    }

    CRL_EXCEPTION("erase op timed out after %.0f seconds", ERASE_TIMEOUT);
}

//
// Program or verify a flash region from a file

void impl::programOrVerifyFlashRegion(std::ifstream& file,
                                      uint32_t       operation,
                                      uint32_t       region)
{
    //
    // Get file size

    file.seekg(0, file.end);
    std::streamoff fileLength = file.tellg();
    file.seekg(0, file.beg);

    wire::SysFlashOp op(operation, region, 0,
                        wire::SysFlashOp::MAX_LENGTH);

    int prevProgress = -1;

    const char *opNameP;

    switch(operation) {
    case wire::SysFlashOp::OP_PROGRAM: opNameP = "programming"; break;
    case wire::SysFlashOp::OP_VERIFY:  opNameP = "verifying";   break;
    default: 
        CRL_EXCEPTION("unknown operation type: %d", operation);
    }

    do {

        //
        // Initialize data and read next chunk

        memset(op.data, 0xFF, op.length);
        file.read((char *) op.data, op.length);

        //
        // Send command, await response

        wire::SysFlashResponse rsp;

        Status status = waitData(op, rsp, 0.5, 4);
        if (Status_Ok != status)
            CRL_EXCEPTION("SysFlashOp (%s) failed: %d", opNameP, status);
        else if (wire::SysFlashResponse::STATUS_SUCCESS != rsp.status)
            CRL_EXCEPTION("%s failed @ %d/%d bytes", opNameP,
                          file.tellg(), fileLength);

        //
        // Print out progress

        int progress = static_cast<int> ((100 * op.start_address) / fileLength);
        if (progress != prevProgress && 0 == (progress % 5))
            CRL_DEBUG("%s... %3d%%\n", opNameP, progress);

        //
        // Update state

        prevProgress = progress;
        op.start_address += op.length;

    } while (!file.eof());

    if ((int) op.start_address < fileLength)
        CRL_EXCEPTION("unexpected EOF while %s", opNameP);

    CRL_DEBUG("%s complete\n", opNameP);
}

//
// Wrapper for all flash operations

Status impl::doFlashOp(const std::string& filename,
                       uint32_t           operation,
                       uint32_t           region)
{
    try {
        std::ifstream file(filename.c_str(), 
                           std::ios::in | std::ios::binary);

        if (!file.good()) 
            CRL_EXCEPTION("unable to open file: \"%s\"", 
                          filename.c_str());

        if (wire::SysFlashOp::OP_PROGRAM == operation)
            eraseFlashRegion(region);

        programOrVerifyFlashRegion(file, operation, region);

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Ok;
}    

}}}; // namespaces
