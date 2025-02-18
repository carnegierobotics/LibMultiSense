/**
 * @file FirmwareUpdateUtility/Updater.hh
 *
 * Copyright 2024-2025
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

#include "Updater.hh"
#include "Messages.hh"
#include "crc.h"

#include <iostream>
#include <fstream>
#include <string>

int Updater::Receive(uint8_t * buf, const size_t len, long int *RxLen)
{

    int ret = 0;
    struct sockaddr_in PeerAddr;
    long int BytesReceived = 0;
#if WIN32
    int PeerLength = sizeof(struct sockaddr_in);
#else
    socklen_t PeerLength = sizeof(struct sockaddr_in);
#endif

    // Disable narrowing conversion warning for 'len' argument
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (push)
#pragma warning (disable : 4267)
#endif
    BytesReceived = recvfrom(m_Ip->Sock(), (char*)buf, len, 0, (struct sockaddr *)&PeerAddr, &PeerLength);
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (pop)
#endif
    if (BytesReceived<0) {
        if (SOCKET_ERRNO != CRL_EAGAIN)
            std::cerr << "Failed to receive from socket: " << SOCKET_STR_ERR <<std::endl;
        return -SOCKET_ERRNO;
    } else if (BytesReceived==0) {
        std::cerr << "Socket connection closed!\n";
        return -1;
    }

    if (RxLen)
        *RxLen = BytesReceived;

    ret = BytesReceived;
    return ret;
}

 int Updater::Send(uint8_t * buf, const size_t len)
 {

#if WIN32
    int PeerLength = sizeof(struct sockaddr_in);
#else
    socklen_t PeerLength = sizeof(struct sockaddr_in);
#endif
    long int Sent = 0;
    struct sockaddr_in server = m_Ip->Server();

    // Disable narrowing conversion warning for 'len' argument
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (push)
#pragma warning (disable : 4267)
#endif
    Sent = sendto(m_Ip->Sock(), (char*)buf, len, 0, (struct sockaddr *)&server, PeerLength);
#if defined(WIN32) && !defined(__MINGW64__)
#pragma warning (pop)
#endif
    if (Sent<0)
    {
        std::cerr << "Failed to send to socket: " << SOCKET_STR_ERR << std::endl;
    }
    else if (Sent == 0)
    {
        std::cerr << "Sent 0 len datagram on socket!\n";
    }

    return Sent;

}

int Updater::SendFile(std::string &filePath, bool verbose)
{
    int ret = 0;
    uint32_t crc32_final = 0;
    uint32_t crc32_initial = 0;
    uint32_t bytesWritten = 0;
    int32_t blockSeq=0;
    long int bytesRcvd = 0;
    uint8_t chunk[Messages::BLOCK_SIZE] = {};

    std::ifstream file;
    try {
        file.open(filePath, std::ios::in | std::ios::binary);
        if (!file.good()) {
            std::cerr << "Cannot open file: `" << filePath << "`\n";
            return -1;
        }

    } catch (const std::exception &e) {
        std::cerr << "Exception accessing `" << filePath << "` Error: " << e.what() << std::endl;
        return -1;
    }

    file.seekg(0, file.end);
    long long fileLength = file.tellg();
    file.seekg(0, file.beg);

    long long l;
    do {

        file.read((char *)chunk, Messages::BLOCK_SIZE);
        l = file.gcount();
        crc32_initial = crc32(crc32_initial, chunk, static_cast<uint32_t>(l));

    } while(!file.eof());

    crc32_final = crc32_initial;

    file.clear();
    file.seekg(0, file.beg);

    const long long numBlocks = fileLength/Messages::BLOCK_SIZE;
    if (fileLength > MAX_UPDATE_FS) {
        std::cerr << "Update file too large (" << fileLength << "B), can't send\n";
        return -1;
    }
    Messages::MessageBlockAck BlockAck;

    do {
        uint32_t sent = 0;

        file.read((char *)chunk, Messages::BLOCK_SIZE);
        l = file.gcount();

        if (l == 0)
        {
            std::cerr << "Error: Failed to read chunk from File\n";
            file.close();
            return -1;
        }

        Messages::MessageFileBlock Block(static_cast<uint32_t>(fileLength), bytesWritten, crc32_final, static_cast<uint32_t>(l), blockSeq++, chunk);
        sent = Send((uint8_t*)&Block, sizeof(Block));
        if (sent!=sizeof(Block))
        {
            std::cerr << "Warning: Truncated send expected: " << sizeof(Block) << " sent: " << sent << std::endl;
        }

        bytesWritten += static_cast<uint32_t>(l);

        bytesRcvd = Receive((uint8_t *)&BlockAck, sizeof(BlockAck), NULL);
        if (bytesRcvd < 0)
        {
            std::cerr << "Socket timed out waiting for ACK, check connections and try again\n";
            file.close();
            exit(1);
        }

        if (bytesRcvd < (long int)sizeof(BlockAck))
        {
            std::cerr << "Invalid ACK\n";
        }

        if (BlockAck.Sequence != Block.Sequence)
        {
            std::cerr << "Invalid Sequence, Resending block\n";
        }

        if (verbose)
            std::cout << "Sending Block (" << BlockAck.Sequence << "/" << numBlocks << ")\r";

    } while(!file.eof());

    std::cout << "Finished sending file to camera!\n";
    file.close();
    return ret;
}
