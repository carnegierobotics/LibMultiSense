/**
 * @file utilities.cc
 *
 * Copyright 2013-2026
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
 *   2026-04-17, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include "details/amb/utilities.hh"

#include <vector>

namespace multisense{
namespace amb{

std::string base64_decode(const std::string &input)
{
    std::string decoded_string;

    // Create a lookup table initialized to -1
    std::vector<int> base64_lookup(256, -1);

    // Populate the lookup table with Base64 characters
    const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    for (int i = 0; i < 64; i++)
    {
        base64_lookup[base64_chars[i]] = i;
    }

    int val = 0;
    int valb = -8;

    for (unsigned char c : input)
    {
        if (base64_lookup[c] == -1)
        {
            break;
        }

        //
        // Shift the current value over by 6 bits and add the new 6 bits
        //
        val = (val << 6) + base64_lookup[c];
        valb += 6;

        //
        // Once we have at least 8 bits (a full byte), extract it
        //
        if (valb >= 0)
        {
            decoded_string.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return decoded_string;
}

}
}
