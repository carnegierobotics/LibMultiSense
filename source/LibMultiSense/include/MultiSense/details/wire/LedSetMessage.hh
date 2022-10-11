/**
 * @file LibMultiSense/LedSetMessage.hh
 *
 * This message contains the current camera configuration.
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
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, Significant rewrite.
 *   2012-07-17, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_LedSetMessage
#define LibMultiSense_LedSetMessage

#include "MultiSense/details/utility/Portability.hh"

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class LedSet {
public:
    static CRL_CONSTEXPR IdType      ID      = ID_CMD_LED_SET;
    static CRL_CONSTEXPR VersionType VERSION = 4;

    //
    // Bit mask selecting which LEDs to update

    uint8_t mask;

    //
    // LED duty cycles; 0 = off; 255 = 100%

    uint8_t intensity[lighting::MAX_LIGHTS];

    //
    // If non-zero, LEDs are only on while sensors are exposing

    uint8_t flash;

    //
    // The delay of the LED from turning on to visible light seen in microseconds.
    // Should be measured and specific to the LED in question.

    uint32_t led_delay_us;

    //
    // For LED synchronized to shutter the number of pulses is how many pulses
    // per exposure of an image.

    uint32_t number_of_pulses;

    //
    // Invert the output signal that drives lighting. 1 means the output
    // will be low during the exposure. 0 means the output will be high
    // during the exposure.

    uint8_t invert_pulse;

    //
    // Used to enable the rolling shutter led feature, applicable for cameras
    // with a rolling shutter imager.

    uint8_t rolling_shutter_led;

    //
    // Constructors

    LedSet(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    LedSet() : mask(0), flash(0), led_delay_us(0), number_of_pulses(1),
    invert_pulse(0), rolling_shutter_led(0) {};

    //
    // Serialization routine

    template <class Archiver>
        void serialize(Archiver& archive,
                       const VersionType version)
    {
        (void) version;
        archive & mask;
        for(uint32_t i=0; i<lighting::MAX_LIGHTS; i++)
            archive & intensity[i];
        archive & flash;

        if (version >= 2)
        {
          archive & led_delay_us;
          archive & number_of_pulses;
        }
        else
        {
          led_delay_us = 0;
          number_of_pulses = 1;
        }

        if (version >= 3)
        {
          archive & invert_pulse;
        }
        else
        {
          invert_pulse = 0;
        }

        if (version >= 4)
        {
          archive & rolling_shutter_led;
        }
        else
        {
          rolling_shutter_led = 0;
        }
    }
};

}}}} // namespaces

#endif
