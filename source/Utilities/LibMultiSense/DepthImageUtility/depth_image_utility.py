#!/usr/bin/env python
#
# @file saved_image_utility.cc
#
# Copyright 2013-2025
# Carnegie Robotics, LLC
# 4501 Hatfield Street, Pittsburgh, PA 15201
# http://www.carnegierobotics.com
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Carnegie Robotics, LLC nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Significant history (date, user, job code, action):
#   2025-02-07, malvarado@carnegierobotics.com, IRAD, Created file.
#

import argparse
import cv2

import libmultisense as lms

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    channel = lms.Channel.create(channel_config)
    if not channel:
        print("Invalid channel")
        exit(1)

    config = channel.get_configuration()
    config.frames_per_second = 10.0
    if channel.set_configuration(config) != lms.Status.OK:
        print("Cannot set configuration")
        exit(1)

    if channel.start_streams([lms.DataSource.LEFT_DISPARITY_RAW]) != lms.Status.OK:
        print("Unable to start streams")
        exit(1)

    while True:
        frame = channel.get_next_image_frame()
        if frame:
            # MONO16 depth images are quantized to 1 mm per 1 pixel value
            depth_image = lms.create_depth_image(frame, lms.PixelFormat.MONO16, lms.DataSource.LEFT_DISPARITY_RAW, 65535)
            if depth_image:
                print("Saving depth image for frame id: ", frame.frame_id)
                cv2.imwrite(str(frame.frame_id) + ".png", depth_image.as_array)

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense save image utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    main(parser.parse_args())
