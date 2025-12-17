#!/usr/bin/env python
#
# @file multi_channel_utility.py
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
#   2025-12-02, malvarado@carnegierobotics.com, IRAD, Created file.
#

import argparse
import datetime
import numpy as np

import libmultisense as lms

def main(args):

    channels = []
    for ip_address in args.ip_addresses:
        channel_config = lms.ChannelConfig()
        channel_config.ip_address = ip_address
        channel_config.mtu = args.mtu

        channel = lms.Channel.create(channel_config)
        if not channel:
            print("Invalid channel")
            exit(1)


        config = channel.get_config()
        config.frames_per_second = 10.0
        config.time_config.ptp_enabled = True
        if channel.set_config(config) != lms.Status.OK:
            print("Cannot set configuration")
            exit(1)


        if channel.start_streams([lms.DataSource.LEFT_MONO_RAW]) != lms.Status.OK:
            print("Unable to start streams")
            exit(1)

        channels.append(channel)

    with lms.MultiChannelSynchronizer(channels, datetime.timedelta(milliseconds=args.tolerance)) as synchronizer:
        timeout = datetime.timedelta(milliseconds=500)
        while True:
            frames = synchronizer.get_synchronized_frame(timeout)
            if frames:
                print("sync group:")
                for frame in frames:
                    print(f"frame_id: {frame.frame_id} time: {frame.frame_time}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense multi-channel synchronization utility")
    parser.add_argument("-a", "--ip_addresses", action='append', help="The IPv4 addresses of the MultiSense to synchronize.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    parser.add_argument("-t", "--tolerance", type=int, default=50, help="The sync tolerance in milliseconds.")
    main(parser.parse_args())
