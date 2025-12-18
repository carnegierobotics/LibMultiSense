#!/usr/bin/env python
#
# @file version_info_utility.cc
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
import datetime
import time

import libmultisense as lms

def get_ptp_status_string(status):
    if status.ptp is None:
        return "PTP not supported"

    grandmaster_id = ",".join(str(i) for i in status.ptp.grandmaster_id)

    output = f"Grandmaster present: {status.ptp.grandmaster_present}, "\
             f"Grandmaster id: {grandmaster_id}," \
             f"Grandmaster offset: {status.ptp.grandmaster_offset}," \
             f"Path delay: {status.ptp.path_delay}," \
             f"Steps from local to grandmaster: {status.ptp.steps_from_local_to_grandmaster}"

    return output

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

    config = channel.get_config()
    if config.time_config is None:
        print("Camera does not support PTP")
        exit(1)
    config.time_config.ptp_enabled = True
    status = channel.set_config(config)
    if status != lms.Status.OK:
        print("Cannot set configuration", lms.to_string(status))
        exit(1)

    if channel.start_streams([lms.DataSource.LEFT_RECTIFIED_RAW]) != lms.Status.OK:
        print("Unable to start streams")
        exit(1)

    while True:
        frame = channel.get_next_image_frame()
        if frame:
            now = datetime.datetime.now()
            delay = frame.ptp_frame_time - now
            print(f"Approximate time offset between camera and system: {delay}")

        status = channel.get_system_status()
        if status:
            print(get_ptp_status_string(status))

        time.sleep(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense version info utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    main(parser.parse_args())
