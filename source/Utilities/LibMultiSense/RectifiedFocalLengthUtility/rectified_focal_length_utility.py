#!/usr/bin/env python
#
# @file rectified_focal_length_utility.cc
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
#   2025-02-08, malvarado@carnegierobotics.com, IRAD, Created file.
#

import argparse

import libmultisense as lms

def update_calibration(cal, new_focal_length):
    print("Updating focal length from", cal.left.P[0][0], "to", new_focal_length)

    cal.left.P[0][0] = new_focal_length
    cal.left.P[1][1] = new_focal_length

    right_tx = cal.right.P[0][3] / cal.right.P[0][0]
    cal.right.P[0][0] = new_focal_length
    cal.right.P[1][1] = new_focal_length
    cal.right.P[0][3] = new_focal_length * right_tx

    if cal.aux:
        aux_tx = cal.aux.P[0][3] / cal.aux.P[0][0]
        aux_ty = cal.aux.P[1][3] / cal.aux.P[1][1]

        cal.aux.P[0][0] = new_focal_length
        cal.aux.P[1][1] = new_focal_length
        cal.aux.P[0][3] = new_focal_length * aux_tx
        cal.aux.P[1][3] = new_focal_length * aux_ty

    return cal;

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        current_calibration = channel.get_calibration();

        new_calibration = update_calibration(current_calibration, args.rectified_focal_length)

        if args.set:
            if channel.set_calibration(new_calibration) != lms.Status.OK:
                print("Failed to set the updated calibration")
                exit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense save image utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    parser.add_argument("-r", "--rectified-focal-length", type=float, required=True, help="The new rectified focal length")
    parser.add_argument("-s", "--set", action="store_true", help="Write the new calibration to camera")
    main(parser.parse_args())
