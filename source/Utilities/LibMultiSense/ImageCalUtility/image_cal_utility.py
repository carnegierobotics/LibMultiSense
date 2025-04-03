#!/usr/bin/env python
#
# @file image_cal_utility.cc
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
#   2025-03-25, malvarado@carnegierobotics.com, IRAD, Created file.
#

import argparse
import cv2
import numpy as np
import os

import libmultisense as lms

def read_cal(intrinsics, extrinsics, index):
    cal = lms.CameraCalibration()

    cal.K = intrinsics.getNode("M" + str(index)).mat()
    cal.D = list(intrinsics.getNode("D" + str(index)).mat()[0])
    cal.R = extrinsics.getNode("R" + str(index)).mat()
    cal.P = extrinsics.getNode("P" + str(index)).mat()

    if len(cal.D) == 5:
        cal.distortion_type = lms.DistortionType.PLUMBBOB
    elif len(cal.D) == 8:
        cal.distortion_type = lms.DistortionType.RATIONAL_POLYNOMIAL
    else:
        cal.distortion_type = lms.DistortionType.NONE

    return cal

def write_cal(intrinsics, extrinsics, index, camera_cal):
    intrinsics.write("M" + str(index), np.array(camera_cal.K))
    intrinsics.write("D" + str(index), np.array([camera_cal.D]))
    extrinsics.write("P" + str(index), np.array(camera_cal.P))
    extrinsics.write("R" + str(index), np.array(camera_cal.R))

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        calibration = channel.get_calibration()

        if args.set_cal:
            print("Attempting to set the MultiSense calibration")

            intrinsics = cv2.FileStorage(args.intrinsics, cv2.FILE_STORAGE_READ)
            extrinsics = cv2.FileStorage(args.extrinsics, cv2.FILE_STORAGE_READ)

            calibration.left = read_cal(intrinsics, extrinsics, 1)
            calibration.right = read_cal(intrinsics, extrinsics, 2)
            if calibration.aux:
                calibration.aux = read_cal(intrinsics, extrinsics, 3)

            if channel.set_calibration(calibration) != lms.Status.OK:
                print("Unable to set the calibration")
                exit(1)

            print("Image calibration successfully updated")
        else:
            if not args.disable_confirmation and (os.path.exists(args.intrinsics) or os.path.exists(args.extrinsics)):
                print("One or both of the input file already exists\n")
                res = input("Really overwrite these files? (y/n):")[0]
                if res != 'Y' and res != 'y':
                    print("Aborting")
                    exit(1)

            intrinsics = cv2.FileStorage(args.intrinsics, cv2.FILE_STORAGE_WRITE)
            extrinsics = cv2.FileStorage(args.extrinsics, cv2.FILE_STORAGE_WRITE)

            write_cal(intrinsics, extrinsics, 1, calibration.left)
            write_cal(intrinsics, extrinsics, 2, calibration.right)
            if calibration.aux:
                write_cal(intrinsics, extrinsics, 3, calibration.aux)

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense save image utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    parser.add_argument("-i", "--intrinsics", required=True, help="The path to the intrinsics file to read from/write to.")
    parser.add_argument("-e", "--extrinsics", required=True, help="The path to the extrinsics file to read from/write to.")
    parser.add_argument("-s", "--set-cal", action='store_true', help="Write the calibration to the camera")
    parser.add_argument("-y", "--disable-confirmation", action='store_true', help="Disable confirmation prompts")
    main(parser.parse_args())
