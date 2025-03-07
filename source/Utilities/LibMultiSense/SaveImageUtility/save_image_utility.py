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
import time
import cv2

import libmultisense as lms

def get_status_string(status):
    output = f"Camera Time(ns): {status.time.camera_time}, "\
             f"System Ok: {status.system_ok}, "

    temp = ""
    if status.temperature:
        temp = f"FPGA Temp (C): {status.temperature.fpga_temperature}, "\
               f"Left Imager Temp (C): {status.temperature.left_imager_temperature}, "\
               f"Right Imager Temp (C): {status.temperature.right_imager_temperature}, "

    power = ""
    if status.power:
        power = f"Input Voltage (V): {status.power.input_voltage}, "\
                f"Input Current (A): {status.power.input_current}, "\
                f"FPGA Power (W): {status.power.fpga_power}, "\

    stats = f"Received Messages {status.client_network.received_messages}, "\
            f"Dropped Messages {status.client_network.dropped_messages}"

    return output + temp + power + stats

def save_image(frame, source):
    base_path = str(frame.frame_id) + "_" + str(source);

    if source == lms.DataSource.LEFT_DISPARITY_RAW:
        depth_image = lms.create_depth_image(frame, lms.PixelFormat.MONO16, lms.DataSource.LEFT_DISPARITY_RAW, 65535)
        if depth_image:
            lms.write_image(depth_image, base_path + ".pgm")
    elif source == lms.DataSource.LEFT_RECTIFIED_RAW:
        lms.write_image(frame.get_image(source), base_path + ".pgm")
    elif source == lms.DataSource.AUX_RAW:
        bgr = lms.create_bgr_image(frame, lms.DataSource.AUX_RAW)
        if bgr:
            lms.write_image(bgr, base_path + ".ppm")

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            return

        info = channel.get_info()

        print("Firmware build date :  ", info.version.firmware_build_date)
        print("Firmware version    :  ", info.version.firmware_version.to_string())
        print("Hardware version    :  ", hex(info.version.hardware_version))

        config = channel.get_configuration()
        config.frames_per_second = 30.0
        if channel.set_configuration(config) != lms.Status.OK:
            print("Cannot set configuration")
            exit(1)

        streams = []
        if args.save_depth:
            streams.append(lms.DataSource.LEFT_DISPARITY_RAW)
        if args.save_left_rect:
            streams.append(lms.DataSource.LEFT_RECTIFIED_RAW)
        if args.save_color:
            streams.append(lms.DataSource.AUX_RAW)

        if channel.start_streams(streams) != lms.Status.OK:
            print("Unable to start streams")
            exit(1)

        #Only save the first image
        saved_images = 0

        while True:
            if saved_images < args.number_of_images:
                frame = channel.get_next_image_frame()
                if frame:
                    for stream in streams:
                        save_image(frame, stream)

                saved_images += 1

            status = channel.get_system_status()
            if status:
                print(get_status_string(status))

            time.sleep(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense save image utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    parser.add_argument("-n", "--number-of-images", type=int, default=1, help="The number of images to save.")
    parser.add_argument("-d", "--save-depth", action='store_true', help="Save a 16 bit depth image.")
    parser.add_argument("-l", "--save-left-rect", action='store_true', help="Save a left rectified image.")
    parser.add_argument("-c", "--save-color", action='store_true', help="Save a color image.")
    main(parser.parse_args())
