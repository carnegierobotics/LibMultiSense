#!/usr/bin/env python
#
# @file feature_detector_utility.cc
#
# Copyright 2013-2026
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
#   2026-03-26, malvarado@carnegierobotics.com, IRAD, Created file.
#

import argparse
import cv2
from jsondiff import diff

import libmultisense as lms

def main():
    parser = argparse.ArgumentParser("LibMultiSense feature detector utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    parser.add_argument("-n", "--num-features", type=int, default=1500, help="The max number of features to target.")
    parser.add_argument("-f", "--fps", type=int, default=10, help="The framerate of the camera.")
    args = parser.parse_args()

    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        config = channel.get_config()
        if config:
            config.frames_per_second = args.fps

            # Set feature configuration to enable the feature detector
            config.feature_detector_config = lms.FeatureDetectorConfig()
            config.feature_detector_config.number_of_features = args.num_features
            config.feature_detector_config.grouping_enabled = True
            config.feature_detector_config.motion_octave = 1
            status = channel.set_config(config)
            if status != lms.Status.OK:
                print("Cannot enable feature detector", lms.to_string(status))


        # Start both the rectified image and the corresponding feature stream
        sources = [lms.DataSource.LEFT_MONO_RAW, lms.DataSource.LEFT_ORB_FEATURES]
        if channel.start_streams(sources) != lms.Status.OK:
            print("Unable to start streams")
            exit(1)

        while True:
            frame = channel.get_next_image_frame()
            if frame and frame.has_image(lms.DataSource.LEFT_MONO_RAW):
                img = frame.get_image(lms.DataSource.LEFT_MONO_RAW).as_array

                # Convert grayscale to BGR for color rendering
                display_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

                # Draw features on the image
                if frame.has_feature(lms.DataSource.LEFT_ORB_FEATURES):
                    features = frame.get_feature(lms.DataSource.LEFT_ORB_FEATURES)
                    print(f"Frame {frame.frame_id}: Received {len(features.keypoints)} features")

                    for kp in features.keypoints:
                        cv2.circle(display_img, (int(kp.x), int(kp.y)), 3, (0, 255, 0), -1)

                cv2.imshow("MultiSense Features", display_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

if __name__ == '__main__':
    main()
