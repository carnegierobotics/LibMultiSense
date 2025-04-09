#!/usr/bin/env python
#
# @file change_ip_utility.cc
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
#   2025-04-07, malvarado@carnegierobotics.com, IRAD, Created file.
#

import argparse
import os

import libmultisense as lms


def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.current_ip_address
    channel_config.connect_on_initialization = (args.interface is None)

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        if not args.disable_confirmation:
            print(f"NEW address {args.new_ip_address}")
            print(f"NEW gateway {args.new_gateway}")
            print(f"NEW netmask {args.new_netmask}")

            if args.interface is not None:
                print(f"** WARNING: All MultiSense devices attached to the interface {args.interface} "
                       "will have their IPs changed **")

            res = input("Really update network configuration? (y/n):")[0]
            if res != 'Y' and res != 'y':
                print("Aborting")
                exit(1)

        config = lms.NetworkInfo()
        config.ip_address = args.new_ip_address
        config.gateway = args.new_gateway
        config.netmask = args.new_netmask
        status = channel.set_network_config(config, args.interface)
        if status != lms.Status.OK:
            print("Cannot set configuration", lms.to_string(status))
            exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense change ip utility")
    parser.add_argument("-a", "--current-ip_address", default="10.66.171.21", help="The current IPv4 address of the MultiSense.")
    parser.add_argument("-A", "--new-ip_address", default="10.66.171.21", help="The new IPv4 address of the MultiSense.")
    parser.add_argument("-G", "--new-gateway", default="10.66.171.1", help="The new IPv4 gateway of the MultiSense.")
    parser.add_argument("-N", "--new-netmask", default="255.255.255.0", help="The new IPv4 netmask of the MultiSense.")
    parser.add_argument("-b", "--interface", help="send broadcast packet to a specified network interface. "
                                                  "This resets the IP address to the new configured ip")
    parser.add_argument("-y", "--disable-confirmation", action='store_true', help="Disable confirmation prompts")
    main(parser.parse_args())
