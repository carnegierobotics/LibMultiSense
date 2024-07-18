--[[
 - @file multisense.lua
 -
 - Copyright 2024
 - Carnegie Robotics, LLC
 - 4501 Hatfield Street, Pittsburgh, PA 15201
 - http://www.carnegierobotics.com
 -
 - All rights reserved.
 -
 - Redistribution and use in source and binary forms, with or without
 - modification, are permitted provided that the following conditions are met:
 -     * Redistributions of source code must retain the above copyright
 -       notice, this list of conditions and the following disclaimer.
 -     * Redistributions in binary form must reproduce the above copyright
 -       notice, this list of conditions and the following disclaimer in the
 -       documentation and/or other materials provided with the distribution.
 -     * Neither the name of the Carnegie Robotics, LLC nor the
 -       names of its contributors may be used to endorse or promote products
 -       derived from this software without specific prior written permission.
 -
 - THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 - ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 - WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 - DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 - DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 - (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 - LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 - ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 - (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 - SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 -
 - Significant history (date, user, job code, action):
 -   2024-07-18, jyakubik@carnegierobotics.com, IRAD, Created file.
 ]]--

local dissector = Dissector.get("data")

local fragments = {}
local messages = {}

local multisense_protocol = Proto("MultiSense", "MultiSense Wire Protocol")

local multisense_header_magic = ProtoField.uint16("multisense.header.magic", "magic", base.HEX)
local multisense_header_version = ProtoField.uint16("multisense.header.version", "version", base.HEX)
local multisense_header_group = ProtoField.uint16("multisense.header.group", "group", base.DEC)
local multisense_header_flags = ProtoField.uint16("multisense.header.flags", "flags", base.HEX)
local multisense_header_sequenceIdentifier = ProtoField.uint16("multisense.header.sequenceIdentifier", "sequenceIdentifier", base.DEC)
local multisense_header_messageLength = ProtoField.uint32("multisense.header.messageLength", "messageLength", base.DEC)
local multisense_header_byteOffset = ProtoField.uint32("multisense.header.byteOffset", "byteOffset", base.DEC)
local multisense_header_type = ProtoField.uint16("multisense.header.type", "type", base.HEX)

local multisense_lost_packet = ProtoExpert.new("multisense.lostPacket.expoert", "Did not capture prior packet", expert.group.MALFORMED, expert.severity.ERROR)

local multisense_payload_bytes = ProtoField.bytes("multisense.payload.bytes", "bytes")

multisense_protocol.fields = {
    multisense_header_magic,
    multisense_header_version,
    multisense_header_group,
    multisense_header_flags,
    multisense_header_sequenceIdentifier,
    multisense_header_messageLength,
    multisense_header_byteOffset,
    multisense_header_type,
    multisense_payload_bytes
}

multisense_protocol.experts = {
    multisense_lost_packet
}

function multisense_protocol.init()
    fragments = {}
    messages = {}
end

function multisense_protocol.dissector(tvb, pinfo, tree)
    length = tvb:len()
    if length == 0 then return end

    pinfo.cols.protocol = multisense_protocol.name

    local header = tree:add(multisense_protocol, tvb(0, 20), "MultiSense Wire Header")
    header:add_le(multisense_header_magic, tvb(0, 2))
    header:add_le(multisense_header_version, tvb(2, 2))
    header:add_le(multisense_header_group, tvb(4, 2))
    header:add_le(multisense_header_flags, tvb(6, 2))
    header:add_le(multisense_header_sequenceIdentifier, tvb(8, 2))
    header:add_le(multisense_header_messageLength, tvb(10, 4))
    header:add_le(multisense_header_byteOffset, tvb(14, 4))
    local type_field = header:add_le(multisense_header_type, tvb(18, 2))

    local sequenceIdentifier = tvb(8, 2):le_uint()
    if pinfo.visited == false then
        local messageLength = tvb(10, 4):le_uint()
        local byteOffset = tvb(14, 4):le_uint()

        if fragments[sequenceIdentifier] == nil then
            fragments[sequenceIdentifier] = {}
        end

        if fragments[sequenceIdentifier][byteOffset] == nil then
            fragments[sequenceIdentifier][byteOffset] = tvb(18):bytes()
        end

        if byteOffset + tvb(18):len() == messageLength then
            pinfo.cols.info:set("FOO BAR")
            byteOffset = 0
            while byteOffset < messageLength do
                if fragments[sequenceIdentifier][byteOffset] == nil then
                    print("Lost packet!!!", sequenceIdentifier)
                    messages[sequenceIdentifier] = "failed"
                    break
                end
                byteOffset = byteOffset + fragments[sequenceIdentifier][byteOffset]:len()
            end

            if byteOffset == messageLength then
                print("Complete message!", sequenceIdentifier)
            end
        end
    end
    
    if messages[sequenceIdentifier] == "failed" then
        print(header:add_proto_expert_info(multisense_lost_packet))
    end

    if tvb(14, 4):le_uint() ~= 0 then return end

    local payload = nil;
    local idType = tvb(18, 2):le_uint()
    if idType == 0x0001 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "ACK")
    elseif idType == 0x0002 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_GET_VERSION")
    elseif idType == 0x0003 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_GET_STATUS")
    elseif idType == 0x0004 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_CAM_GET_CONFIG")
    elseif idType == 0x0007 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_CAM_CONTROL")
    elseif idType == 0x0008 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_CAM_GET_HISTORY")
    elseif idType == 0x000b then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_CAM_SET_HDR")
    elseif idType == 0x000c then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_CAM_SET_RESOLUTION")
    elseif idType == 0x000d then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_LIDAR_GET_CONFIG")
    elseif idType == 0x0010 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_LIDAR_SET_MOTOR")
    elseif idType == 0x0012 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_LED_GET_STATUS")
    elseif idType == 0x0013 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_LED_SET")
    elseif idType == 0x0014 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_MTU")
    elseif idType == 0x0015 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_FLASH_OP")
    elseif idType == 0x0016 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_SET_NETWORK")
    elseif idType == 0x0017 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_DEVICE_INFO")
    elseif idType == 0x0018 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_CAMERA_CAL")
    elseif idType == 0x0019 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_LIDAR_CAL")
    elseif idType == 0x001a then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_MTU")
    elseif idType == 0x001b then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_NETWORK")
    elseif idType == 0x001c then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_STREAM_CONTROL")
    elseif idType == 0x001d then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_DEVICE_MODES")
    elseif idType == 0x001e then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_CAM_SET_TRIGGER_SOURCE")
    elseif idType == 0x001f then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_IMU_GET_INFO")
    elseif idType == 0x0020 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_IMU_GET_CONFIG")
    elseif idType == 0x0021 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_TEST_MTU")
    elseif idType == 0x0022 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_DIRECTED_STREAMS")
    elseif idType == 0x0023 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_SENSOR_CAL")
    elseif idType == 0x0024 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_EXTERNAL_CAL")
    elseif idType == 0x0025 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_LED_GET_SENSOR_STATUS")
    elseif idType == 0x0026 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_SET_TRANSMIT_DELAY")
    elseif idType == 0x0027 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_GET_TRANSMIT_DELAY")
    elseif idType == 0x0028 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_MOTOR_POLL")
    elseif idType == 0x0029 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_SYS_SET_PTP")
    elseif idType == 0x002a then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_REMOTE_HEAD_GET_CONFIG")
    elseif idType == 0x002b then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_REMOTE_HEAD_CONTROL")
    elseif idType == 0x0102 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_VERSION")
    elseif idType == 0x0103 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_STATUS")
    elseif idType == 0x0104 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_CAM_CONFIG")
    elseif idType == 0x0105 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_CAM_HISTORY")
    elseif idType == 0x0108 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_LIDAR_CONFIG")
    elseif idType == 0x0109 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_LIDAR_SCAN")
    elseif idType == 0x010a then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_LED_STATUS")
    elseif idType == 0x010b then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_FLASH_RESPONSE")
    elseif idType == 0x010c then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_DEVICE_INFO")
    elseif idType == 0x010d then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_CAMERA_CAL")
    elseif idType == 0x010e then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_LIDAR_CAL")
    elseif idType == 0x010f then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_IMAGE_META")
    elseif idType == 0x0110 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_IMAGE")
    elseif idType == 0x0111 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_DISPARITY")
    elseif idType == 0x0112 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_DEVICE_MODES")
    elseif idType == 0x0113 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_PPS")
    elseif idType == 0x0114 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_IMU")
    elseif idType == 0x0115 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_IMU_INFO")
    elseif idType == 0x0116 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_IMU_CONFIG")
    elseif idType == 0x0117 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_TEST_MTU_RESPONSE")
    elseif idType == 0x0118 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_JPEG_IMAGE")
    elseif idType == 0x0119 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_DIRECTED_STREAMS")
    elseif idType == 0x011a then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SENSOR_CAL")
    elseif idType == 0x011b then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_EXTERNAL_CAL")
    elseif idType == 0x011c then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_LED_SENSOR_STATUS")
    elseif idType == 0x011d then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_MOTOR_POLL")
    elseif idType == 0x011e then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_EXPOSURE_CONFIG")
    elseif idType == 0x011f then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_GROUND_SURFACE_SPLINE_DATA_MESSAGE")
    elseif idType == 0x0120 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_COMPRESSED_IMAGE")
    elseif idType == 0x0121 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_GROUND_SURFACE_PARAM")
    elseif idType == 0x0122 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_APRILTAG_DETECTIONS_MESSAGE")
    elseif idType == 0x0123 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "DATA_SYS_APRILTAG_PARAM")
    elseif idType == 0x0124 then
        payload = tree:add(multisense_protocol, tvb(18, length - 18), "CMD_REMOTE_HEAD_CONFIG")
    end

    if payload ~= nil then
        payload:add_le(multisense_payload_bytes, tvb(18, length - 18))
    end
end

local udp_port = DissectorTable.get("udp.port")
udp_port:add(9001, multisense_protocol)

