/**
 * @file DpuTestUtility/DpuTestUtility.cc
 *
 * Copyright 2023
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
 *   2023-03-13, bblakeslee@carnegierobotics.com, 2033.1, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <bitset>
#include <map>
#include <vector>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// #include <torch/csrc/api/include/torch/torch.h>

#include <Utilities/portability/getopt/getopt.h>

using namespace crl::multisense;

std::map<int64_t, cv::Mat> luma_map;
std::map<int64_t, cv::Mat> chroma_map;

namespace {

    volatile bool quit_flag = false;

    void usage(const char *programNamePtr) {
        std::cerr << "USAGE: " << programNamePtr << " [<options>]" << std::endl;
        std::cerr << "Available <options>:" << std::endl;
        std::cerr << "\t-a <ip_address> : IPV4 address of camera (default=10.66.171.21)" << std::endl;
        std::cerr << "\t-m <mtu>        : Max packet size in bytes (default=7200)" << std::endl;

        exit(1);
    }

#ifdef WIN32
    BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
    CRL_UNUSED(dwCtrlType);
    std::cerr << "Shutting down on signal: CTRL-C" << std::endl;
    quit_flag = true;
    return TRUE;
}
#else

    void signalHandler(int sig) {
        std::cerr << "Shutting down on signal: " << strsignal(sig) << std::endl;
        quit_flag = true;
    }

#endif

    void dpuResultCallback(const dpu_result::Header &header, void *userDataPtr) {
        (void) userDataPtr;
        std::cout << "DPU Result Metadata:" << std::endl;
        std::cout << "  Frame ID: " << header.frameId << std::endl;
        std::cout << "  Time Stamp: " << header.timestamp << std::endl;
        std::cout << "  Success: " << uint16_t(header.success) << std::endl;
        // std::cout << "  Detection Count: " << uint16_t(header.numDetections) << std::endl;
        // std::cout << "  Sequence ID: " << uint16_t(header.sequenceId) << std::endl;
        std::cout << "  Result Type: " << header.resultType << std::endl;
        std::cout << "  Mask Blob Len:  " << header.maskBlobLen << std::endl;
        std::cout << "  Mask Rank:  " << header.maskRank << std::endl;
        std::cout << "  Mask Dims:  ";
        for (int i = 0; i < header.maskRank; i++) {
            std::cout << header.maskDims[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "DPU Result Data: " << std::endl;
        // std::cout << "  Class ID: " << int(header.classId) << std::endl;
        // std::cout << "  Confidence Score: " << header.confidenceScore << std::endl;
        /*
        std::cout << "  Bbox Data: ";
        for (uint32_t i = 0; i < 4; i++) {
            std::cout << header.bboxArray[i] << " ";
        }
        std::cout << std::endl;
        */
        uint32_t num_zeros = 0;
        uint32_t num_ones = 0;
        uint32_t num_twos = 0;
        std::cout << "  Mask Data: " << std::endl;
        for (uint32_t i = 0; i < header.maskBlobLen; i++) {
            if (header.maskArray[i] == 0) {
                num_zeros++;
            } else if (header.maskArray[i] == 1) {
                num_ones++;
            } else if (header.maskArray[i] == 2) {
                num_twos++;
            }
        }
        std::cout << "    Num Zeros = " << num_zeros << std::endl;
        std::cout << "    Num Ones = " << num_ones << std::endl;
        std::cout << "    Num Twos = " << num_twos << std::endl;
        std::cout << "    Mask Count Sum = " << num_zeros + num_ones + num_twos << std::endl;
        std::cout << "********************" << std::endl;
        cv::Mat mask = cv::Mat(160, 256, CV_8UC1, const_cast<uint8_t*>(header.maskArray)).clone();
        mask = mask * 100;
        cv::imwrite(std::to_string(header.frameId) + "_mask.png", mask);
    }

    void lumaImageCallback(const image::Header &header, void *userDataPtr) {
        (void) userDataPtr;

        cv::Mat luma = cv::Mat(header.height, header.width, CV_8UC1, const_cast<void*>(header.imageDataP)).clone();
        cv::imwrite(std::to_string(header.frameId) + "_luma.png", luma);
        // luma_map[header.frameId] = luma;
    }

    /*
    void chromaImageCallback(const image::Header &header, void *userDataPtr) {
        (void) userDataPtr;

        cv::Mat chroma = cv::Mat(header.height, header.width, CV_16UC1, const_cast<void*>(header.imageDataP)).clone();
        cv::imwrite(std::to_string(header.frameId) + "_chroma.png", chroma);
        // chroma_map[header.frameId] = chroma;
    }
    */

    void destroyChannel(Channel *channelPtr) {
        Channel::Destroy(channelPtr);
        exit(0);
    }

}   // Anonymous

/*
std::vector<torch::Tensor> convert_tensor_map_to_vector(std::map<std::string, torch::Tensor> tensor_map,
                                                        std::vector<std::string> key_order) {
    std::vector<torch::Tensor> tensor_vector;
    for (const auto &ko: key_order) {
        tensor_vector.push_back(tensor_map[ko]);
    }
    return tensor_vector;
}

void export_tensor_to_pytorch(const std::string &tensor_filename,
                              std::map<std::string, torch::Tensor> tensor_map,
                              std::vector<std::string> key_order) {
    std::vector<torch::Tensor> tensor_vector;
    if (tensor_map.empty()) {
        for (const auto &ko : key_order) {
            (void)ko;
            tensor_vector.push_back(torch::empty({0}));
        }
    } else {
        }
    } else {
        tensor_vector = convert_tensor_map_to_vector(tensor_map, key_order);
    }
    torch::save(tensor_vector, tensor_filename);
}
*/

int main(int argc, char** argv){
    std::string currentAddress = "10.66.171.21";
    // uint32_t mtu = 7200;
    bool dpuSupported = false;
    image::Config cfg;

    crl::multisense::CameraProfile profile = crl::multisense::User_Control;
    std::vector<system::DeviceMode> deviceModes;
    image::Config config;

#if WIN32
    SetConsoleCtrlHandler(signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    int arg;
    while(-1 != (arg = getopt(argc, argv, "a:m"))){
        switch(arg){
            case 'a':
                currentAddress = std::string(optarg);
                break;
            case 'm':
                // mtu = atoi(optarg);
                break;
            default:
                usage(*argv);
        }
    }

    // Camera communication initialization
    Channel *channelPtr = Channel::Create(currentAddress);
    if (NULL == channelPtr){
        std::cerr << "Failed to initialize communications with camera at " << currentAddress << std::endl;
        std::cerr << "Please check that IP address is valid." << std::endl;
        return -1;
    }

    // Fetch firmware version
    system::VersionInfo v;
    Status status = channelPtr->getVersionInfo(v);
    if(Status_Ok != status){
        std::cerr << "Failed to obtain sensor version: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    // Check to see if firmware supports DPU results
    status = channelPtr->getDeviceModes(deviceModes);
    if (Status_Ok != status) {
        std::cerr << "Failed to get device modes: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    dpuSupported = std::any_of(deviceModes.begin(), deviceModes.end(), [](const auto &mode) {
        return mode.supportedDataSources & Source_DPU_Result;});

    if (!dpuSupported) {
        std::cerr << "DPU results not supported with this firmware" << std::endl;
        destroyChannel(channelPtr);
    }

    // Shut down all streams
    // FIXME:  Stopping all streams results in the following error:
    //         00026.291 s19: Selected stream to remove does not exist. Port 16017
    //         The port number appears variable.
    status = channelPtr->stopStreams(Source_All);
    if (Status_Ok != status){
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    // Change MTU size
    // NOTE: This will fail if the MTU is set to over 1500 when using an
    // external USB-C to ethernet adapter.
    /*
    status = channelPtr->setMtu(mtu);
    if (Status_Ok != status) {
        std::cerr << "Failed to set MTU to " << mtu << ": " << Channel::statusString(status) << std::endl;
        std::cerr << "Check adapter to make sure it supports a frame size >1500 (Jumbo frames)" << std::endl;
        destroyChannel(channelPtr);
    }
    */

    // Enable DPU results profile
    status = channelPtr->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Reconfigure: Failed to query image config: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    profile |= crl::multisense::DpuResult;

    cfg.setCameraProfile(profile);
    // TODO: This is a hack.  Fix this for real.
    cfg.setGain(2.0);

    // FIXME:  This setImageConfig call results in a crash, hitting the error handler.
    //         This corresponds the following error in the S19 binary:
    //         00243.068 s19: Error: Invalid gain setting!
    //         This appears to be caused by the default gain of 1 being outside the
    //         values of 1.68421 to 16.  Need to find how to set this properly.
    status = channelPtr->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::cerr << "Reconfigure: failed to set image config: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    // Configure callbacks
    status = channelPtr->addIsolatedCallback(dpuResultCallback);
    if (Status_Ok != status) {
        std::cerr << "Failed to add DPU callback." << std::endl;
    }

    status = channelPtr->addIsolatedCallback(lumaImageCallback, Source_Luma_Rectified_Aux);
    if (Status_Ok != status) {
        std::cerr << "Failed to add luma image callback." << std::endl;
    }

    /*
    status = channelPtr->addIsolatedCallback(chromaImageCallback, Source_Chroma_Rectified_Aux);
    if (Status_Ok != status) {
        std::cerr << "Failed to add chroma image callback." << std::endl;
    }
    */

    // Start streaming
    status = channelPtr->startStreams(Source_DPU_Result | Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux);
    if (Status_Ok != status) {
        std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }

    while(!quit_flag){
        /*
        for (const auto &rm : reconstruction_map) {
            if (rm.second.size() == rm.second[0].numDetections) {
                std::vector<uint8_t> class_ids;
                std::vector<float> confidence_scores;
                std::vector<uint16_t> bboxes;
                std::vector<uint8_t> masks;
                for (const auto &rms : rm.second) {
                    class_ids.push_back(rms.classId);
                    confidence_scores.push_back(rms.confidenceScore);
                    for (int i = 0; i < 4; i++) {
                        bboxes.push_back(rms.bboxArray[i]);
                    }
                    for (uint32_t i = 0; i < rms.maskBlobLen; i++) {
                        masks.push_back(rms.maskArray[i]);
                    }
                }
                torch::TensorOptions class_options = torch::TensorOptions().dtype(torch::kUInt8).device(torch::kCPU);
                torch::Tensor class_id_tensor = torch::from_blob(
                        class_ids.data(), {int64_t(class_ids.size())}, class_options);
                torch::TensorOptions score_options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU);
                torch::Tensor score_tensor = torch::from_blob(
                        confidence_scores.data(), {int64_t(confidence_scores.size())}, score_options);
                torch::TensorOptions bbox_options = torch::TensorOptions().dtype(torch::kInt16).device(torch::kCPU);
                torch::Tensor bbox_tensor = torch::from_blob(
                        bboxes.data(), {int64_t(bboxes.size() / 4), 4}, bbox_options);
                torch::TensorOptions mask_options = torch::TensorOptions().dtype(torch::kUInt8).device(torch::kCPU);
                torch::Tensor mask_tensor = torch::from_blob(
                        masks.data(), {int64_t(class_ids.size()), 600, 960}, mask_options);
                int64_t fid = rm.second[0].frameId;
                cv::Mat matching_luma = luma_map[fid];
                cv::Mat matching_chroma = chroma_map[fid];
                cv::Mat rgb;
                try {
                    cv::cvtColorTwoPlane(matching_luma, matching_chroma, rgb, cv::COLOR_YUV2BGR_NV12);
                } catch (cv::Exception &e) {
                    break;
                std::map<std::string, torch::Tensor> export_map;
                export_map["class"] = class_id_tensor;
                export_map["score"] = score_tensor;
                export_map["box"] = bbox_tensor;
                export_map["mask"] = mask_tensor;
                export_tensor_to_pytorch(
                    "camera_test.pt", export_map, {"box", "class", "mask", "score"});
                reconstruction_map.erase(rm.first);
                for (const auto &lm : luma_map) {
                    if (lm.first <= fid) {
                        luma_map.erase(lm.first);
                    } else {
                        break;
                    }
                }
                for (const auto &cm : chroma_map) {
                    if (cm.first <= fid) {
                        chroma_map.erase(cm.first);
                    } else {
                        break;
                    }
                }
                break;
            }
        }
        */
        usleep(100000);
    }

    // Stop stream
    status = channelPtr->stopStreams(Source_All);
    if (Status_Ok != status){
        std::cerr << "Failed to stop streams: " << Channel::statusString(status) << std::endl;
        destroyChannel(channelPtr);
    }
}
