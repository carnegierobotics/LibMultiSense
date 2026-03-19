/**
 * @file feature_test.cc
 *
 * Copyright 2024-2025
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
 **/

#include <gtest/gtest.h>

#include <MultiSense/MultiSenseTypes.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include <utility/BufferStream.hh>
#include <wire/FeatureMessage.hh>
#include <wire/FeatureMetaMessage.hh>

#ifdef HAVE_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#endif

using namespace multisense;

TEST(FeatureMessage, Basic)
{
    FeatureMessage msg;
    msg.source = DataSource::LEFT_ORB_FEATURES;
    msg.descriptor_type = DescriptorType::ORB;

    KeyPoint kp1;
    kp1.x = 10.5f;
    kp1.y = 20.2f;
    kp1.response = 100.0f;
    kp1.octave = 1;
    kp1.class_id = 5;

    msg.keypoints.push_back(kp1);

    // ORB descriptor is 32 bytes
    std::vector<uint8_t> desc(32, 0xAA);
    msg.descriptors.insert(msg.descriptors.end(), desc.begin(), desc.end());

    EXPECT_EQ(msg.source, DataSource::LEFT_ORB_FEATURES);
    EXPECT_EQ(msg.descriptor_type, DescriptorType::ORB);
    EXPECT_EQ(msg.keypoints.size(), 1);
    EXPECT_EQ(msg.descriptors.size(), 32);
    EXPECT_FLOAT_EQ(msg.keypoints[0].x, 10.5f);
}

TEST(ImageFrame, Features)
{
    ImageFrame frame;
    frame.frame_id = 1234;

    FeatureMessage msg;
    msg.source = DataSource::LEFT_ORB_FEATURES;
    msg.keypoints.resize(10);

    frame.add_feature(msg);

    EXPECT_TRUE(frame.has_feature(DataSource::LEFT_ORB_FEATURES));
    EXPECT_FALSE(frame.has_feature(DataSource::RIGHT_ORB_FEATURES));
    EXPECT_EQ(frame.get_feature(DataSource::LEFT_ORB_FEATURES).keypoints.size(), 10);
    EXPECT_EQ(frame.frame_id, 1234);
}

#ifdef HAVE_OPENCV
TEST(FeatureMessage, OpenCVConversion)
{
    FeatureMessage msg;
    msg.source = DataSource::LEFT_ORB_FEATURES;
    msg.descriptor_type = DescriptorType::ORB;

    for (int i = 0; i < 5; ++i)
    {
        KeyPoint kp;
        kp.x = static_cast<float>(i * 10);
        kp.y = static_cast<float>(i * 20);
        kp.angle = 45.0f;
        kp.response = 50.0f;
        kp.octave = 0;
        kp.class_id = i;
        msg.keypoints.push_back(kp);
    }

    // 5 features, 32 bytes each = 160 bytes
    msg.descriptors.resize(5 * 32, 0xBB);

    auto cv_kp = msg.cv_keypoints();
    auto cv_desc = msg.cv_descriptors();

    ASSERT_EQ(cv_kp.size(), 5);
    EXPECT_FLOAT_EQ(cv_kp[0].pt.x, 0.0f);
    EXPECT_FLOAT_EQ(cv_kp[4].pt.x, 40.0f);
    EXPECT_FLOAT_EQ(cv_kp[2].angle, 45.0f);
    EXPECT_EQ(cv_kp[3].class_id, 3);

    ASSERT_EQ(cv_desc.rows, 5);
    ASSERT_EQ(cv_desc.cols, 32);
    ASSERT_EQ(cv_desc.type(), CV_8UC1);
    EXPECT_EQ(cv_desc.at<uint8_t>(0, 0), 0xBB);
}
#endif

TEST(FeatureWire, Serialization)
{
    using namespace crl::multisense::details;

    // Simulate FeatureDetectorMeta
    wire::FeatureDetectorMeta meta;
    meta.frameId = 555;
    meta.timeSeconds = 100;
    meta.timeNanoSeconds = 200;
    meta.ptpNanoSeconds = 300;
    meta.numFeatures = 2;
    meta.numDescriptors = 2;

    std::vector<uint8_t> meta_buf(512);
    utility::BufferStreamWriter meta_writer(meta_buf.data(), meta_buf.size());
    meta.serialize(meta_writer, wire::FeatureDetectorMeta::VERSION);

    // Simulate FeatureDetector data
    std::vector<uint8_t> data_buf(1024);
    utility::BufferStreamWriter data_writer(data_buf.data(), data_buf.size());

    // FeatureDetector has a specialized serialize that includes writing raw data
    // In our implementation it just writes the header then seeks past data
    // So we manually write the data after the header
    data_writer & wire::FeatureDetectorHeader::VERSION; // version
    data_writer & (uint64_t)wire::SOURCE_SECONDARY_APP_DATA_0; // source
    data_writer & (int64_t)555; // frameId
    data_writer & (uint16_t)2; // numFeatures
    data_writer & (uint16_t)2; // numDescriptors

    wire::Feature f1{10, 20, 30, 40, 50, 60};
    wire::Feature f2{100, 200, 30, 40, 50, 60};
    data_writer.write(&f1, sizeof(f1));
    data_writer.write(&f2, sizeof(f2));

    wire::Descriptor d1{{1,2,3,4,5,6,7,8}};
    wire::Descriptor d2{{10,20,30,40,50,60,70,80}};
    data_writer.write(&d1, sizeof(d1));
    data_writer.write(&d2, sizeof(d2));

    // Now deserialize
    utility::BufferStreamReader meta_reader(meta_buf.data(), meta_buf.size());
    wire::FeatureDetectorMeta meta_decoded(meta_reader, wire::FeatureDetectorMeta::VERSION);
    EXPECT_EQ(meta_decoded.frameId, 555);
    EXPECT_EQ(meta_decoded.numFeatures, 2);

    utility::BufferStreamReader data_reader(data_buf.data(), data_buf.size());
    wire::FeatureDetector data_decoded(data_reader, wire::FeatureDetector::VERSION);
    EXPECT_EQ(data_decoded.frameId, 555);
    EXPECT_EQ(data_decoded.numFeatures, 2);

    // Verify raw data pointer
    wire::Feature* features_ptr = reinterpret_cast<wire::Feature*>(data_decoded.dataP);
    EXPECT_EQ(features_ptr[0].x, 10);
    EXPECT_EQ(features_ptr[1].x, 100);

    wire::Descriptor* desc_ptr = reinterpret_cast<wire::Descriptor*>(reinterpret_cast<uint8_t*>(data_decoded.dataP) + 2*sizeof(wire::Feature));
    EXPECT_EQ(desc_ptr[0].d[0], 1);
    EXPECT_EQ(desc_ptr[1].d[0], 10);
}

TEST(FeatureConfig, Basic)
{
    FeatureDetectorConfig config;
    config.number_of_features = 2000;
    config.grouping_enabled = false;
    config.motion_octave = 2;

    EXPECT_EQ(config.number_of_features, 2000);
    EXPECT_FALSE(config.grouping_enabled);
    EXPECT_EQ(config.motion_octave, 2);
}


