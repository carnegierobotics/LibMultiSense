/**
 * @file MultiSenseTypes.hh
 *
 * Copyright 2013-2025
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
 *   2024-12-24, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

#if !defined(MULTISENSE_API)
#if defined (_MSC_VER)
#define MULTISENSE_API __declspec(dllexport)
#else
#define MULTISENSE_API
#endif
#endif


namespace multisense
{

using TimeT = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

enum class Status : uint8_t
{
    UNKNOWN,
    OK,
    TIMEOUT,
    INTERNAL_ERROR,
    FAILED,
    UNSUPPORTED,
    EXCEPTION,
    UNINITIALIZED,
    INCOMPLETE_APPLICATION
};

///
/// @brief Identifies which camera or data source the image is from
///
enum class DataSource : uint16_t
{
    UNKNOWN,
    ALL,
    LEFT_MONO_RAW,
    RIGHT_MONO_RAW,
    LEFT_MONO_COMPRESSED,
    RIGHT_MONO_COMPRESSED,
    LEFT_RECTIFIED_RAW,
    RIGHT_RECTIFIED_RAW,
    LEFT_RECTIFIED_COMPRESSED,
    RIGHT_RECTIFIED_COMPRESSED,
    LEFT_DISPARITY_RAW,
    LEFT_DISPARITY_COMPRESSED,
    AUX_COMPRESSED,
    AUX_RECTIFIED_COMPRESSED,
    AUX_LUMA_RAW,
    AUX_LUMA_RECTIFIED_RAW,
    AUX_CHROMA_RAW,
    AUX_CHROMA_RECTIFIED_RAW,
    AUX_RAW,
    AUX_RECTIFIED_RAW,
    COST_RAW,
    IMU
};

struct CameraCalibration
{
    ///
    /// @brief Distortion type
    ///
    enum class DistortionType : uint8_t
    {
        NONE,
        PLUMBOB,
        RATIONAL_POLYNOMIAL
    };

    ///
    /// @brief Unrectified camera projection matrix stored in row-major ordering
    ///
    std::array<std::array<float, 3>, 3> K{{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};

    ///
    /// @brief Rotation matrix which takes points in the unrectified camera frame and transform
    ///        them in to the rectified coordinate frame
    ///
    std::array<std::array<float, 3>, 3> R{{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};

    ///
    /// @brief Rectified projection matrix which takes points in the origin camera coordinate
    ///        frame and projects them into the current camera
    ///
    std::array<std::array<float, 4>, 3> P{{{0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}}};

    ///
    /// @brief The type of the distortion model used for the unrectified camera
    ///
    DistortionType distortion_type = DistortionType::NONE;

    ///
    /// @brief Coefficients for the distortion model
    ///
    std::vector<float> D = {};
};

struct StereoCalibration
{
    ///
    /// @brief Calibration information for the left camera
    ///
    CameraCalibration left;

    ///
    /// @brief Calibration information for the right camera
    ///
    CameraCalibration right;

    ///
    /// @brief Calibration information for the aux camera (optional 3rd center camera)
    ///
    std::optional<CameraCalibration> aux = std::nullopt;
};

///
/// @brief Represents a single image plus metadata
///
struct Image
{
    ///
    /// @brief Pixel formats
    ///
    enum class PixelFormat : uint8_t
    {
        UNKNOWN,
        MONO8,
        BGR8,
        MONO16,
        FLOAT32,
        JPEG,
        H264
    };

    ///
    /// @brief A pointer to the raw image data sent from the camera
    ///
    std::shared_ptr<const std::vector<uint8_t>> raw_data = nullptr;

    ///
    /// @brief An offset into the raw_data pointer where the image data starts
    ///
    int64_t image_data_offset = 0;

    ///
    /// @brief The length of the image data after the image_data_offset has been applied
    ///
    size_t image_data_length = 0;

    ///
    /// @brief The format of the image data stored in the raw_data stored in the raw_data buffer
    ///
    PixelFormat format = PixelFormat::UNKNOWN;

    ///
    /// @brief Width of the image in pixels
    ///
    int width = -1;

    ///
    /// @brief Height of the image in pixels
    ///
    int height = -1;

    ///
    /// @brief The timestamp associated with the image based on the camera's clock. Starts at 0
    ///        on boot
    ///
    TimeT camera_timestamp{};

    ///
    /// @brief The timestamp associated with the image based using the camera's clock which is potentially PTP
    ///        synchronized with a remote PTP master
    ///
    TimeT ptp_timestamp{};

    ///
    /// @brief The camera data source which this image corresponds to
    ///
    DataSource source = DataSource::UNKNOWN;

    ///
    /// @brief The scaled calibration associated with the image
    ///
    CameraCalibration calibration{};

    ///
    /// @brief Get a pixel at a certain width/height location in the image.
    ///        NOTE: This check is slower since it checks to make sure the request is safe. It
    ///        should not be called repeatedly at high frequency
    ///
    template <typename T>
    std::optional<T> at(int w, int h) const
    {
        if (w < 0 || h < 0 || w >= width || h >= height ||
            raw_data == nullptr ||
            format == PixelFormat::UNKNOWN ||
            format == PixelFormat::JPEG ||
            format == PixelFormat::H264)
        {
            return std::nullopt;
        }

        if ((sizeof(T) == 8 && format == PixelFormat::MONO8) ||
            (sizeof(T) == 16 && format == PixelFormat::MONO16) ||
            (sizeof(T) == 24 && format == PixelFormat::BGR8) ||
            (sizeof(T) == 32 && format == PixelFormat::FLOAT32))
        {
            const size_t offset = sizeof(T) * ((width * h) + w);

            return *reinterpret_cast<const T*>(raw_data->data() + image_data_offset + offset);
        }

        return std::nullopt;
    }

#ifdef HAVE_OPENCV
    ///
    /// @brief Transform a image into a cv::Mat object if the user wants to build OpenCV utilities
    ///        The cv::Mat returned here wraps the underlying image data pointer associated with
    ///        the Image object. If the input Image object goes out of scope while you are still using
    ///        the corresponding cv::Mat, you will need to `clone` the cv::Mat creating an internal copy
    ///        of all the data
    ///
    cv::Mat cv_mat() const;
#endif
};

///
/// @brief A frame containing multiple images (indexed by DataSource).
///
struct ImageFrame
{
    ///
    /// @brief Add an image to the frame, keyed by the image's DataSource.
    ///
    void add_image(const Image& image)
    {
        images[image.source] = image;
    }

    ///
    /// @brief Retrieve image by DataSource. Throws if not found.
    ///
    const Image& get_image(const DataSource &source) const
    {
        auto it = images.find(source);
        if (it == images.end())
        {
            throw std::runtime_error("No image found for requested DataSource");
        }
        return it->second;
    }

    ///
    /// @brief Check if we have an image for a given data source
    ///
    bool has_image(const DataSource &source) const
    {
        return (images.find(source) != images.end());
    }

    ///
    /// @brief The unique monotonically increasing ID for each frame populated by the MultiSense
    ///
    int64_t frame_id = 0;

    ///
    /// @brief The images assocated with each source in the frame
    ///
    std::map<DataSource, Image> images;

    ///
    /// @brief The scaled calibration for the entire camera
    ///
    StereoCalibration calibration;

    ///
    /// @brief The MultiSense timestamp associated with the frame
    ///
    TimeT frame_time{};

    ///
    /// @brief The MultiSense ptp timestamp associated with the frame
    ///
    TimeT ptp_frame_time{};
};

///
/// @brief A single IMU sample from the camera
///
struct ImuSample
{
    ///
    /// @brief A generic measurement for a 3-axis IMU
    ///
    struct Measurement
    {
        ///
        /// @brief Measurement on the x-axis of the sensor
        ///
        float x = 0.0f;

        ///
        /// @brief Measurement on the y-axis of the sensor
        ///
        float y = 0.0f;

        ///
        /// @brief Measurement on the z-axis of the sensor
        ///
        float z = 0.0f;
    };

    ///
    /// @brief The acceleration in units of Gs. Depending on the IMU configuration of the sensor,
    ///        this may be invalid (i.e. the MultiSense has separate accelerometer and gyroscope chips)
    ///
    std::optional<Measurement> accelerometer = std::nullopt;
    ///
    /// @brief The rotational velocity in degrees-per-second. Depending on the IMU configuration of the sensor,
    ///        this may be invalid (i.e. the MultiSense has separate accelerometer and gyroscope chips)
    ///
    std::optional<Measurement> gyroscope = std::nullopt;
    ///
    /// @brief The measured magnetic field in milligauss. Depending on the IMU configuration of the sensor,
    ///        this may be invalid (i.e. the MultiSense has a separate magnetometer chip)
    ///
    std::optional<Measurement> magnetometer = std::nullopt;

    ///
    /// @brief The MultiSense timestamp associated with the frame
    ///
    TimeT sample_time{};

    ///
    /// @brief The MultiSense ptp timestamp associated with the frame
    ///
    TimeT ptp_sample_time{};
};

///
/// @brief A collection of IMU samples from the camera
///
struct ImuFrame
{
    ///
    /// @brief A batched collection of IMU samples
    ///
    std::vector<ImuSample> samples;
};

///
/// @brief A sample rate, and what impact it has on bandwidth
///
struct ImuRate
{
    ///
    /// @brief The sample rate for the sensor in Hz
    ///
    float sample_rate = 0.0f;

    ///
    /// @brief The bandwith cutoff for a given IMU mode in Hz
    ///
    float bandwith_cutoff = 0.0f;

    ///
    /// @brief Equality operator
    ///
    bool operator==(const ImuRate &rhs) const
    {
        return sample_rate == rhs.sample_rate && bandwith_cutoff == rhs.bandwith_cutoff;
    }
};

///
/// @brief The range for each sensor along with the corresponding sampling resolution
///
struct ImuRange
{
    ///
    /// @brief The max value the sensor can return. This value is +/- units for the given source
    ///
    float range = 0.0f;

    ///
    /// @brief The min resolution the sensor can return. In units per LSB
    ///
    float resolution = 0.0f;

    ///
    /// @brief Equality operator
    ///
    bool operator==(const ImuRange &rhs) const
    {
        return range == rhs.range && resolution == rhs.resolution;
    }
};


///
/// @brief Complete configuration object for configuring the MultiSense. Can be updated during camera operation
///
struct MultiSenseConfig
{
    ///
    /// @brief Stereo specific configuration
    ///
    struct StereoConfig
    {
        ///
        /// @brief This is used to filter low confidence stereo data before it is sent to the
        ///        host. Larger numbers indicate more aggressive filtering.
        ///        Valid range is [0, 1.0]
        ///
        float postfilter_strength = 0.85f;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const StereoConfig &rhs) const
        {
            return postfilter_strength == rhs.postfilter_strength;
        }
    };

    ///
    /// @brief Manual exposure specific configuration
    ///
    struct ManualExposureConfig
    {
        ///
        /// @brief The desired electrical and digital gain used to brighten the image.
        ///        Valid range is [1.6842, 16]
        ///
        float gain = 1.68f;

        ///
        /// @brief The manual exposure time in microseconds
        ///        Valid range is [0, 33000]
        ///
        std::chrono::microseconds exposure_time{10000};

        ///
        /// @brief Equality operator
        ///
        bool operator==(const ManualExposureConfig &rhs) const
        {
           return gain == rhs.gain && exposure_time == rhs.exposure_time;
        }
    };

    ///
    /// @brief Auto-exposure Region-of-Interest (ROI) specific configuration
    ///
    struct AutoExposureRoiConfig
    {
        ///
        /// @brief The x value of the top left corner of the ROI in the full resolution image. Note (0,0) is the top
        ///        left corner in the image coordinate frame.
        ///
        uint16_t top_left_x_position = 0;
        ///
        /// @brief The y value of the top left corner of the ROI in the full resolution image. Note (0,0) is the top
        ///        left corner in the image coordinate frame.
        ///
        uint16_t top_left_y_position = 0;
        ///
        /// @brief The width of the ROI in the full resolution image. A value of 0 disables the ROI
        ///
        uint16_t width = 0;
        ///
        /// @brief The height of the ROI in the full resolution image. A value of 0 disables the ROI
        ///
        uint16_t height = 0;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const AutoExposureRoiConfig &rhs) const
        {
            return top_left_x_position == rhs.top_left_x_position &&
                   top_left_y_position == rhs.top_left_y_position &&
                   width == rhs.width &&
                   height == rhs.height;
        }
    };

    ///
    /// @brief Auto-exposure specific configuration
    ///
    struct AutoExposureConfig
    {
        ///
        /// @brief The max exposure time auto exposure algorithm can set in microseconds
        ///        Valid range is [0, 33000]
        ///
        std::chrono::microseconds max_exposure_time{10000};

        ///
        /// @brief The desired auto-exposure decay rate. A larger value increases the number of frames it takes
        ///        for the current auto exposure desired exposure to apply.
        ///        Valid range is [1, 20]
        ///
        uint32_t decay = 3;

        ///
        /// @brief The auto-exposure algorithm in digital imaging endeavors to achieve a specified target intensity,
        ///        which is represented as a ratio. This ratio, when multiplied by the maximum potential pixel
        ///        brightness (255 for 8-bit images), produces the 'target auto-exposure pixel value'. The algorithm
        ///        then adjusts the exposure of the camera to ensure a given 'target threshold' percentage of
        ///        image pixels fall below this target intensity. This process maintains an ideal brightness balance,
        ///        preventing overexposure or underexposure.
        ///
        float target_intensity = 0.5f;

        ///
        /// @brief The fraction of pixels which must be equal or below the pixel value set by the target intensity
        ///        pixel value
        ///
        float target_threshold = 0.85f;

        ///
        /// @brief The auto exposure algorithm adjusts both exposure and gain. This caps the gain the auto exposure
        ///        algorithm can use
        ///
        float max_gain = 2.0f;

        ///
        /// @brief The auto exposure region-of-interest used to restrict the portion of the image which the
        ///        auto exposure algorithm is run on
        ///
        AutoExposureRoiConfig roi{};

        ///
        /// @brief Equality operator
        ///
        bool operator==(const AutoExposureConfig &rhs) const
        {
            return max_exposure_time == rhs.max_exposure_time &&
                   decay == rhs.decay &&
                   target_intensity == rhs.target_intensity &&
                   target_threshold == rhs.target_threshold &&
                   max_gain == rhs.max_gain &&
                   roi == rhs.roi;
        }
    };

    ///
    /// @brief Manual white balance specific configuration
    ///
    struct ManualWhiteBalanceConfig
    {
        ///
        /// @brief The manual red white-balance setting
        ///        Valid range is [0.25, 4]
        ///
        float red = 1.0f;

        ///
        /// @brief The manual blue white-balance setting
        ///        Valid range is [0.25, 4]
        ///
        float blue = 1.0f;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const ManualWhiteBalanceConfig &rhs) const
        {
            return red == rhs.red && blue == rhs.blue;
        }
    };

    ///
    /// @brief Auto white balance specific configuration
    ///
    struct AutoWhiteBalanceConfig
    {
        ///
        /// @brief The decay rate used for auto-white-balance
        ///        Valid range [0, 20]
        ///
        uint32_t decay = 3;

        ///
        /// @brief The auto white balance threshold
        ///        Valid range [0.0, 1.0]
        ///
        float threshold = 0.5f;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const AutoWhiteBalanceConfig &rhs) const
        {
            return decay == rhs.decay && threshold == rhs.threshold;
        }
    };

    ///
    /// @brief Image specific configuration
    ///
    struct ImageConfig
    {
        ///
        /// @brief Set the gamma correction for the image.
        ///        Valid range [1.0, 2.2]
        ///
        float gamma = 2.2f;

        ///
        /// @brief Enable or disable auto exposure
        ///
        bool auto_exposure_enabled = true;

        ///
        /// @brief The exposure config to use if auto exposure is disabled
        ///
        std::optional<ManualExposureConfig> manual_exposure = std::nullopt;

        ///
        /// @brief The exposure config to use if auto exposure is enabled
        ///
        std::optional<AutoExposureConfig> auto_exposure = std::nullopt;

        ///
        /// @brief Enable or disable auto white balance
        ///
        bool auto_white_balance_enabled = true;

        ///
        /// @brief The white balance parameters to use if auto white balance is disabled
        ///
        std::optional<ManualWhiteBalanceConfig> manual_white_balance = std::nullopt;

        ///
        /// @brief The white balance parameters to use if auto white balance is enabled
        ///
        std::optional<AutoWhiteBalanceConfig> auto_white_balance = std::nullopt;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const ImageConfig& rhs) const
        {
            return gamma == rhs.gamma &&
                   auto_exposure_enabled == rhs.auto_exposure_enabled &&
                   manual_exposure == rhs.manual_exposure &&
                   auto_exposure == rhs.auto_exposure &&
                   auto_white_balance_enabled == rhs.auto_white_balance_enabled &&
                   manual_white_balance == rhs.manual_white_balance &&
                   auto_white_balance == rhs.auto_white_balance;
        }
    };

    ///
    /// @brief Image specific configuration for the Aux imager
    ///
    struct AuxConfig
    {
        ///
        /// @brief Image configuration for the Aux imager
        ///
        ImageConfig image_config;

        ///
        /// @brief Enable sharpening
        ///
        bool sharpening_enabled = false;

        ///
        /// @brief The percentage strength of the sharpening gain to apply to the aux image
        ///        Valid range is [0, 100]
        ///
        float sharpening_percentage = 50.0f;

        ///
        ///
        ///  @brief The maximum difference in pixels that sharpening is
        ///         is allowed to change between neighboring pixels. This is useful for clamping
        ///         the sharpening percentage, while still maintaining a large gain.
        ///
        uint8_t sharpening_limit = 100;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const AuxConfig &rhs) const
        {
            return image_config == rhs.image_config &&
                   sharpening_enabled == rhs.sharpening_enabled &&
                   sharpening_percentage == rhs.sharpening_percentage &&
                   sharpening_limit == rhs.sharpening_limit;
        }
    };

    ///
    /// @brief Predefined disparity pixel search windows. Larger values allows the camera to see objects
    ///        closer to the camera
    ///
    enum class MaxDisparities : uint8_t
    {
        ///
        /// @brief 64 pixels
        ///
        D64,
        ///
        /// @brief 128 pixels
        ///
        D128,
        ///
        /// @brief 256 pixels
        ///
        D256
    };

    ///
    /// @brief Config for time-based controls
    ///
    struct TimeConfig
    {
        ///
        /// @brief Enable PTP sync on the camera
        ///
        bool ptp_enabled = false;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const TimeConfig &rhs) const
        {
           return ptp_enabled == rhs.ptp_enabled;
        }
    };

    ///
    /// @brief Config for transmitting packets from the MultiSense to the host
    ///
    struct NetworkTransmissionConfig
    {
        ///
        /// @brief Add a small delay between the transmission of each packet to hopefully interact
        ///        better with slower client machines, or more fragile networks
        ///
        bool packet_delay_enabled = false;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const NetworkTransmissionConfig &rhs) const
        {
            return packet_delay_enabled == rhs.packet_delay_enabled;
        }
    };

    ///
    /// @brief Config for the IMU sensor
    ///
    struct ImuConfig
    {
        ///
        /// @brief Config for a specific IMU operating mode. There are separate modes for each of the
        ///        IMU sensors (i.e. accelerometer, gyroscope, magnetometer)
        ///
        struct OperatingMode
        {
            ///
            /// @brief Enable the current source
            ///
            bool enabled = false;

            ///
            /// @brief The specific IMU rate configuration specified in ImuInfo::Source
            ///        table
            ///
            ImuRate rate{};

            ///
            /// @brief The specific IMU range configuration specified in ImuInfo::Source
            ///
            ImuRange range{};

            ///
            /// @brief Equality operator
            ///
            bool operator==(const OperatingMode &rhs) const
            {
                return enabled == rhs.enabled && rate == rhs.rate && range == rhs.range;
            }
        };

        ///
        /// @brief The number of IMU samples which should be included in a IMU frame
        ///
        uint32_t samples_per_frame = 0;

        ///
        /// @brief Configuration for the onboard accelerometer
        ///
        std::optional<OperatingMode> accelerometer = std::nullopt;

        ///
        /// @brief Configuration for the onboard gyroscope
        ///
        std::optional<OperatingMode> gyroscope = std::nullopt;

        ///
        /// @brief Configuration for the onboard magnetometer
        ///
        std::optional<OperatingMode> magnetometer = std::nullopt;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const ImuConfig &rhs) const
        {
            return samples_per_frame == rhs.samples_per_frame &&
                   accelerometer == rhs.accelerometer &&
                   gyroscope == rhs.gyroscope &&
                   magnetometer == rhs.magnetometer;
        }
    };

    ///
    /// @brief Lighting configuration for the camera
    ///
    struct LightingConfig
    {
        ///
        /// @brief Lighting config for lights integrated into the MultiSense
        ///
        struct InternalConfig
        {
            ///
            /// @brief Lighting brightness ranging from 0 to 100.0. A value of 0 will turn off the LEDs
            ///
            float intensity = 0.0f;

            ///
            /// @brief Enable flashing of the light
            ///
            bool flash = false;

            ///
            /// @brief Equality operator
            ///
            bool operator==(const InternalConfig &rhs) const
            {
                 return intensity == rhs.intensity && flash == rhs.flash;
            }
        };

        ///
        /// @brief Lighting config for lights driven by GPIO outputs from the MultiSense
        ///
        struct ExternalConfig
        {
            ///
            /// @brief Different flash modes for the camera
            ///
            enum class FlashMode : uint8_t
            {
                NONE,
                SYNC_WITH_MAIN_STEREO,
                SYNC_WITH_AUX
            };

            ///
            /// @brief Lighting brightness ranging from 0 to 100.0. A value of 0 will turn off the LEDs
            ///
            float intensity = 0.0f;

            ///
            /// @brief Configure flash mode
            ///
            FlashMode flash = FlashMode::NONE;

            ///
            /// @brief The number of pulses of the light per single exposure.
            ///        This is used to trigger the light or output signal multiple times after a
            ///        single exposure. For values greater than 1, pulses will occur between the
            ///        exposures, not during. This can be used to leverage human persistence of
            ///        vision to make the light appear as though it is not flashing
            ///
            uint32_t pulses_per_exposure = 1;

            ///
            /// @brief The time it takes for the light to reach full brightness. This will be used to
            ///        ensure the light is at full brightness when the image is exposed
            ///
            std::chrono::microseconds startup_time{0};

            ///
            /// @brief Equality operator
            ///
            bool operator==(const ExternalConfig &rhs) const
            {
                return intensity == rhs.intensity &&
                       flash == rhs.flash &&
                       pulses_per_exposure == rhs.pulses_per_exposure &&
                       startup_time == rhs.startup_time;
            }
        };

        ///
        /// @brief The internal lighting config. Will be nullopt if the camera does not
        ///        support internal lighting controls
        ///
        std::optional<InternalConfig> internal = std::nullopt;

        ///
        /// @brief The external lighting config. Will be nullopt if the camera does not
        ///        support internal lighting controls
        ///
        std::optional<ExternalConfig> external = std::nullopt;

        ///
        /// @brief Equality operator
        ///
        bool operator==(const LightingConfig &rhs) const
        {
             return internal == rhs.internal && external == rhs.external;
        }
    };

    ///
    /// @brief The operating width of the MultiSense in pixels. For available operating resolutions
    ///        see the MultiSenseInfo::SupportedOperatingMode
    ///
    uint32_t width = 0;

    ///
    /// @brief The operating height of the MultiSense in pixels. For available operating resolutions
    ///        see the MultiSenseInfo::SupportedOperatingMode
    ///
    uint32_t height = 0;

    ///
    /// @brief The max number of pixels the MultiSense searches when computing the disparity output
    ///
    MaxDisparities disparities = MaxDisparities::D256;

    ///
    /// @brief The target framerate the MultiSense should operate at
    ///
    float frames_per_second = 10.0f;

    ///
    /// @brief The stereo configuration to use
    ///
    StereoConfig stereo_config;

    ///
    /// @brief The image configuration to use for the main stereo pair
    ///
    ImageConfig image_config;

    ///
    /// @brief The image configuration to use for the aux camera if present
    ///
    std::optional<AuxConfig> aux_config = std::nullopt;

    ///
    /// @brief Config for the MultiSense time-sync options
    ///
    std::optional<TimeConfig> time_config = std::nullopt;

    ///
    /// @brief Config to control network transmission settings
    ///
    std::optional<NetworkTransmissionConfig> network_config = std::nullopt;

    ///
    /// @brief The imu configuration to use for the camera. Will be invalid if sensor does not contain an IMU
    ///
    std::optional<ImuConfig> imu_config = std::nullopt;

    ///
    /// @brief The lighting configuration for the camera. If invalid, the camera does not support lighting
    ///        configuration
    ///
    std::optional<LightingConfig> lighting_config = std::nullopt;

    ///
    /// @brief Equality operator
    ///
    bool operator==(const MultiSenseConfig &rhs) const
    {
        return width == rhs.width &&
               height == rhs.height &&
               disparities == rhs.disparities &&
               frames_per_second == rhs.frames_per_second &&
               stereo_config == rhs.stereo_config &&
               image_config == rhs.image_config &&
               aux_config == rhs.aux_config &&
               time_config == rhs.time_config &&
               network_config == rhs.network_config &&
               imu_config == rhs.imu_config &&
               lighting_config == rhs.lighting_config;
    }
};

///
/// @brief Consolidated status information which can be queried on demand from the MultiSense. Will change
///        during camera operation
///
struct MultiSenseStatus
{
    struct PtpStatus
    {
        ///
        /// @brief Status of the grandmaster clock. true if synchronized to a non-local grandmaster OR if
        ///        a non-local grandmaster was present any time during the current boot
        ///
        bool grandmaster_present = false;

        ///
        /// @brief The id of the current grandmaster clock (8 bytes, 0xXXXXXX.XXXX.XXXXXX)
        ///
        std::array<uint8_t, 8> grandmaster_id{0, 0, 0, 0, 0, 0, 0, 0};

        ///
        /// @brief Offset between the camera's PTP Hardware Clock and the grandmaster clock
        ///
        std::chrono::nanoseconds grandmaster_offset{0};

        ///
        /// @brief The estimate delay of the PTP synchronization messages from the grandmaster
        ///
        std::chrono::nanoseconds path_delay{0};

        ///
        /// @brief The number of network hops from the grandmaster to the camera's clock
        ///
        uint16_t steps_from_local_to_grandmaster = 0;
    };

    struct CameraStatus
    {
        ///
        /// @brief True if the cameras are operating and currently streaming data
        ///
        bool cameras_ok = false;

        ///
        /// @brief True if the onboard processing pipeline is ok and currently processing images
        ///
        bool processing_pipeline_ok = false;
    };

    struct TemperatureStatus
    {
        ///
        /// @brief Temperature of the FPGA  in Celsius
        ///
        float fpga_temperature = 0.0f;

        ///
        /// @brief Temperature of the left imager in Celsius
        ///
        float left_imager_temperature = 0.0f;

        ///
        /// @brief Temperature of the right imager in Celsius
        ///
        float right_imager_temperature = 0.0f;

        ///
        /// @brief Temperature of the internal switching power supply in Celsius
        ///
        float power_supply_temperature = 0.0f;
    };

    struct PowerStatus
    {
        ///
        /// @brief The current input voltage in volts
        ///
        float input_voltage = 0.0f;

        ///
        /// @brief The current input current in Amperes
        ///
        float input_current = 0.0f;

        ///
        /// @brief The current power draw of the FPGA in Watts
        ///
        float fpga_power = 0.0f;
    };

    struct ClientNetworkStatus
    {
        ///
        /// @brief The total number of valid messages received from the client
        ///
        size_t received_messages = 0;

        ///
        /// @brief The total number of dropped messages on the client side
        ///
        size_t dropped_messages = 0;

        ///
        /// @brief The total number of invalid packets received on the client side
        ///
        size_t invalid_packets = 0;
    };

    struct TimeStatus
    {
        ///
        /// @brief The camera's system time when the status message request was received
        ///
        std::chrono::nanoseconds camera_time{0};

        ///
        /// @brief The time of the host machine running the client when the status request was sent
        ///
        std::chrono::nanoseconds client_host_time{0};

        ///
        /// @brief The estimated network delay between when the status request was sent, and when the
        ///        status request was received. This is computed by measuring the time between when the client
        ///        machine sent the status request, and received the status response
        ///
        std::chrono::nanoseconds network_delay{0};

        ///
        /// @brief Compute the time offset which can manually be added to all camera timestamps to
        ///        change the camera timestamps from the reference clock of the camera into reference clock of the host
        ///
        std::chrono::nanoseconds offset_to_host() const
        {
            return (client_host_time + network_delay) - camera_time;
        }

        ///
        /// @brief Apply the time offset to a camera timestamps to camera timestamp from the reference clock of
        ///        the camera into reference clock of the host
        ///
        TimeT apply_offset_to_host(const TimeT &input_camera_time) const
        {
            return input_camera_time + std::chrono::duration_cast<std::chrono::system_clock::duration>(offset_to_host());
        }
    };

    ///
    /// @brief Summary of the current MultiSense state. True if the MultiSense is operating properly
    ///
    bool system_ok = false;

    ///
    /// @brief The current ptp status. Only valid if ptp is enabled
    ///
    std::optional<PtpStatus> ptp = std::nullopt;

    ///
    /// @brief The current camera status
    ///
    CameraStatus camera;

    ///
    /// @brief The current temperature status
    ///
    std::optional<TemperatureStatus> temperature = std::nullopt;

    ///
    /// @brief The current power status
    ///
    std::optional<PowerStatus> power = std::nullopt;

    ///
    /// @brief The current client network statistics
    ///
    ClientNetworkStatus client_network;

    ///
    /// @brief The current timing status information
    ///
    std::optional<TimeStatus> time = std::nullopt;
};

///
/// @brief Static status info for the MultiSense. Will not change during camera operation
///
struct MultiSenseInfo
{
    ///
    /// @brief The network configuration for the MultiSense
    ///
    struct NetworkInfo
    {
        ///
        /// @brief The ip address of the camera (i.e. X.X.X.X)
        ///
        std::string ip_address = "10.66.171.21";

        ///
        /// @brief The gateway of the camera (i.e. X.X.X.X)
        ///
        std::string gateway = "10.66.171.1";

        ///
        /// @brief The netmask of the camera (i.e. X.X.X.X)
        ///
        std::string netmask = "255.255.255.0";
    };

    ///
    /// @brief The Device information associated with the MultiSense. The DeviceInfo is used to determine what features
    ///        the MultiSense offers, and provides debug information to the Carnegie Robotics' team.
    ///        The DeviceInfo can only be set at the factory
    ///
    struct DeviceInfo
    {
        ///
        /// @brief Info for the PCBs contained in the unit
        ///
        struct PcbInfo
        {
            ///
            /// @brief The name of the PCB
            ///        This value can store at most 32 characters
            std::string name;
            ///
            /// @brief The revision number of the PCB
            ///
            uint32_t revision;
        };

        ///
        /// @brief MultiSense Hardware revisions
        ///
        enum class HardwareRevision : uint8_t
        {
            UNKNOWN,
            S7,
            S21,
            ST21,
            S27,
            S30,
            KS21,
            MONOCAM,
            KS21_SILVER,
            ST25,
            KS21i
        };

        ///
        /// @brief Different imager types
        ///
        enum class ImagerType : uint8_t
        {
            UNKNOWN,
            CMV2000_GREY,
            CMV2000_COLOR,
            CMV4000_GREY,
            CMV4000_COLOR,
            FLIR_TAU2,
            AR0234_GREY,
            AR0239_COLOR,
            TENUM1280
        };

        ///
        /// @brief MultiSense lighting types
        ///
        enum class LightingType : uint8_t
        {
            ///
            /// @brief No lights
            ///
            NONE,
            ///
            /// @brief Lights driven internally
            ///
            INTERNAL,
            ///
            /// @brief Drive lights via an external output
            ///
            EXTERNAL,
            ///
            /// @brief A pattern projector
            ///
            PATTERN_PROJECTOR
        };

        ///
        /// @brief MultiSense lens types
        ///
        enum class LensType : uint8_t
        {
            UNKNOWN,
            STANDARD,
            FISHEYE
        };

        ///
        /// @brief The name of the MultiSense variant.
        ///        This value can store at most 32 characters
        ///
        std::string camera_name{};

        ///
        /// @brief The date the MultiSense was manufactured
        ///        This value can store at most 32 characters
        ///
        std::string build_date{};

        ///
        /// @brief The unique serial number of the MultiSense
        ///        This value can store at most 32 characters
        ///
        std::string serial_number{};

        ///
        /// @brief The hardware revision of the MultiSense
        ///
        HardwareRevision hardware_revision{};

        ///
        /// @brief Information about each PCB
        ///
        std::vector<PcbInfo> pcb_info{};

        ///
        /// @brief The name of the imager used by the primary camera. For stereo cameras this is the
        ///        Left/Right stereo pair. For mono cameras this is the single imager
        ///        This value can store at most 32 characters
        ///
        std::string imager_name{};

        ///
        /// @brief The type of the imager
        ///
        ImagerType imager_type{};

        ///
        /// @brief The native width of the primary imager
        ///
        uint32_t imager_width = 0;

        ///
        /// @brief The native height of the primary imager
        ///
        uint32_t imager_height = 0;

        ///
        /// @brief The name of the lens used for the primary camera For stereo cameras this is the
        ///        Left/Right stereo pair. For mono cameras this is the single camera
        ///        This value can store at most 32 characters
        ///
        std::string lens_name{};

        ///
        /// @brief The type of the primary imager
        ///
        LensType lens_type{};

        ///
        /// @brief The nominal stereo baseline in meters
        ///
        float nominal_stereo_baseline = 0.0f;

        ///
        /// @brief The nominal focal length for the primary lens in meters
        ///
        float nominal_focal_length = 0.0f;

        ///
        /// @brief The nominal relative aperture for the primary camera modules in f-stop
        ///
        float nominal_relative_aperture = 0.0f;

        ///
        /// @brief The type of lighting used in the MultiSense
        ///
        LightingType lighting_type{};

        ///
        /// @brief The number of lights the MultiSense controls
        ///
        uint32_t number_of_lights = 0;

        ///
        /// @brief Determine if the MultiSense has a Aux color camera based on the DeviceInfo
        ///
        constexpr bool has_aux_camera() const
        {
            switch (hardware_revision)
            {
                case HardwareRevision::S27:
                case HardwareRevision::S30:
                case HardwareRevision::MONOCAM:
                case HardwareRevision::KS21i:
                    return true;
                default:
                    return false;
            }
        }
    };

    ///
    /// @brief Convenience wrapper for a version number
    ///        See https://semver.org/
    ///
    struct Version
    {
        ///
        /// @brief Major version number
        ///
        uint32_t major = 0;

        ///
        /// @brief Minor version number
        ///
        uint32_t minor = 0;

        ///
        /// @brief Patch version number
        ///
        uint32_t patch = 0;

        ///
        /// @brief Convert a Version info to string for convenience
        ///
        std::string to_string() const
        {
            return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
        }

        ///
        /// @brief Convenience operator for comparing versions
        ///
        bool operator<(const Version& other) const
        {
            return major < other.major ||
                   (major == other.major && minor < other.minor) ||
                   (major == other.major && minor == other.minor && patch < other.patch);
        }
    };

    ///
    /// @brief Version information for the MultiSense
    ///
    struct SensorVersion
    {
        ///
        /// @brief The date the firmware running on the camera was built
        ///
        std::string firmware_build_date{};

        ///
        /// @brief The version of the firmware running on the camera
        ///
        MultiSenseInfo::Version firmware_version{};

        ///
        /// @brief ID for the version of hardware
        ///
        uint64_t hardware_version = 0;
    };

    ///
    /// @brief A valid operating mode for the MultiSense
    ///
    struct SupportedOperatingMode
    {
        ///
        /// @brief The width of the output image in pixels
        ///
        uint32_t width = 0;

        ///
        /// @brief The height of the output image in pixels
        ///
        uint32_t height = 0;

        ///
        /// @brief Supported operating disparity
        ///
        MultiSenseConfig::MaxDisparities disparities = MultiSenseConfig::MaxDisparities::D256;

        ///
        /// @brief Data sources supported at that mode
        ///
        std::vector<DataSource> supported_sources{};
    };

    ///
    /// @brief Information about the IMU onboard the MultiSense
    ///
    struct ImuInfo
    {
        ///
        /// @brief Info about the available IMU configurations
        ///
        struct Source
        {
            ///
            /// @brief The name of the IMU sensor
            ///
            std::string name{};

            ///
            /// @brief The name of the IMU chip
            ///
            std::string device{};

            ///
            /// @brief The available rates supported by this operating mode
            ///
            std::vector<ImuRate> rates{};

            ///
            /// @brief The available ranges supported by this operating mode
            ///
            std::vector<ImuRange> ranges{};
        };

        ///
        /// @brief Configuration specific to the accelerometer
        ///
        std::optional<Source> accelerometer = std::nullopt;

        ///
        /// @brief Configuration specific to the gyroscope
        ///
        std::optional<Source> gyroscope = std::nullopt;

        ///
        /// @brief Configuration specific to the magnetometer
        ///
        std::optional<Source> magnetometer = std::nullopt;
    };

    ///
    /// @brief Device info
    ///
    DeviceInfo device;

    ///
    /// @brief Sensor Version info
    ///
    SensorVersion version;

    ///
    /// @brief Supported operating modes
    ///
    std::vector<SupportedOperatingMode> operating_modes;

    ///
    /// @brief Supported operating modes for the IMU sensors (accelerometer, gyroscope, magnetometer). Will
    ///        be invalid if an IMU is not present
    ///
    std::optional<ImuInfo> imu;

    ///
    /// @brief The network configuration of the MultiSense
    ///
    NetworkInfo network;
};

}
