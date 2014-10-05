/**
 * @file LibMultiSense/MultiSenseChannel.hh
 *
 * Copyright 2013
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
 *   2013-04-25, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_MultiSenseChannel_hh
#define LibMultiSense_MultiSenseChannel_hh

#include <stdint.h>
#include <string>
#include <vector>

#include "MultiSenseTypes.hh"

namespace crl {
namespace multisense {

/**
 * Class which manages all communications with a MultiSense device.
 *
 * Example code to instantiate and destroy a Channel
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     //
 *     // Query and send data to the sensor...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 */
class Channel {
public:

    /**
     * Create a Channel instance, used to manage all communications
     * with a sensor.  The resulting pointer must be explicitly
     * destroyed using the static member function Channel::Destroy().
     *
     * @param sensorAddress The device IPv4 address which can be a dotted-quad,
     * or any hostname resolvable by gethostbyname().
     *
     * return A pointer to a new Channel instance
     */

    static Channel* Create(const std::string& sensorAddress);

    /**
     * Destroy a channel instance that was created using the static
     * member function Channel::Create(). This operation should be
     * done before exiting
     *
     * @param instanceP A pointer to a Channel instance to be destroyed
     */

    static void Destroy(Channel *instanceP);

    /**
     * Destructor.
     */
    virtual ~Channel() {};

    /**
     * Helper method used to get a string describing a specific status code
     *
     * @param status A status code to convert to a string
     *
     * return A char* corresponding to the input status code
     */

    static const char *statusString(Status status);


    /**
     * Add a user defined callback attached to the image streams specified by
     * imageSourceMask. Each callback will create a unique internal thread
     * dedicated to servicing the callback.
     *
     * The image data passed to the callback are valid only until the
     * callback returns. To reserve image data so that it persists
     * longer than that, see member function reserveCallbackBuffer() below.
     *
     * Image data is queued per-callback. For each image callback the
     * internal queue depth is 5 images, but see member functions
     * getLargeBufferDetails() and setLargeBuffers().
     *
     * Adding multiple callbacks subscribing to the same image data is allowed.
     * The same instance of image data will be presented to each callback with
     * no copying done.
     *
     * Multiple image types may be subscribed to simultaneously in a
     * single callback by using an imageSourceMask argument that is
     * the bitwise or of the appropriate DataSource values.
     *
     * Multiple callbacks of differing types may be added in order to isolate
     * image processing by thread.
     *
     * @param callback A user defined image::Callback to which image
     * data will be sent.
     *
     * @param imageSourceMask The specific image types to stream to the user
     * defined callback. Multiple image sources may be combined using the
     * bitwise OR operator.
     *
     * @param userDataP A pointer to arbitrary user data. This typically
     * is a pointer to the instance of the object where the callback is defined.
     * See image::Header for a example of this usage
     *
     * @return A crl::multisense::Status indicating if the callback registration
     * succeeded or failed
     *
     */
    virtual Status addIsolatedCallback(image::Callback callback,
                                       DataSource      imageSourceMask,
                                       void           *userDataP=NULL) = 0;

    /**
     * Add a user defined callback attached to a lidar stream.
     * Each callback will create a unique internal thread
     * dedicated to servicing the callback.
     *
     * The lidar data passed to the callback are valid only until the
     * callback returns. To reserve image data so that it persists
     * longer than that, see member function reserveCallbackBuffer() below.
     *
     * Lidar data is queued per-callback. For each lidar callback the maximum
     * queue depth is 20 lidar scans.
     *
     * Adding multiple callbacks subscribing to the same lidar data is allowed.
     * The same instance of lidar data will be presented to each callback with
     * no copying done.
     *
     * @param callback A user defined lidar::Callback to send lidar data
     * to
     *
     * @param userDataP A pointer to arbitrary user data.
     *
     * @return A crl::multisense::Status indicating if the callback registration
     * succeeded or failed
     */
    virtual Status addIsolatedCallback(lidar::Callback callback,
                                       void           *userDataP=NULL) = 0;

    /**
     * Add a user defined callback attached to a pulse per-second (PPS) stream.
     * This PPS stream corresponds to the pulse on the OPTO-TX line.
     *
     * Each callback will create a unique internal thread
     * dedicated to servicing the callback.
     *
     * PPS data is queued per-callback. For each PPS callback the maximum
     * queue depth is 2 pps events.
     *
     * Adding multiple callbacks subscribing to the same PPS data is allowed.
     *
     * PPS data is stored on the heap and released after returning from
     * the callback
     *
     * @param callback A user defined pps::Callback to send PPS data
     * to
     *
     * @param userDataP A pointer to arbitrary user data.
     *
     * @return A crl::multisense::Status indicating if the callback registration
     * succeeded or failed
     */
    virtual Status addIsolatedCallback(pps::Callback   callback,
                                       void           *userDataP=NULL) = 0;

    /**
     * Add a user defined callback attached to the IMU stream.
     *
     * Each callback will create a unique internal thread
     * dedicated to servicing the callback.
     *
     * IMU data is queued per-callback. For each IMU callback the maximum
     * queue depth is 50 IMU messages.
     *
     * Adding multiple callbacks subscribing to the same IMU data is allowed.
     *
     * IMU data is stored on the heap and released after returning from
     * the callback
     *
     * Each imu::Header contains multiple IMU samples to reduce the number
     * of UDP packets sent from the sensor.
     *
     * @param callback A user defined imu::Callback to send PPS data
     * to
     *
     * @param userDataP A pointer to arbitrary user data.
     *
     * @return A crl::multisense::Status indicating if the callback registration
     * succeeded or failed
     */
    virtual Status addIsolatedCallback(imu::Callback   callback,
                                       void           *userDataP=NULL) = 0;


    /**
     * Unregister a user defined image::Callback. This stops the callback
     * from receiving image data.
     *
     * @param callback The user defined imu::Callback to unregister
     *
     * @return A crl::multisense::Status indicating if the callback deregistration
     * succeeded or failed
     */

    virtual Status removeIsolatedCallback(image::Callback callback) = 0;

    /**
     * Unregister a user defined lidar::Callback. This stops the callback
     * from receiving lidar data
     *
     * @param callback The user defined image::Callback to unregister
     *
     * @return A crl::multisense::Status indicating if the callback deregistration
     * succeeded or failed
     */

    virtual Status removeIsolatedCallback(lidar::Callback callback) = 0;

    /**
     * Unregister a user defined pps::Callback. This stops the callback
     * from receiving pps data
     *
     * @param callback The user defined lidar::Callback to unregister
     *
     * @return A crl::multisense::Status indicating if the callback deregistration
     * succeeded or failed
     */

    virtual Status removeIsolatedCallback(pps::Callback   callback) = 0;

    /**
     * Unregister a user defined imu::Callback. This stops the callback
     * from receiving imu data
     *
     * @param callback The user defined pps::Callback to unregister
     *
     * @return A crl::multisense::Status indicating if the callback deregistration
     * succeeded or failed
     */

    virtual Status removeIsolatedCallback(imu::Callback   callback) = 0;

    /**
     * Reserve image or lidar data within a isolated callback so it is available
     * after the callback returns.
     *
     * The memory buffer behind an image or lidar datum within an isolated callback
     * may be reserved by the user.  This is useful for performing data
     * processing outside of the channel callback, without having to perform
     * a memory copy of the sensor data.
     *
     * The channel is guaranteed not to reuse the memory buffer until the
     * user releases it back.  Note that there are a limited amount of memory
     * buffers, and care should be taken not to reserve them for too long.
     *
     * @return A valid (non NULL) reference only
     * when called within the context of an image or lidar callback.
     *
     */

    virtual void  *reserveCallbackBuffer()                 = 0;

    /**
     * Release image or lidar data reserved within a isolated callback.
     *
     * releaseCallbackBuffer() may be called from any thread context.
     *
     * @param referenceP A pointer to the reserved data returned by
     * reserveCallbackBuffer()
     *
     * @return A crl::multisense::Status indicating if the callback data
     * buffer was successfully released
     */

    virtual Status releaseCallbackBuffer(void *referenceP) = 0;

    /**
     * Enable or disable local network-based time synchronization.
     *
     * Each Channel will keep a continuously updating and filtered offset between
     * the sensor's internal clock and the local system clock.
     *
     * If enabled, all sensor timestamps will be reported in the local system
     * clock frame, after the offset has been applied.
     *
     * If disabled, all sensor timestamps will be reported in the frame of the
     * sensor's clock, which is free-running from 0 seconds on power up.
     *
     * The network-based time synchronization is enabled by default.
     *
     * @param enabled A boolean flag which enables or disables network time
     * synchronization
     *
     * @return A crl::multisense::Status indicating if the network-based time
     * synchronization was successfully enabled or disabled
     */

    virtual Status networkTimeSynchronization(bool enabled) = 0;

    /**
     * Start streaming various DataSources from the sensor.
     *
     * This is the primary means of stream control. All streams will come to
     * the requestor (i.e., the machine making the request with this API. The
     * server peeks the source address and port from the request UDP packet.)
     *
     * Multiple streams may be started at once by setting the individual source bits
     * in the mask.  Different streams may be started at differing times
     * by calling startStreams() multiple times.
     *
     * @param mask The DataSources to start streaming. Multiple streams can
     * be started by performing a bitwise OR operation with various
     * crl::multisense::DataSource definitions
     *
     * @return A crl::multisense::Status indicating if the specified streams were
     * successfully started
     */

    virtual Status startStreams     (DataSource mask)  = 0;

    /**
     * Stop specific data streams from the sensor.
     *
     * Stop streams may be called to selectively disable streams at any time.
     *  (use stopStreams(crl::multisense::Source_All) to disable all streaming.
     *
     * @param mask The DataSources to stop streaming. Multiple streams can
     * be stopped by performing a bitwise OR operation with various
     * crl::multisense::DataSource definitions
     *
     * @return A crl::multisense::Status indicating if the specified streams were
     * successfully stopped
     */

    virtual Status stopStreams      (DataSource mask)  = 0;

    /**
     * Get the current data streams which are enabled on the sensor.
     *
     * @param mask The current data streams which are enabled. These are returned
     * by reference in mask
     *
     * @return A crl::multisense::Status indicating if the getEnabledStreams
     * query succeeded.
     */

    virtual Status getEnabledStreams(DataSource& mask) = 0;


    /**
     * Start a directed stream used to stream data to multiple 3rd parties.
     *
     * NOTE: Directed streams are currently only supported by CRL's
     *       Monocular IP Camera.  MultiSense S* hardware variations
     *       are not supported (the following functions will return
     *       the 'Status_Unknown' error code.)
     *
     * Secondary stream control.  In addition to the primary stream control
     * above, a set of streams may be directed to a 3rd party (or multiple
     * 3rd parties), where a 3rd party is uniquely defined as an IP address /
     * UDP port pair.
     *
     * The number of simulataneous parties supported can be queried via
     * maxDirectedStreams(). If the number of maximum directed streams has
     * already been achieved, startDirectedStream() will return an error code.
     *
     * If the stream address/port combination already exists as a streaming
     * destination, then startDirectedStream() will update the data source
     * mask and FPS decimation.
     *
     * @param stream A DrirectedStream to either add or update setting of
     *
     * @return A crl::multisense::Status indicating if the DirectedStream was
     * successfully started
     */

    virtual Status startDirectedStream(const DirectedStream& stream) = 0;

    /**
     * Stop a specific directed stream.
     *
     * NOTE: Directed streams are currently only supported by CRL's
     *       Monocular IP Camera.  MultiSense S* hardware variations
     *       are not supported (the following functions will return
     *       the 'Status_Unknown' error code.)
     *
     * @param stream A DirectedStream to stop
     *
     * @return A crl::multisense::Status indicating if the DirectedStream was
     * successfully stopped
     */

    virtual Status stopDirectedStream (const DirectedStream& stream) = 0;

    /**
     * Get the current list of active streams.
     *
     * NOTE: Directed streams are currently only supported by CRL's
     *       Monocular IP Camera.  MultiSense S* hardware variations
     *       are not supported (the following functions will return
     *       the 'Status_Unknown' error code.)
     *
     * @param streams A vector of DirectedStreams which is populated by
     * getDirectedStreams()
     *
     * @return A crl::multisense::Status indicating if the DirectedStream query
     * succeeded
     */

    virtual Status getDirectedStreams (std::vector<DirectedStream>& streams) = 0;

    /**
     * Query the number of simultaneous parties which can be supported.
     *
     * NOTE: Directed streams are currently only supported by CRL's
     *       Monocular IP Camera.  MultiSense S* hardware variations
     *       are not supported (the following functions will return
     *       the 'Status_Unknown' error code.)
     *
     * If the number of maximum directed streams has
     * already been achieved, startDirectedStream() will return an error code.
     *
     * @param maximum The maximum number of DirectedStreams returned by reference
     *
     * @return A crl::multisense::Status indicating if the query
     * succeeded
     */

    virtual Status maxDirectedStreams (uint32_t& maximum) = 0;




    /**
     * Set a new image trigger source. This is used to specify which source
     * is used to trigger a image capture.
     *
     * By default Trigger_Internal is used
     *
     * @param s The new desired trigger source
     *
     * @return A crl::multisense::Status indicating if the trigger source
     * change succeeded
     */

    virtual Status setTriggerSource    (TriggerSource s)                    = 0;

    /**
     * Set the laser spindle rotation speed. A positive value rotates the laser
     * in the counter-clockwise direction. A negative value rotates the laser
     * in the clockwise direction.
     *
     * NOTE: Only positive rotations are recommended. Full life-cycle testing has
     * only been done using a positive rotation.
     *
     * @param rpm The number of rotations per minute [-50.0, 50.0]
     *
     * @return A crl::multisense::Status indicating if the motor speed
     * change succeeded
     */

    virtual Status setMotorSpeed       (float rpm)                          = 0;

    /**
     * Query the on-board lighting configuration.
     *
     * See lighting::Config for a usage example
     *
     * @param c A lighting configuration returned by reference
     *
     * @return A crl::multisense::Status indicating if the lighting configuration
     * query succeeded
     */

    virtual Status getLightingConfig   (lighting::Config& c)                = 0;

    /**
     * Set the on-board lighting configuration.
     *
     * See lighting::Config for a usage example
     *
     * @param c A lighting configuration to send to the sensor
     *
     * @return A crl::multisense::Status indicating if the lighting reconfiguration
     *  succeeded
     */

    virtual Status setLightingConfig   (const lighting::Config& c)          = 0;

    /**
     * Get the API version of the sensor firmware.
     *
     * @param version The API version of the sensor firmware, returned by reference
     *
     * @return A crl::multisense::Status indicating if the sensor API version
     * query succeeded
     */

    virtual Status getSensorVersion    (VersionType& version)               = 0;

    /**
     * Get the API version of LibMultiSense.
     *
     * @param version The API version of LibMultiSense, returned by reference
     *
     * @return A crl::multisense::Status indicating if the LibMultiSense API
     * version query succeeded
     */

    virtual Status getApiVersion       (VersionType& version)               = 0;

    /**
     * Get the version info for the sensor and LibMultiSense.
     *
     * See system::VersionInfo for a usage example.
     *
     * @param v The version information returned by reference
     *
     * @return A crl::multisense::Status indicating if the version information
     * query succeeded
     */

    virtual Status getVersionInfo      (system::VersionInfo& v)             = 0;

    /**
     * Query the image configuration.
     *
     * See image::Config for a usage example
     *
     * @param c The image configuration returned by reference from the query
     *
     * @return A crl::multisense::Status indicating if the image configuration
     * query succeeded
     */

    virtual Status getImageConfig      (image::Config& c)                   = 0;

    /**
     * Set the image configuration.
     *
     * See image::Config for a usage example
     *
     * @param c The new image configuration to send to the sensor
     *
     * @return A crl::multisense::Status indicating if the image configuration
     * was successfully received by the sensor
     */

    virtual Status setImageConfig      (const image::Config& c)             = 0;

    /**
     * Query the current camera calibration.
     *
     * See image::Calibration for a usage example
     *
     * @param c A image calibration returned by reference from
     * getImageCalibration()
     *
     * @return A crl::multisense::Status indicating if the image calibration
     * was successfully queried
     */

    virtual Status getImageCalibration (image::Calibration& c)              = 0;

    /**
     * Set the current camera calibration.
     *
     * See image::Calibration for a usage example
     *
     * @param c A new image calibration to send to the sensor
     *
     * @return A crl::multisense::Status indicating if the image calibration
     * was successfully received by the sensor
     */

    virtual Status setImageCalibration (const image::Calibration& c)        = 0;

    /**
     * Query the current laser calibration.
     *
     * See lidar::Calibration for a usage example
     *
     * @param c A laser calibration returned by reference from
     * getLidarCalibration()
     *
     * @return A crl::multisense::Status indicating if the laser calibration
     * was successfully queried
     */

    virtual Status getLidarCalibration (lidar::Calibration& c)              = 0;

    /**
     * Set the current laser calibration.
     *
     * See lidar::Calibration for a usage example
     *
     * @param c A new laser calibration to send to the sensor
     *
     * @return A crl::multisense::Status indicating if the laser calibration
     * was successfully received by the sensor
     */

    virtual Status setLidarCalibration (const lidar::Calibration& c)        = 0;

    /**
     * Get the image histogram for the image corresponding to a specified
     * frameId.
     *
     * See image::Histogram for a usage example
     *
     * @param frameId The frameId of the corresponding left image to query a
     * histogram for. Histograms can only be queried for images with frameIds
     * fewer than 20 frameIds from the most recent image's frameId.
     *
     * @param histogram A histogram returned by reference
     *
     * @return A crl::multisense::Status indicating if the histogram query
     * was successful
     */

    virtual Status getImageHistogram   (int64_t frameId,  // from last 20 images, left only
                                        image::Histogram& histogram)        = 0;


    /**
     * Query the available sensor device modes.
     *
     * See system::DeviceMode for a usage example
     *
     * @param m A vector of possible device modes returned by reference from
     * the sensor
     *
     * @return A crl::multisense::Status indicating if the device mode query
     * was successful
     */

    virtual Status getDeviceModes      (std::vector<system::DeviceMode>& m) = 0;

    /**
     * Query the current sensor's MTU configuration.  The MTU setting
     * controls the maximum size of the UDP packets that will be
     * transmitted from the sensor via Ethernet.  In general, larger
     * MTU settings are preferred, but the MTU value must not exceed
     * the MTU setting of the network interface to which the
     * MultiSense unit is sending data.
     *
     * @param mtu An int32_t returned by reference containing the current MTU
     * setting
     *
     * @return A crl::multisense::Status indicating if the mtu query
     * was successful
     */

    virtual Status getMtu              (int32_t& mtu)                       = 0;

    /**
     * Set the current sensor's MTU.
     *
     * @param mtu The new MTU to configure the sensor with
     *
     * @return A crl::multisense::Status indicating if the mtu configuration
     * was successfully received
     */

    virtual Status setMtu              (int32_t mtu)                        = 0;

    /**
     * Query the current sensor's network configuration.
     *
     * See system::NetworkConfig for a usage example
     *
     * @param c A NetworkConfig returned by reference containing the current
     * sensor's network configuration
     *
     * @return A crl::multisense::Status indicating if the network configuration
     * query was successful
     */

    virtual Status getNetworkConfig    (system::NetworkConfig& c)           = 0;

    /**
     * Set the current sensor's network configuration.
     *
     * See system::NetworkConfig for a usage example
     *
     * @param c A new network configuration to send to the sensor
     *
     * @return A crl::multisense::Status indicating if the network configuration
     * was successfully received
     */
    virtual Status setNetworkConfig    (const system::NetworkConfig& c)     = 0;

    /**
     * Query the current sensor's device information.
     *
     * See system::DeviceInfo for a usage example
     *
     * @param info A DeviceInfo returned by reference containing the
     * device information for the current sensor
     *
     * @return A crl::multisense::Status indicating if the device information
     * query was successful
     */

    virtual Status getDeviceInfo       (system::DeviceInfo& info)           = 0;

    /**
     * Set the current sensor's device information.
     *
     * NOTE: This function is intended for use at the factory, and is
     * not publicly supported.
     *
     * @param key The secret key required to write new device information
     *
     * @param i The new DeviceInfo to write to the sensor
     *
     * @return A crl::multisense::Status indicating if the device information
     * was successfully received
     */

    virtual Status setDeviceInfo       (const std::string& key,
                                        const system::DeviceInfo& i)        = 0;

    /**
     * Flash a new FPGA bitstream file to the sensor.
     *
     * WARNING: This member should not be used directly. Inproper usage can
     * result in the sensor being inoperable. Use the MultiSenseUpdater
     * script to update the sensor's firmware/bitstream
     *
     * @param file The path to the file containing the new sensor bitstream
     *
     * @return A crl::multisense::Status indicating if the new bitstream
     * was successfully flashed
     */

    virtual Status flashBitstream      (const std::string& file)            = 0;

    /**
     * Flash a new firmware file to the sensor.
     *
     * WARNING: This member should not be used directly. Inproper usage can
     * result in the sensor being inoperable. Use the MultiSenseUpdater
     * script to update the sensor's firmware/bitstream
     *
     * @param file The path to the file containing the new sensor firmware
     *
     * @return A crl::multisense::Status indicating if the new firmware
     * was successfully flashed
     */

    virtual Status flashFirmware       (const std::string& file)            = 0;

    /**
     * Verify that the current bitstream in the sensor's flash is the same as
     * the bitstream specified by file
     *
     * @param file The path to the file containing the bitstream to check
     *
     * @return A crl::multisense::Status indicating if the bitstream specified
     * by file is loaded in the sensor's non-volatile flash memory
     */

    virtual Status verifyBitstream     (const std::string& file)            = 0;

    /**
     * Verify the current firmware in the sensor's flash is the same as
     * the firmware specified by file
     *
     * @param file The path to the file containing the firmware to check
     *
     * @return A crl::multisense::Status indicating if the firmware specified
     * by file is loaded in the sensor's non-volatile flash memory
     */

    virtual Status verifyFirmware      (const std::string& file)            = 0;

    //
    // IMU configuration.
    //
    // Detailed info may be queried, and configuration may be queried or set.
    //
    // See imu::Details and imu::Config classes for more information.
    //
    // 'samplesPerMessage' is the number of samples (aggregate from all IMU types)
    // that the sensor will queue internally before putting on the wire.
    // Note that low settings combined with high IMU sensor rates may interfere
    // with the acquisition and transmission of image and lidar data.
    //
    // For setImuConfig():
    //
    //    Set 'storeSettingsInFlash' to true to have the configuration saved in
    //    non-volatile flash on the sensor head.
    //
    //    Set 'samplesPerMessage' to zero for the sensor to keep its current
    //    samplesPerMessage setting.
    //
    //    IMU streams must be restarted for any configuration changes to be
    //    reflected.

    /**
     * Query detailed information about the current sensor's IMU.
     *
     * See imu::Info for a usage example
     *
     * @param maxSamplesPerMesage The maximum number of IMU samples which
     * can be aggregated in a single IMU message
     *
     * @param info A vector of imu::Info returned by reference. This contains
     * all the possible configurations for each IMU sensor
     *
     * @return A crl::multisense::Status indicating if the IMU info was
     * successfully queried
     */

    virtual Status getImuInfo          (uint32_t& maxSamplesPerMesage,
                                        std::vector<imu::Info>& info)       = 0;

    /**
     * Query the current IMU configuration.
     *
     * See imu::Config for a usage example
     *
     * @param samplesPerMessage The number of samples (aggregate from all IMU
     * types) that the sensor will queue internally before putting on the wire.
     * Note that low settings combined with high IMU sensor rates may interfere
     * with the acquisition and transmission of image and lidar data.
     *
     * @param c The current imu configuration returned by reference
     *
     * @return A crl::multisense::Status indicating if the IMU configuration was
     * successfully queried
    */

    virtual Status getImuConfig        (uint32_t& samplesPerMessage,
                                        std::vector<imu::Config>& c)        = 0;

    /**
     * Set a new IMU configuration.
     *
     * See imu::Config for a usage example
     *
     * IMU streams must be restarted for any configuration changes to be
     * reflected.
     *
     * @param storeSettingsInFlash A boolean flag which indicated if the
     * configuration saved in non-volatile flash on the sensor head
     *
     * @param samplesPerMessage The new number of IMU samples to
     * aggregate before sending a new IMU message. If the value is
     * zero the sensor will keep its current samplesPerMessage setting
     *
     * @param c The imu configuration that will be sent to the sensor.
     *
     * @return A crl::multisense::Status indicating if the IMU configuration was
     * successfully received
     */
    virtual Status setImuConfig        (bool storeSettingsInFlash,
                                        uint32_t samplesPerMessage,
                                        const std::vector<imu::Config>& c)  = 0;

    /**
     *
     * Query the suggested number and size of the image buffers.
     *
     * NOTe: Other number/size can be used but it is not recommended
     *
     * The channel maintains and recycles a set of large buffers used for
     * image storage and dispatching.
     *
     * @param bufferCount The number of buffers returned by reference
     *
     * @param bufferSize The size of a single buffer returned by reference
     *
     * @return A crl::multisense::Status indicating if the buffer details were
     * successfully queried
     */

    virtual Status getLargeBufferDetails(uint32_t& bufferCount,
                                         uint32_t& bufferSize) = 0;
    /**
     * Set user allocated large buffer.
     *
     * This tells the channel to use the supplied buffers in lieu
     * of its automatically allocated internal buffers. The channel's internal buffers
     * will be freed.
     *
     * All supplied buffers must be of the same size.
     *
     * Responsibility for freeing the supplied buffers after channel closure is left
     * to the user
     *
     * @param buffers A vector of new buffers to use for image storage. All
     * buffers in the vector must be of the same size
     *
     * @param bufferSize The size of one buffer in the buffers vector
     *
     * @return A crl::multisense::Status indicating if the new buffers were
     * successfully received
     */
    virtual Status setLargeBuffers      (const std::vector<uint8_t*>& buffers,
                                         uint32_t                     bufferSize) = 0;

    /**
     * Query the system-assigned local UDP port.
     *
     * @param port The local system UDP port returned by reference
     *
     * @return A crl::multisense::Status indicating if the local UDP port was
     * successfully queried
     */

    virtual Status getLocalUdpPort(uint16_t& port) = 0;
};


}; // namespace multisense
}; // namespace crl

#endif // LibMultiSense_MultiSenseChannel_hh
