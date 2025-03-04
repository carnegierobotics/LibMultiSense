# LibMultiSense

- [Hello World](#hello-world)
  - [OpenCV Integration](#opencv-integration)
  - [Copy-Free Buffer Reservations](#copy-free-operations-image-buffer-reservations)
  - [Camera Configuration](#camera-configuration)
  - [Depth Image Generation](#depth-image-generation)
- [Installation](#installation)
- [Documentation](#documentation)
- [Support](#support)

LibMultiSense is a C++ library used to interface with the MultiSense S
family of sensors from Carnegie Robotics. For more information on the
various MultiSense products please visit
https://carnegierobotics.com/products

For more detailed documentation on general MultiSense operation
please visit
https://docs.carnegierobotics.com/docs/index.html

### Installation

#### Linux

LibMultiSense uses CMake for its build system.

To build the standalone LibMultiSense library and demonstration applications.

    # Note this only needs to be run once before building
    > sudo apt install build-essential

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > mkdir build
    > cd build && cmake -DCMAKE_INSTALL_PREFIX=../install ..
    > make install
    > cd ../install

To build the standalone LibMultiSense library without the demonstration applications,
set the cmake variable `-DMULTISENSE_BUILD_UTILITIES=OFF`

Integrating LibMultiSense into an existing CMake project is easy. There are two
primary methods for integration: a local install on your system, or a submodule
clone within your repository

##### Local Installation

LibMultiSense is installed on your system (i.e. in a location like /opt/multisense)

    find_package(MultiSense)
    target_link_libraries(<your-library-or-binary> MultiSense)

When running CMake, make sure to specify the location of the LibMultiSense install
via `-DCMAKE_PREFIX_PATH`

##### Git Submodule

Clone the LibMultiSense repository into the existing project's source tree.
In the main CMakeLists.txt file of the project, add the following lines:

     include_directories(LibMultiSense/source/LibMultiSense)
     add_subdirectory(LibMultiSense/source/LibMultiSense)

#### Windows

LibMultiSense uses CMake to create a Microsoft Visual Studio project file used
to build the LibMultiSense DLL.

Download and install CMake on Windows (http://www.cmake.org/download/), making
sure CMake is included included in the system PATH.

Clone LibMultiSense to the Windows machine using a Windows Git client.

Open a command prompt and execute the following commands:

    > cd <LibMultiSense_checkout_directory>
    > mkdir build
    > cd build
    > cmake ..

This will create a LibMultiSense.sln Visual Studio Solution file in the build directory.
Open the solution file with Visual Studio (http://msdn.microsoft.com/en-us/vstudio/aa718325.aspx)
and build the Solution.

### Documentation

Documentation of high-level LibMutliSense concepts can be found
[here](https://docs.carnegierobotics.com/docs/software/libmultisense.html)

Doxygen documentation can be built for LibMultisense by running the Doxygen
configuration file located in the docs directory

    > cd LibMultiSense/docs
    > doxygen Doxyfile

HTML and LaTex documentation will be generated in the docs directory.

Usage examples are included in the Doxygen documentation.

### Support

To report an issue with this library or request a new feature,
please use the [GitHub issues system](https://github.com/carnegierobotics/LibMultiSense/issues)

For product support, please see the [support section of our website](https://carnegierobotics.com/support)
Individual support requests can be created in our [support portal](https://carnegierobotics.com/submitaticket)

### Hello World

LibMultiSense builds as a single shared library which can be linked into
any existing project.

The two header files MultiSenseChannel.hh and MultiSenseTypes.hh contain
all the declarations necessary to interface with a MultiSense sensor.

The following example demonstrates how to connect, stream, and receive image
for the MultiSense camera.

#### test.cpp

```c++
#include <iostream>
#include <unistd.h>

#include <MultiSense/MultiSenseTypes.hh>
#include <MultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

struct UserData
{
    size_t count = 0;
};

void image_callback(const image::Header& header, void* user_data)
{
    UserData* meta = reinterpret_cast<UserData*>(user_data);
    meta->count++;

    switch (header.source) {
        case Source_Luma_Left: std::cout << "luma left" << std::endl; break;
        case Source_Luma_Right: std::cout << "luma right" << std::endl; break;
        case Source_Disparity: std::cout << "disparity" << std::endl; break;
    }

    std::cout << "frame_id: " << header.frameId <<
                 ", seconds: " << header.timeSeconds <<
                 ", microseconds: " << header.timeMicroSeconds << std::endl;
}

int main()
{
    //
    // Instantiate a channel connecting to a sensor at the factory default
    // IP address
    Channel* channel = nullptr;
    channel = Channel::Create("10.66.171.21");
    channel->setMtu(1500);

    Status status;

    //
    // Data which can be shared among callbacks
    UserData meta;

    //
    // Attached a callback to the Channel which will get called when certain image types
    // are received by the camera. Multiple image callbacks can be attached to a
    // Channel
    status = channel->addIsolatedCallback(image_callback, Source_Luma_Left, &meta);
    status = channel->startStreams(Source_Luma_Left);
    if(Status_Ok != status) {
        std::cerr << "unable to add isolated callbacks and start image streams" << std::endl;
    }

    //
    // Spin until the user wants to exit. Images will be serviced in the image_callback
    // as they are received by the camera
    while(true)
    {
        usleep(100000);
    }

    //
    // Stop streams and remove our callback
    status = channel->stopStreams(Source_All);
    status = channel->removeIsolatedCallback(image_callback);

    if(Status_Ok != status) {
        std::cerr << "unable to stop streams and remove isolated callback" << std::endl;
    }

    //
    // Destroy the channel instance
    Channel::Destroy(channel);
}

```

#### CMakeLists.txt

```
cmake_minimum_required(VERSION 3.0)
project(multisense_example VERSION 0.0.0.0 LANGUAGES C CXX)

find_package(MultiSense REQUIRED)
add_executable(test test.cpp)
target_link_libraries(test MultiSense)

install(TARGETS test
        RUNTIME DESTINATION bin)
```

#### Build Hello World

    > mkdir build && cd build
    > cmake -DCMAKE_PREFIX_PATH=<path-to-libmultisense-install> ..
    > make

### OpenCV Integration

To display the images received from the camera in a OpenCV window, the image callback function
and CMakeLists.txt outlined in the [Hello World](#hello-world) example can be
updated to the following

#### image_callback function

```c++
#include <opencv2/highgui.hpp>

void image_callback(const image::Header& header, void* user_data)
{
    //
    // Create a OpenCV matrix using our image container
    uint8_t* raw_image_data =
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(header.imageDataP));
    const cv::Mat_<uint8_t> image(header.height, header.width, raw_image_data);

    //
    // Display the image using OpenCV
    cv::namedWindow("example");
    cv::imshow("example", image);
    cv::waitKey(1);
}
```

#### CMakeLists.txt

```
find_package(MultiSense REQUIRED)
find_package(OpenCV REQUIRED core highgui imgproc)
add_executable(test test.cpp)
target_link_libraries(test MultiSense
                           opencv_core
                           opencv_highgui
                           opencv_imgproc)
```

### Copy-free Operations Image Buffer Reservations

Many use cases require multiple MultiSense image sources to be combined into artifacts
like color images or point clouds. By default, the memory underlying a Header object
is only valid in the scope of a callback. LibMultiSense offers a reservation mechanism
to extend the lifetime of data underlying Header objects for as long as the user would like.

The following modified version of the [Hello World](#hello-world) example,
highlights how to perform this reservation operation to create and display color aux images.

Note, there are a limited number of buffers which LibMultiSense internally manages to store
large image data. Depending on the processing speed of the host machine, and the streaming
frequency of the MultiSense, there could be situations where all the internal buffers are
actively reserved. The `Channel::setLargeBuffers` member function can be used to increased
the number of available internal buffers.

#### test.cc

```c++
#include <iostream>
#include <unistd.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <MultiSense/MultiSenseTypes.hh>
#include <MultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

struct UserData
{
    Channel* channel = nullptr;

    image::Header mono_image;
    image::Header chroma_image;

    void* mono_image_ref = nullptr;
    void* chroma_image_ref = nullptr;
};

void image_callback(const image::Header& header, void* user_data)
{
    UserData* meta = reinterpret_cast<UserData*>(user_data);

    if (!meta->channel) return;

    //
    // Reserve our header for color processing. If there is already a valid
    // ref for the image, it means the corresponding luma or chroma image
    // was dropped, and we should release the existing buffer before reserving
    switch (header.source) {
        case Source_Luma_Aux:
            if (meta->mono_image_ref) {
                meta->channel->releaseCallbackBuffer(meta->mono_image_ref);
            }

            meta->mono_image_ref = meta->channel->reserveCallbackBuffer();
            meta->mono_image = header;

            break;
        case Source_Chroma_Aux:
            if (meta->chroma_image_ref) {
                meta->channel->releaseCallbackBuffer(meta->chroma_image_ref);
            }

            meta->chroma_image_ref = meta->channel->reserveCallbackBuffer();
            meta->chroma_image = header;
            break;
    }

    //
    // If we have a valid luma and chroma pair create a color image
    if (meta->mono_image_ref &&
        meta->chroma_image_ref &&
        meta->mono_image.frameId == meta->chroma_image.frameId) {

        //
        // Create a OpenCV images using our stored image headers.
        uint8_t* raw_luma =
            const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(meta->mono_image.imageDataP));
        const cv::Mat_<uint8_t> luma(meta->mono_image.height,
                                     meta->mono_image.width,
                                     raw_luma);

        uint16_t* raw_chroma =
            const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(meta->chroma_image.imageDataP));
        const cv::Mat_<uint16_t> chroma(meta->chroma_image.height,
                                        meta->chroma_image.width,
                                        raw_chroma);

        cv::Mat bgr;
        cv::cvtColorTwoPlane(luma, chroma, bgr, cv::COLOR_YUV2BGR_NV12);

        cv::namedWindow("example");
        cv::imshow("example", bgr);
        cv::waitKey(1);

        //
        // Release and invalidate our buffers
        meta->channel->releaseCallbackBuffer(meta->mono_image_ref);
        meta->channel->releaseCallbackBuffer(meta->chroma_image_ref);
        meta->mono_image_ref = nullptr;
        meta->chroma_image_ref = nullptr;
    }
}

int main()
{
    //
    // Instantiate a channel connecting to a sensor at the factory default
    // IP address
    Channel* channel = nullptr;
    channel = Channel::Create("10.66.171.21");
    channel->setMtu(1500);

    Status status;

    //
    // Data which can be shared among callbacks
    UserData meta{channel, {}, {}, nullptr, nullptr};

    //
    // Attached a callback to the Channel which will get called when certain image types
    // are received by the camera. Multiple image callbacks can be attached to a
    // Channel
    status = channel->addIsolatedCallback(image_callback,
                                           Source_Luma_Aux |
                                           Source_Chroma_Aux,
                                           &meta);
    status = channel->startStreams(Source_Luma_Aux | Source_Chroma_Aux);
    if(Status_Ok != status) {
        std::cerr << "unable to add isolated callbacks and start image streams" << std::endl;
    }

    //
    // Spin until the user wants to exit. Images will be serviced in the image_callback
    // as they are received by the camera
    while(true)
    {
        usleep(100000);
    }

    //
    // Stop streams and remove our callback
    status = channel->stopStreams(Source_All);
    status = channel->removeIsolatedCallback(image_callback);

    if(Status_Ok != status) {
        std::cerr << "unable to stop streams and remove isolated callback" << std::endl;
    }

    //
    // Destroy the channel instance
    Channel::Destroy(channel);
}
```

### Camera Configuration

Camera settings like resolution, exposure, FPS, gain, gamma, and white balance can be configured
via the LibMultiSense `image::Config`

#### test.cc

```c++
int main()
{
    //
    // Instantiate a channel connecting to a sensor at the factory default
    // IP address. Note jumbo frames are suggested when running at 30 FPS
    Channel* channel = nullptr;
    channel = Channel::Create("10.66.171.21");
    channel->setMtu(9000);

    Status status;

    //
    // Query the current camera's image configuration, and update the desired
    // configuration settings
    image::Config image_config;
    status = channel->getImageConfig(image_config);
    if (Status_Ok != status) {
        std::cerr << "Failed to get image config"  << std::endl;
    }

    image_config.setResolution(960, 600);
    image_config.setDisparities(256);
    image_config.setFps(30.0);

    status = channel->setImageConfig(image_config);
    if (Status_Ok != status) {
        std::cerr << "Failed to set image config" << std::endl;
    }

    Channel::Destroy(channel);
}
```

### Depth Image Generation

Disparity images can be converted to depth images using the camera's onboard calibration.

The following modified version of the [Hello World](#hello-world) example,
converts disparity images to openni depth images and saves them to disk using OpenCV.

#### test.cpp

```c++
#include <iostream>
#include <limits>
#include <unistd.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <MultiSense/MultiSenseTypes.hh>
#include <MultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

struct UserData
{
    image::Calibration calibration;

    crl::multisense::system::DeviceInfo device_info;
};

void image_callback(const image::Header& header, void* user_data)
{
    UserData* meta = reinterpret_cast<UserData*>(user_data);

    //
    // Scale the full resolution calibration based on the current operating resolution
    // Also negate the tx estimate to get the raw baseline
    // See https://docs.carnegierobotics.com/docs/calibration/stereo.html
    const double x_scale = static_cast<double>(header.width) /
                           static_cast<double>(meta->device_info.imagerWidth);

    const double f = meta->calibration.left.P[0][0] * x_scale;
    const double b = -1.0 * meta->calibration.right.P[0][3] /
                     meta->calibration.right.P[0][0];

    const uint16_t* raw_disparity = reinterpret_cast<const uint16_t*>(header.imageDataP);

    //
    // Convert each quantized disparity pixel to depth using: z = (f * b) / d.
    // Store the output in the quantized openni depth image format
    const double max_ni_depth = std::numeric_limits<uint16_t>::max();
    std::vector<uint16_t> output_depth_buffer(header.width * header.height, 0);
    for (size_t i = 0 ; i < (header.width * header.height) ; ++i)
    {
        const uint16_t d = raw_disparity[i];
        if (d == 0) continue;

        //
        // Disparity images are quantized to 1/16 of a pixel
        const double z = (f * b) / (static_cast<double>(d) / 16.0);

        //
        // OpenNI Depth images are quantized to millimeters
        output_depth_buffer[i] =
            static_cast<uint16_t>(std::min(max_ni_depth, std::max(0.0, z * 1000)));
    }

    //
    // Save the output depth image with OpenCV
    const cv::Mat_<uint16_t> depth_image(header.height, header.width,
                                         output_depth_buffer.data());
    cv::imwrite(std::to_string(header.frameId) + ".png", depth_image);
}

int main()
{
    //
    // Instantiate a channel connecting to a sensor at the factory default
    // IP address
    Channel* channel = nullptr;
    channel = Channel::Create("10.66.171.21");
    channel->setMtu(1500);

    Status status;

    //
    // Query calibration
    image::Calibration calibration;
    status = channel->getImageCalibration(calibration);
    if (Status_Ok != status) {
        std::cerr << "Failed to query calibraiton" << std::endl;
    }

    //
    // Query device info
    system::DeviceInfo device_info;
    status = channel->getDeviceInfo(device_info);
    if (Status_Ok != status) {
        std::cerr << "Failed to query device info" << std::endl;
    }

    //
    // Query and set the image config to 1/4 resolution
    image::Config image_config;
    status = channel->getImageConfig(image_config);
    if (Status_Ok != status) {
        std::cerr << "Failed to get image config" << std::endl;
    }

    image_config.setResolution(device_info.imagerWidth / 2, device_info.imagerHeight / 2);
    image_config.setDisparities(256);
    image_config.setFps(10.0);

    status = channel->setImageConfig(image_config);
    if (Status_Ok != status) {
        std::cerr << "Failed to set image config" << std::endl;
    }

    //
    // Data which can be shared among callbacks
    UserData meta{calibration, device_info};

    //
    // Attached a callback to the Channel which will get called when certain image types
    // are received by the camera. Multiple image callbacks can be attached to a
    // Channel
    status = channel->addIsolatedCallback(image_callback, Source_Disparity, &meta);
    status = channel->startStreams(Source_Disparity);
    if(Status_Ok != status) {
        std::cerr << "unable to add isolated callbacks and start image streams" << std::endl;
    }

    //
    // Spin until the user wants to exit. Images will be serviced in the image_callback
    // as they are received by the camera
    while(true)
    {
        usleep(100000);
    }

    //
    // Stop streams and remove our callback
    status = channel->stopStreams(Source_All);
    status = channel->removeIsolatedCallback(image_callback);

    if(Status_Ok != status) {
        std::cerr << "unable to stop streams and remove isolated callback" << std::endl;
    }

    //
    // Destroy the channel instance
    Channel::Destroy(channel);
}
```
