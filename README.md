# LibMultiSense

- [Hello World](#libmultisense-hello-world)
    - [OpenCV Integration](#opencv-integration)
    - [Copy-Free Buffer Reservations](#copy-free-operations-image-buffer-reservations)
- [Installation](#installation)
- [Doccumentation](#doccumentation)
- [Support](#support)

LibMultiSense is a C++ library used to interface with the MultiSense S
family of sensors from Carnegie Robotics. For more information on the
various MultiSense products please visit
http://carnegierobotics.com/products/

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

Doccumentation of high-level LibMutliSense concepts can be found
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
Individual support requests can be created in our [support portal](https://support.carnegierobotics.com/hc/en-us)


### LibMultiSense Hello World

LibMultiSense builds as a single shared library which can be linked into
any existing project.

The two header files MultiSenseChannel.hh and MultiSenseTypes.hh contain
all the declarations necessary to interface with a MultiSense sensor.

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
    UserData* metadata = reinterpret_cast<UserData*>(user_data);
    metadata->count++;

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
and CMakeLists.txt outlined in the [Hello World](#libmultisense-hello-orld) example can be
updated to the following

#### image_callback function

```c++
#include <opencv2/highgui.hpp>

void image_callback(const image::Header& header, void* user_data)
{
    //
    // Create a OpenCV matrix using our image container
    uint8_t* raw_image_data = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(header.imageDataP));
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

The following modified version of the [Hello World](#libmultisense-hello-world) example,
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
    UserData* metadata = reinterpret_cast<UserData*>(user_data);

    if (!metadata->channel) return;

    //
    // Reserve our header for color processing. If there is already a valid
    // ref for the image, it means the corresponding luma or chroma image
    // was dropped, and we should release the existing buffer before reserving
    switch (header.source) {
        case Source_Luma_Aux:
            if (metadata->mono_image_ref) {
                metadata->channel->releaseCallbackBuffer(metadata->mono_image_ref);
            }

            metadata->mono_image_ref = metadata->channel->reserveCallbackBuffer();
            metadata->mono_image = header;

            break;
        case Source_Chroma_Aux:
            if (metadata->chroma_image_ref) {
                metadata->channel->releaseCallbackBuffer(metadata->chroma_image_ref);
            }

            metadata->chroma_image_ref = metadata->channel->reserveCallbackBuffer();
            metadata->chroma_image = header;
            break;
    }

    //
    // If we have a valid luma and chroma pair create a color image
    if (metadata->mono_image_ref &&
        metadata->chroma_image_ref &&
        metadata->mono_image.frameId == metadata->chroma_image.frameId) {

        //
        // Create a OpenCV images using our stored image headers.
        uint8_t* raw_luma = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(metadata->mono_image.imageDataP));
        const cv::Mat_<uint8_t> luma(metadata->mono_image.height, metadata->mono_image.width, raw_luma);

        uint16_t* raw_chroma = const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(metadata->chroma_image.imageDataP));
        const cv::Mat_<uint16_t> chroma(metadata->chroma_image.height, metadata->chroma_image.width, raw_chroma);

        cv::Mat bgr;
        cv::cvtColorTwoPlane(luma, chroma, bgr, cv::COLOR_YUV2BGR_NV12);

        cv::namedWindow("example");
        cv::imshow("example", bgr);
        cv::waitKey(1);

        //
        // Release and invalidate our buffers
        metadata->channel->releaseCallbackBuffer(metadata->mono_image_ref);
        metadata->channel->releaseCallbackBuffer(metadata->chroma_image_ref);
        metadata->mono_image_ref = nullptr;
        metadata->chroma_image_ref = nullptr;
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
    status = channel->addIsolatedCallback(image_callback, Source_Luma_Aux | Source_Chroma_Aux, &meta);
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
