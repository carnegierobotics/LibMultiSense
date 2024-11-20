# LibMultiSense

LibMultiSense is a C++ library used to interface with the MultiSense S
family of sensors from Carnegie Robotics. For more information on the
various MultiSense products please visit
http://carnegierobotics.com/products/

### LibMultiSense Hello World

#### test.cpp
```
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
    UserData* metadata= reinterpret_cast<UserData*>(user_data);
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

LibMultiSense builds as a single shared library which can be linked into
any existing project.

The two header files MultiSenseChannel.hh and MultiSenseTypes.hh contain
all the declarations necessary to interface with a MultiSense sensor.

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
