#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h> // htons
#endif

#include <atomic>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <getopt/getopt.h>

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseChannel.hh>

#include "thermal_wire.h"

using namespace crl::multisense;

namespace {  // anonymous

const char *THERMAL_APP_NAME = "crl_thermal";

volatile bool doneG = false;

class RawAppConfig : public system::SecondaryAppConfig
{
public:
    RawAppConfig() {}

    RawAppConfig(const void *d, uint32_t len)
    {
        dataLength = len;
        memcpy(data, d, len);
    }

    virtual void serialize() {}
};

struct Stats
{
    std::atomic<uint64_t> groupsOk{0};
    std::atomic<uint64_t> groupsBad{0};
    std::atomic<uint64_t> imagesOk{0};
    std::atomic<uint64_t> bytesOk{0};
    std::atomic<uint64_t> framesSkipped{0};   // frame-id gaps; expected under load
    std::atomic<uint64_t> metaMismatch{0};
    std::atomic<uint64_t> pgmsSaved{0};
};

struct UserData
{
    Stats       stats;
    int64_t     lastFrameId    = -1;
    bool        verbose        = true;
    bool        savePgms       = false;
    std::string saveDir        = ".";
    uint64_t    framesToCapture = 0;   // 0 = run until ^C
    bool        printedFirst   = false;
};

void usage(const char *programNameP)
{
    std::cerr << "USAGE: " << programNameP << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <address>   : IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>       : MTU to set (default: negotiate best)" << std::endl;
    std::cerr << "\t-c             : query and print thermal config, then exit" << std::endl;
    std::cerr << "\t-r <0|1>       : send CTRL_SET_RECTIFIED before streaming" << std::endl;
    std::cerr << "\t-b <8|16>      : send CTRL_SET_BITS_PER_PIXEL (expected to" << std::endl;
    std::cerr << "\t                 fail until tura_cmd plumbing lands)" << std::endl;
    std::cerr << "\t-e <mask>      : send CTRL_SET_IMAGER_ENABLES, hex ok (ditto)" << std::endl;
    std::cerr << "\t-n <count>     : exit after <count> valid frame groups" << std::endl;
    std::cerr << "\t-s <dir>       : save every image of every group as PGM in <dir>" << std::endl;
    std::cerr << "\t-q             : quiet (1 Hz stats only, no per-group lines)" << std::endl;
    exit(1);
}

#ifdef WIN32
BOOL WINAPI signalHandler(DWORD dwCtrlType)
{
    CRL_UNUSED (dwCtrlType);
    doneG = true;
    return TRUE;
}
#else
void signalHandler(int sig)
{
    (void) sig;
    doneG = true;
}
#endif

//
// P5 PGM, big-endian 16-bit per the format spec (same as SaveImage/FD utils)
bool savePgm(const std::string &fileName,
             uint32_t width, uint32_t height, uint32_t bitsPerPixel,
             const void *dataP)
{
    std::ofstream outputStream(fileName.c_str(), std::ios::binary | std::ios::out);

    if (false == outputStream.good()) {
        std::cerr << "Failed to open \"" << fileName << "\"" << std::endl;
        return false;
    }

    const uint32_t imageSize = height * width;

    switch (bitsPerPixel) {
    case 8:
    {
        outputStream << "P5\n" << width << " " << height << "\n" << 0xFF << "\n";
        outputStream.write(reinterpret_cast<const char*>(dataP), imageSize);
        break;
    }
    case 16:
    {
        outputStream << "P5\n" << width << " " << height << "\n" << 0xFFFF << "\n";

        const uint16_t *imageP = reinterpret_cast<const uint16_t*>(dataP);

        for (uint32_t i = 0; i < imageSize; ++i) {
            uint16_t o = htons(imageP[i]);
            outputStream.write(reinterpret_cast<const char*>(&o), sizeof(uint16_t));
        }
        break;
    }
    default:
        std::cerr << "Unsupported bpp for PGM: " << bitsPerPixel << std::endl;
        return false;
    }

    outputStream.close();
    return true;
}

void printConfig(const thermal_wire::config_t &cfg)
{
    std::cout << "Thermal config:" << std::endl;
    std::cout << "  magic              : 0x" << std::hex << cfg.magic << std::dec
              << (cfg.magic == thermal_wire::GROUP_MAGIC ? " (ok)" : " (BAD)") << std::endl;
    std::cout << "  version            : " << cfg.version << std::endl;
    std::cout << "  rectified          : " << (unsigned)cfg.rectified << std::endl;
    std::cout << "  bits_per_pixel     : " << (unsigned)cfg.bits_per_pixel << std::endl;
    std::cout << "  num_imagers_max    : " << (unsigned)cfg.num_imagers_max << std::endl;
    std::cout << "  imager_enable_mask : 0x" << std::hex << cfg.imager_enable_mask << std::dec << std::endl;
    std::cout << "  width x height     : " << cfg.width << " x " << cfg.height
              << (cfg.width == 0 ? "  (0 until first frame flows -- expected pre-stream)" : "")
              << std::endl;
}

//
// Validate one SecondaryAppData message against the thermal_wire contract.
// Every check that fails prints once and counts the group as bad; the walk
// keeps going so one message shows all its problems at once.
void secondaryAppCallback(const secondary_app::Header &header, void *userDataP)
{
    UserData *ud = reinterpret_cast<UserData*>(userDataP);

    const uint8_t *payload = reinterpret_cast<const uint8_t*>(header.secondaryAppDataP);
    const uint32_t length  = header.secondaryAppDataLength;

    bool ok = true;

    //
    // Group header (memcpy: payload alignment is not guaranteed)

    if (length < sizeof(thermal_wire::group_header_t)) {
        std::cerr << "frame " << header.frameId << ": payload too short for group header ("
                  << length << "b)" << std::endl;
        ud->stats.groupsBad++;
        return;
    }

    thermal_wire::group_header_t gh;
    memcpy(&gh, payload, sizeof(gh));

    if (gh.magic != thermal_wire::GROUP_MAGIC) {
        std::cerr << "frame " << header.frameId << ": bad magic 0x" << std::hex << gh.magic
                  << std::dec << " (want 'T6RG' = 54 36 52 47 on the wire)" << std::endl;
        ud->stats.groupsBad++;
        return;   // nothing downstream is trustworthy
    }

    if (gh.version != thermal_wire::GROUP_VERSION) {
        std::cerr << "frame " << header.frameId << ": version " << gh.version
                  << " != " << thermal_wire::GROUP_VERSION << std::endl;
        ok = false;
    }

    if (gh.payload_type != thermal_wire::PAYLOAD_FRAME_GROUP) {
        std::cerr << "frame " << header.frameId << ": unexpected payload_type "
                  << gh.payload_type << std::endl;
        ok = false;
    }

    if (gh.frame_id != header.frameId) {
        std::cerr << "frame " << header.frameId << ": group frame_id " << gh.frame_id
                  << " disagrees with SecondaryAppHeader" << std::endl;
        ok = false;
    }

    const uint32_t expect_hdr_len = (uint32_t)(sizeof(thermal_wire::group_header_t) +
                                    gh.num_images * sizeof(thermal_wire::image_desc_t));

    if (gh.num_images == 0 || gh.num_images > thermal_wire::MAX_IMAGERS) {
        std::cerr << "frame " << header.frameId << ": bad num_images "
                  << (unsigned)gh.num_images << std::endl;
        ud->stats.groupsBad++;
        return;
    }

    if (gh.header_length != expect_hdr_len || gh.header_length > length) {
        std::cerr << "frame " << header.frameId << ": header_length " << gh.header_length
                  << " (expected " << expect_hdr_len << ", payload " << length << "b)" << std::endl;
        ud->stats.groupsBad++;
        return;
    }

    //
    // Metadata cross-check: the firmware sends the same group header as the
    // per-frame SecondaryAppMetadata payload.

    if (header.secondaryAppMetadataLength != sizeof(thermal_wire::group_header_t)) {
        std::cerr << "frame " << header.frameId << ": metadata length "
                  << header.secondaryAppMetadataLength << " != " << sizeof(thermal_wire::group_header_t)
                  << std::endl;
        ud->stats.metaMismatch++;
        ok = false;
    } else {
        thermal_wire::group_header_t mh;
        memcpy(&mh, header.secondaryAppMetadataP, sizeof(mh));
        if (mh.magic != thermal_wire::GROUP_MAGIC || mh.frame_id != gh.frame_id) {
            std::cerr << "frame " << header.frameId << ": metadata magic/frame_id mismatch"
                      << std::endl;
            ud->stats.metaMismatch++;
            ok = false;
        }
    }

    //
    // Descriptor walk: contiguous tiling, in-bounds, geometry-consistent

    uint32_t expect_offset = gh.header_length;

    for (unsigned i = 0; i < gh.num_images; ++i) {

        thermal_wire::image_desc_t d;
        memcpy(&d, payload + sizeof(gh) + i * sizeof(d), sizeof(d));

        const uint32_t geom_len = (uint32_t)d.width * d.height * (d.bits_per_pixel / 8);

        if (d.bits_per_pixel != 8 && d.bits_per_pixel != 16) {
            std::cerr << "frame " << header.frameId << " img " << i << ": bpp "
                      << (unsigned)d.bits_per_pixel << std::endl;
            ok = false;
        }
        if (d.offset != expect_offset) {
            std::cerr << "frame " << header.frameId << " img " << i << ": offset " << d.offset
                      << " != expected " << expect_offset << " (tiling gap/overlap)" << std::endl;
            ok = false;
        }
        if (d.length != geom_len) {
            std::cerr << "frame " << header.frameId << " img " << i << ": length " << d.length
                      << " != w*h*bpp/8 = " << geom_len << std::endl;
            ok = false;
        }
        if ((uint64_t)d.offset + d.length > length) {
            std::cerr << "frame " << header.frameId << " img " << i << ": data extends past payload"
                      << std::endl;
            ud->stats.groupsBad++;
            return;   // don't touch pixels out of bounds
        }

        expect_offset = d.offset + d.length;

        if (ok && ud->savePgms) {
            std::stringstream name;
            name << ud->saveDir << "/thermal_f" << std::setw(8) << std::setfill('0')
                 << gh.frame_id << "_i" << (unsigned)d.imager_id
                 << ((d.flags & thermal_wire::IMG_FLAG_RECTIFIED) ? "_rect" : "")
                 << ".pgm";
            if (savePgm(name.str(), d.width, d.height, d.bits_per_pixel, payload + d.offset))
                ud->stats.pgmsSaved++;
        }
    }

    if (expect_offset != length) {
        std::cerr << "frame " << header.frameId << ": descriptors tile to " << expect_offset
                  << "b but payload is " << length << "b" << std::endl;
        ok = false;
    }


    if (ud->lastFrameId >= 0 && gh.frame_id > ud->lastFrameId + 1)
        ud->stats.framesSkipped += (gh.frame_id - ud->lastFrameId - 1);
    ud->lastFrameId = gh.frame_id;

    if (!ok) {
        ud->stats.groupsBad++;
        return;
    }

    ud->stats.groupsOk++;
    ud->stats.imagesOk += gh.num_images;
    ud->stats.bytesOk  += length;

    if (!ud->printedFirst) {
        ud->printedFirst = true;
        std::cout << "First valid group: frame " << gh.frame_id
                  << ", " << (unsigned)gh.num_images << " images, "
                  << length << "b, enables 0x" << std::hex << gh.imager_enable_mask << std::dec
                  << ", ptp_locked " << (unsigned)gh.ptp_locked << std::endl;
    }

    if (ud->verbose) {
        std::cout << "frame " << gh.frame_id
                  << "  imgs " << (unsigned)gh.num_images
                  << "  " << length << "b"
                  << "  t " << gh.time_seconds << "." << std::setw(6) << std::setfill('0')
                  << gh.time_microseconds << std::setfill(' ')
                  << (gh.ptp_locked ? " (ptp)" : "")
                  << std::endl;
    }

    if (ud->framesToCapture && ud->stats.groupsOk >= ud->framesToCapture)
        doneG = true;
}

//
// Send one thermal_wire control_t; report the ACK
Status sendControl(Channel *channelP, uint16_t cmd, uint32_t value, const char *what)
{
    thermal_wire::control_t c = {};
    c.magic   = thermal_wire::GROUP_MAGIC;
    c.version = thermal_wire::GROUP_VERSION;
    c.cmd     = cmd;
    c.value   = value;

    RawAppConfig cfg(&c, sizeof(c));

    const Status status = channelP->setSecondaryAppConfig(cfg);
    std::cout << "control " << what << " = " << value << " : "
              << Channel::statusString(status) << std::endl;
    return status;
}

} // anonymous

int main(int argc, char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    int32_t  mtu          = 0;
    bool     configOnly   = false;
    int      setRectified = -1;
    int      setBpp       = -1;
    UserData userData;
    crl::multisense::image::Config lms_cfg;

#if WIN32
    SetConsoleCtrlHandler (signalHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
#endif

    int opt;
    while (-1 != (opt = getopt(argc, argvPP, "a:m:cr:b:e:n:s:q")))
        switch (opt) {
        case 'a': currentAddress = std::string(optarg);                    break;
        case 'm': mtu            = atoi(optarg);                           break;
        case 'c': configOnly     = true;                                   break;
        case 'r': setRectified   = atoi(optarg) ? 1 : 0;                   break;
        case 'b': setBpp         = atoi(optarg);                           break;
        case 'n': userData.framesToCapture = strtoull(optarg, NULL, 0);    break;
        case 's': userData.savePgms = true; userData.saveDir = optarg;     break;
        case 'q': userData.verbose = false;                                break;
        default: usage(*argvPP);                                           break;
        }

    Channel *channelP = Channel::Create(currentAddress);
    if (NULL == channelP) {
        std::cerr << "Failed to establish communications with \"" << currentAddress << "\"" << std::endl;
        return -1;
    }

    Status status;
    int ret = 0;

    {
        system::VersionInfo v;
        status = channelP->getVersionInfo(v);
        if (Status_Ok != status) {
            std::cerr << "Failed to query version: " << Channel::statusString(status) << std::endl;
            Channel::Destroy(channelP);
            return -1;
        }

        std::cout << "API build date      : " << v.apiBuildDate << std::endl;
        std::cout << "Firmware build date : " << v.sensorFirmwareBuildDate << std::endl;
        std::cout << "Firmware version    : 0x" << std::hex << std::setw(4) << std::setfill('0')
                  << v.sensorFirmwareVersion << std::dec << std::setfill(' ') << std::endl;
    }

    //
    // 1. Enumerate; find crl_thermal BY NAME (never apps[0] -- ordering is
    //    not a contract, and socket-registered apps can coexist)

    {
        system::SecondaryAppRegisteredApps apps;
        status = channelP->getRegisteredApps(apps);
        if (Status_Ok != status) {
            std::cerr << "getRegisteredApps failed: " << Channel::statusString(status) << std::endl;
            Channel::Destroy(channelP);
            return -2;
        }

        bool found = false;
        std::cout << "Registered secondary apps (" << apps.apps.size() << "):" << std::endl;
        for (const auto &a : apps.apps) {
            const bool match = (a.appName == THERMAL_APP_NAME);
            std::cout << "  " << a.appName << (match ? "   <-- target" : "") << std::endl;
            found |= match;
        }

        if (!found) {
            std::cerr << "\"" << THERMAL_APP_NAME << "\" is not registered on this unit" << std::endl;
            Channel::Destroy(channelP);
            return -2;
        }
    }

    //
    // 2. Activate

    status = channelP->secondaryAppActivate(THERMAL_APP_NAME);
    if (Status_Ok != status) {
        std::cerr << "secondaryAppActivate failed: " << Channel::statusString(status) << std::endl;
        std::cerr << "(older firmware ACKed Status_Unsupported unconditionally here -- "
                  << "if this says Unsupported, the LMS build predates the ACK fix)" << std::endl;
        Channel::Destroy(channelP);
        return -2;
    }
    std::cout << "Activated " << THERMAL_APP_NAME << std::endl;

    //
    // 3. Config round trip

    {
        RawAppConfig cfg;
        status = channelP->getSecondaryAppConfig(cfg);
        if (Status_Ok != status || cfg.dataLength < sizeof(thermal_wire::config_t)) {
            std::cerr << "getSecondaryAppConfig failed: " << Channel::statusString(status)
                      << " (dataLength " << cfg.dataLength << ")" << std::endl;
            ret = -3;
            goto clean_out;
        }

        thermal_wire::config_t tcfg;
        memcpy(&tcfg, cfg.data, sizeof(tcfg));
        printConfig(tcfg);
    }

    if (configOnly)
        goto clean_out;


    if (setRectified >= 0)
        sendControl(channelP, thermal_wire::CTRL_SET_RECTIFIED, setRectified, "rectified");

    if (setBpp >= 0 &&
        Status_Ok != sendControl(channelP, thermal_wire::CTRL_SET_BITS_PER_PIXEL, setBpp, "bpp"))
        std::cout << "  (error expected until tura_cmd plumbing lands)" << std::endl;

    //
    // 4. Stream

    channelP->getImageConfig(lms_cfg);

    lms_cfg.setFps(1);

    status = channelP->setImageConfig(lms_cfg);
    if (Status_Ok != status)
        std::cerr << "Failed to set FPS: " << Channel::statusString(status) << std::endl;

    if (mtu >= 1500)
        status = channelP->setMtu(mtu);
    else
        status = channelP->setBestMtu();
    if (Status_Ok != status)
        std::cerr << "Failed to set MTU: " << Channel::statusString(status) << std::endl;

    channelP->addIsolatedCallback(secondaryAppCallback, &userData);

    status = channelP->startStreams(Source_Secondary_App_Data_0);
    if (Status_Ok != status) {
        std::cerr << "startStreams failed: " << Channel::statusString(status) << std::endl;
        ret = -4;
        goto clean_out;
    }
    std::cout << "Streaming Source_Secondary_App_Data_0; ^C to stop" << std::endl;

    {
        uint64_t lastGroups = 0;
        uint64_t lastBytes  = 0;

        while (!doneG) {
            usleep(1000000);

            const uint64_t g  = userData.stats.groupsOk;
            const uint64_t b  = userData.stats.bytesOk;
            const double mbps = (double)(b - lastBytes) * 8.0 / 1e6;

            std::cout << "[stats] " << (g - lastGroups) << " grp/s, "
                      << std::fixed << std::setprecision(1) << mbps << " Mb/s"
                      << "  | total ok " << g
                      << ", bad " << userData.stats.groupsBad
                      << ", meta-mismatch " << userData.stats.metaMismatch
                      << ", skipped " << userData.stats.framesSkipped
                      << (userData.savePgms
                          ? (", pgms " + std::to_string(userData.stats.pgmsSaved)) : "")
                      << std::endl;

            lastGroups = g;
            lastBytes  = b;
        }

        std::cout << "Final: " << userData.stats.groupsOk << " groups ("
                  << userData.stats.imagesOk << " images, "
                  << userData.stats.bytesOk << "b) ok, "
                  << userData.stats.groupsBad << " bad, "
                  << userData.stats.metaMismatch << " meta mismatches, "
                  << userData.stats.framesSkipped << " skipped" << std::endl;

        if (userData.stats.groupsOk == 0) {
            std::cout << "No valid groups received. Checklist: is the tura pipeline running "
                      << "(sup/t6 up, tura_enbs nonzero)? Does the LMS log show "
                      << "'thermal: activated' and 'Starting secondary app stream'? "
                      << "If both yes, sniff for the meta message -- the client drops "
                      << "SecondaryAppData silently when no SecondaryAppMetadata with a "
                      << "matching frameId arrived first." << std::endl;
            ret = -5;
        }
    }

    status = channelP->stopStreams(Source_Secondary_App_Data_0);
    if (Status_Ok != status)
        std::cerr << "stopStreams failed: " << Channel::statusString(status) << std::endl;

    channelP->removeIsolatedCallback(secondaryAppCallback);

clean_out:

    status = channelP->secondaryAppDeactivate(THERMAL_APP_NAME);
    if (Status_Ok != status)
        std::cerr << "secondaryAppDeactivate failed: " << Channel::statusString(status) << std::endl;

    Channel::Destroy(channelP);
    return ret;
}
