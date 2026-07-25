#ifndef __THERMAL_WIRE_H__
#define __THERMAL_WIRE_H__

#include <stdint.h>

namespace thermal_wire {

// T6TC
static const uint32_t GROUP_MAGIC   = 0x43543654u;
static const uint16_t GROUP_VERSION = 1;
static const int MAX_IMAGERS = 8;

//
// group_header_t.payload_type
enum : uint16_t {
  PAYLOAD_FRAME_GROUP = 1,
  PAYLOAD_CALIBRATION = 2,
};

//
// image_desc_t.flags
enum : uint8_t {
  IMG_FLAG_RECTIFIED = (1u << 0),
};

#pragma pack(push, 1)

//
// One per image in the group, immediately following group_header_t.
struct image_desc_t {
  uint32_t offset;          // byte offset of pixel data from payload start
  uint32_t length;          // pixel data length in bytes
  uint16_t width;
  uint16_t height;
  uint8_t  bits_per_pixel;  // 8 or 16
  uint8_t  flags;           // IMG_FLAG_*
  uint8_t  imager_id;       // physical imager position, 0..MAX_IMAGERS-1
  uint8_t  reserved;
};

//
// Payload layout:
//   [group_header_t][image_desc_t * num_images][pixel data...]
struct group_header_t {
  uint32_t magic;
  uint16_t version;
  uint16_t payload_type;
  uint32_t header_length;       // sizeof(group_header_t) + num_images*sizeof(image_desc_t)
  int64_t  frame_id;            // pipeline frame id (matches SecondaryAppHeader)
  uint32_t time_seconds;        // PTP time if ptp_locked, else pipeline time
  uint32_t time_microseconds;
  uint8_t  ptp_locked;          // 1: time fields are PTP
  uint8_t  num_images;
  uint8_t  reserved[2];
  uint32_t imager_enable_mask;  // raw tura enable mask at capture time
};

//
// Control messages (client -> firmware over SecondaryAppControl, <=1KB)
enum : uint16_t {
  CTRL_SET_RECTIFIED       = 1,  // value: 0 = raw, 1 = rectified
  CTRL_SET_BITS_PER_PIXEL  = 2,  // value: 8 or 16
  CTRL_SET_POST_PROC       = 3,  // value: tura enable mask
};

struct control_t {
  uint32_t magic;    // GROUP_MAGIC
  uint16_t version;  // GROUP_VERSION
  uint16_t cmd;      // CTRL_*
  uint32_t value;
};

//
// Config response (firmware -> client over SecondaryAppConfig, <=1KB)
struct config_t {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved0;
  uint8_t  rectified;           // current send-side selection
  uint8_t  bits_per_pixel;      // current capture mode
  uint8_t  num_imagers_max;     // populated imager slots on this unit
  uint8_t  reserved1;
  uint32_t imager_enable_mask;  // current tura enables
  uint16_t width;               // current per-image dimensions
  uint16_t height;
};

#pragma pack(pop)

static_assert(sizeof(image_desc_t)   == 16, "image_desc_t wire size changed");
static_assert(sizeof(group_header_t) == 36, "group_header_t wire size changed");
static_assert(sizeof(control_t)      == 12, "control_t wire size changed");
static_assert(sizeof(config_t)       == 20, "config_t wire size changed");

} // namespace thermal_wire

#endif /* end of include guard: __THERMAL_WIRE_H__ */
