#include "io/talos.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace io::talos {

namespace {

constexpr uint32_t kImageWidth = 1440;
constexpr uint32_t kImageHeight = 1080;
constexpr uint32_t kImageChannels = 3;
constexpr std::size_t kImageSize = static_cast<std::size_t>(kImageWidth) * kImageHeight * kImageChannels;
constexpr std::size_t kImagePoolSize = kImageSize * 3;
constexpr uint8_t kFlagNew = 0x80;
constexpr uint8_t kIndexMask = 0x03;
constexpr uint32_t kShmMagic = 0x54414C05u;
constexpr uint32_t kShmVersion = 2u;

struct alignas(32) ImageMeta {
  uint64_t seq;
  uint64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
  uint8_t buffer_id;
  uint8_t format;
  uint8_t _pad[6];
};

struct alignas(64) PoseMeta {
  uint64_t frame_seq;
  float position[3];
  float quaternion[4];
  uint64_t timestamp_ns;
  uint8_t _pad[16];
};

struct alignas(32) GimbalCmd {
  uint64_t timestamp_ns;
  float yaw_deg;
  float pitch_deg;
  float distance_m;
  uint8_t fire_advice;
  uint8_t _pad[11];
};

struct alignas(64) ImageTripleBuffer {
  std::atomic<uint8_t> state;
  uint8_t write_idx;
  uint8_t read_idx;
  uint8_t _pad1[61];
  ImageMeta slots[3];
};

struct alignas(64) PoseTripleBuffer {
  std::atomic<uint8_t> state;
  uint8_t write_idx;
  uint8_t read_idx;
  uint8_t _pad1[61];
  PoseMeta slots[3];
};

struct alignas(64) GimbalTripleBuffer {
  std::atomic<uint8_t> state;
  uint8_t write_idx;
  uint8_t read_idx;
  uint8_t _pad1[61];
  GimbalCmd slots[3];
};

struct alignas(64) ShmHeader {
  uint32_t magic;
  uint32_t version;
  uint64_t created_ns;
  uint64_t heartbeat_ns;
  uint32_t image_width;
  uint32_t image_height;
  uint8_t _pad[32];
};

struct ShmMetaRegion {
  ShmHeader header;
  ImageTripleBuffer image;
  PoseTripleBuffer poses[5];
  GimbalTripleBuffer gimbal_cmd;
  uint8_t _pad[1024];
};

struct MmapFile {
  int fd = -1;
  void* ptr = nullptr;
  std::size_t size = 0;
  ~MmapFile() {
    if (ptr != nullptr && ptr != MAP_FAILED) {
      munmap(ptr, size);
    }
    if (fd >= 0) {
      close(fd);
    }
  }
};

inline bool open_file_mmap(const char* path, std::size_t size, MmapFile& file) {
  int fd = open(path, O_RDWR, 0666);
  if (fd < 0) {
    return false;
  }

  void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (ptr == MAP_FAILED) {
    close(fd);
    return false;
  }

  file.fd = fd;
  file.ptr = ptr;
  file.size = size;
  return true;
}

inline uint8_t read_u8(const uint8_t* base, std::size_t offset) {
  return base[offset];
}

inline uint32_t read_u32(const uint8_t* base, std::size_t offset) {
  uint32_t v = 0;
  std::memcpy(&v, base + offset, sizeof(v));
  return v;
}

inline uint64_t read_u64(const uint8_t* base, std::size_t offset) {
  uint64_t v = 0;
  std::memcpy(&v, base + offset, sizeof(v));
  return v;
}

inline float read_f32(const uint8_t* base, std::size_t offset) {
  float v = 0.0f;
  std::memcpy(&v, base + offset, sizeof(v));
  return v;
}

inline bool consume_pose_channel(ShmMetaRegion& meta, std::size_t index) {
  if (index >= 5) {
    return false;
  }

  auto& pose = meta.poses[index];
  uint8_t expected = pose.state.load(std::memory_order_acquire);
  if ((expected & kFlagNew) == 0) {
    return false;
  }

  const uint8_t ready_idx = expected & kIndexMask;
  const uint8_t desired = pose.read_idx;
  while (!pose.state.compare_exchange_weak(
             expected,
             desired,
             std::memory_order_acq_rel,
             std::memory_order_acquire)) {
    if ((expected & kFlagNew) == 0) {
      return false;
    }
  }

  pose.read_idx = ready_idx;
  return true;
}

inline bool read_image_from_shm(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  MmapFile meta_file;
  if (!open_file_mmap("/tmp/talos_ipc_meta", sizeof(ShmMetaRegion), meta_file)) {
    return false;
  }

  auto* meta = static_cast<ShmMetaRegion*>(meta_file.ptr);
  if (meta->header.magic != kShmMagic || meta->header.version != kShmVersion) {
    return false;
  }

  auto& image = meta->image;
  uint8_t expected = image.state.load(std::memory_order_acquire);
  if ((expected & kFlagNew) == 0) {
    return false;
  }

  const uint8_t ready_idx = expected & kIndexMask;
  const uint8_t desired = image.read_idx;
  while (!image.state.compare_exchange_weak(
             expected,
             desired,
             std::memory_order_acq_rel,
             std::memory_order_acquire)) {
    if ((expected & kFlagNew) == 0) {
      return false;
    }
  }
  image.read_idx = ready_idx;

  const auto& img_slot = image.slots[ready_idx];
  const uint64_t ts_ns = img_slot.timestamp_ns;
  const std::size_t buffer_id = img_slot.buffer_id;

  MmapFile pool_file;
  if (!open_file_mmap("/tmp/talos_ipc_image_pool", kImagePoolSize, pool_file)) {
    return false;
  }

  const auto* pool = static_cast<const uint8_t*>(pool_file.ptr);
  const auto* data = pool + buffer_id * kImageSize;
  cv::Mat raw(1080, 1440, CV_8UC3, const_cast<uint8_t*>(data));
  img = raw.clone();

  for (std::size_t i = 0; i < 5; ++i) {
    consume_pose_channel(*meta, i);
  }

  const auto epoch = std::chrono::steady_clock::time_point{};
  timestamp = epoch + std::chrono::nanoseconds(static_cast<long long>(ts_ns));
  return true;
}

inline bool write_command_to_shm(double yaw_deg, double pitch_deg, double distance_m, bool fire_advice)
{
  MmapFile meta_file;
  if (!open_file_mmap("/tmp/talos_ipc_meta", 4096, meta_file)) {
    return false;
  }

  auto* meta = static_cast<uint8_t*>(meta_file.ptr);
  const uint32_t magic = read_u32(meta, 0);
  const uint32_t version = read_u32(meta, 4);
  if (magic != kShmMagic || version != kShmVersion) {
    return false;
  }

  constexpr std::size_t state_offset = 1536;
  constexpr std::size_t slots_offset = 1600;
  uint8_t* state_ptr = meta + state_offset;
  uint8_t& state = *state_ptr;
  uint8_t& write_idx = *(state_ptr + 1);

  // Match the Rust triple-buffer producer semantics exactly:
  // 1) write to the current producer slot
  // 2) mark the slot with FLAG_NEW
  // 3) advance the producer index using the previous state value
  const uint8_t previous_state = state;
  const std::size_t idx = write_idx & kIndexMask;
  auto* slot = reinterpret_cast<GimbalCmd*>(meta + slots_offset + idx * sizeof(GimbalCmd));

  slot->timestamp_ns = 0;
  slot->yaw_deg = static_cast<float>(yaw_deg);
  slot->pitch_deg = static_cast<float>(pitch_deg);
  slot->distance_m = static_cast<float>(distance_m);
  slot->fire_advice = fire_advice ? 1u : 0u;

  state = static_cast<uint8_t>(idx | kFlagNew);
  write_idx = static_cast<uint8_t>(previous_state & kIndexMask);
  return true;
}

}  // namespace

bool connected() {
  MmapFile meta_file;
  return open_file_mmap("/tmp/talos_ipc_meta", 4096, meta_file);
}

bool read_latest_image(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) {
  return read_image_from_shm(img, timestamp);
}

bool read_latest_pose(Eigen::Quaterniond & q, Eigen::Vector3d & position) {
  MmapFile meta_file;
  if (!open_file_mmap("/tmp/talos_ipc_meta", sizeof(ShmMetaRegion), meta_file)) {
    return false;
  }

  auto* meta = static_cast<ShmMetaRegion*>(meta_file.ptr);
  if (meta->header.magic != kShmMagic || meta->header.version != kShmVersion) {
    return false;
  }

  const auto& gimbal = meta->poses[0];
  const uint8_t state = gimbal.state.load(std::memory_order_acquire);
  if ((state & kFlagNew) == 0) {
    return false;
  }

  const uint8_t ready_idx = state & kIndexMask;
  const auto& ready = gimbal.slots[ready_idx];

  q = Eigen::Quaterniond(ready.quaternion[3], ready.quaternion[0], ready.quaternion[1], ready.quaternion[2]);
  position = Eigen::Vector3d(ready.position[0], ready.position[1], ready.position[2]);

  consume_pose_channel(*meta, 0);
  return true;
}

bool write_gimbal_cmd(double yaw_deg, double pitch_deg, double distance_m, bool fire_advice) {
  return write_command_to_shm(yaw_deg, pitch_deg, distance_m, fire_advice);
}

}  // namespace io::talos
