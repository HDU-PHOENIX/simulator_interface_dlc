#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace {

constexpr uint32_t SHM_MAGIC = 0x54414C05u;
constexpr uint32_t SHM_VERSION = 2u;
constexpr uint8_t FLAG_NEW = 0x80u;
constexpr uint8_t INDEX_MASK = 0x03u;
constexpr std::size_t POSE_CHANNEL_COUNT = 5;

constexpr const char* SHM_NAME_META = "/tmp/talos_ipc_meta";
constexpr const char* SHM_NAME_IMAGE_POOL = "/tmp/talos_ipc_image_pool";

constexpr uint32_t IMAGE_WIDTH = 1440;
constexpr uint32_t IMAGE_HEIGHT = 1080;
constexpr uint32_t IMAGE_CHANNELS = 3;
constexpr std::size_t IMAGE_SIZE = static_cast<std::size_t>(IMAGE_WIDTH) * IMAGE_HEIGHT * IMAGE_CHANNELS;

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

struct alignas(64) CameraInfo {
    uint64_t timestamp_ns;
    double fx;
    double fy;
    double cx;
    double cy;
    double distortion[5];
    uint32_t width;
    uint32_t height;
    uint8_t _pad[24];
};

struct alignas(64) ChassisObservation {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    float dt_s;
    float v_body[2];
    float wz_radps;
    float wheel_linear_mps[4];
    float wheel_angular_radps[4];
    float a_body[2];
    float alpha_z_radps2;
    float rpy_rad[3];
    float gyro_xyz_radps[3];
    float accel_xyz_mps2[3];
    uint8_t _pad[16];
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

struct alignas(64) GroundTruthTarget {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint8_t team;
    uint8_t armor_label;
    uint8_t is_outpost;
    uint8_t _pad1;
    float position[3];
    float vyaw;
    float yaw;
    uint8_t _pad[24];
};

struct alignas(64) GroundTruthRune {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint8_t team;
    uint8_t rune_mode;
    uint8_t mechanism_state;
    uint8_t _pad1;
    float r_center_odom[3];
    float radius;
    float current_angle;
    float v_roll;
    int32_t direction;
    float sin_amplitude;
    float sin_omega;
    float sin_phase;
    float sin_offset;
    float relative_time;
    int32_t blade_id;
    uint8_t target_activations[5];
    uint8_t _pad[20];
};

constexpr std::size_t GROUND_TRUTH_MAX_TARGETS = 16;
constexpr std::size_t GROUND_TRUTH_MAX_RUNES = 4;

struct alignas(64) GroundTruthBatch {
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint32_t target_count;
    uint32_t rune_count;
    GroundTruthTarget targets[GROUND_TRUTH_MAX_TARGETS];
    GroundTruthRune runes[GROUND_TRUTH_MAX_RUNES];
    uint8_t _pad[64];
};

struct alignas(64) RuntimeState {
    uint64_t timestamp_ns;
    uint8_t following;
    uint8_t _pad[55];
};

struct ShmMetaRegion {
    ShmHeader header;
    ImageTripleBuffer image;
    PoseTripleBuffer poses[5];
    GimbalTripleBuffer gimbal_cmd;
    CameraInfo camera_info;
    ChassisObservation chassis_observation;
    GroundTruthBatch ground_truth;
    RuntimeState runtime_state;
};

struct SharedMemCtx {
    int meta_fd = -1;
    int image_fd = -1;
    ShmMetaRegion* meta = nullptr;
    uint8_t* image_pool = nullptr;
    std::size_t meta_size = 0;
    std::size_t image_pool_size = 0;
};

bool map_file(const char* path, std::size_t size, int& fd, void*& mapped) {
    fd = open(path, O_RDWR);
    if (fd < 0) {
        std::cerr << "open failed: " << path << " errno=" << errno << std::endl;
        return false;
    }

    struct stat st = {};
    if (fstat(fd, &st) != 0) {
        std::cerr << "fstat failed: " << path << std::endl;
        close(fd);
        return false;
    }

    if (static_cast<std::size_t>(st.st_size) < size) {
        std::cerr << "shared memory too small: " << path << " expected>= " << size
                  << " actual= " << static_cast<std::size_t>(st.st_size) << std::endl;
        close(fd);
        return false;
    }

    mapped = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED) {
        std::cerr << "mmap failed: " << path << " errno=" << errno << std::endl;
        close(fd);
        return false;
    }

    return true;
}

bool consume_pose_channel(ShmMetaRegion& meta, std::size_t index) {
    if (index >= POSE_CHANNEL_COUNT) {
        return false;
    }

    auto& pose = meta.poses[index];
    uint8_t expected = pose.state.load(std::memory_order_acquire);
    if ((expected & FLAG_NEW) == 0) {
        return false;
    }

    const uint8_t ready_idx = expected & INDEX_MASK;
    const uint8_t desired = pose.read_idx;

    while (!pose.state.compare_exchange_weak(
               expected,
               desired,
               std::memory_order_acq_rel,
               std::memory_order_acquire)) {
        if ((expected & FLAG_NEW) == 0) {
            return false;
        }
    }

    pose.read_idx = ready_idx;
    return true;
}

bool consume_image_channel(ShmMetaRegion& meta, std::size_t& ready_idx) {
    uint8_t expected = meta.image.state.load(std::memory_order_acquire);
    if ((expected & FLAG_NEW) == 0) {
        return false;
    }

    ready_idx = expected & INDEX_MASK;
    const uint8_t desired = meta.image.read_idx;

    while (!meta.image.state.compare_exchange_weak(
               expected,
               desired,
               std::memory_order_acq_rel,
               std::memory_order_acquire)) {
        if ((expected & FLAG_NEW) == 0) {
            return false;
        }
    }

    meta.image.read_idx = ready_idx;
    return true;
}

bool try_read_image(SharedMemCtx& ctx, cv::Mat& out_frame, uint64_t& out_seq, uint64_t& out_ts_ns) {
    if (ctx.meta == nullptr || ctx.image_pool == nullptr) {
        return false;
    }

    std::size_t ready_idx = 0;
    if (!consume_image_channel(*ctx.meta, ready_idx)) {
        return false;
    }

    const ImageMeta& image_meta = ctx.meta->image.slots[ready_idx];
    if (image_meta.width == 0 || image_meta.height == 0 || image_meta.buffer_id > 2) {
        return false;
    }

    const std::size_t stride = static_cast<std::size_t>(image_meta.width) *
                              static_cast<std::size_t>(image_meta.height) * IMAGE_CHANNELS;
    const uint8_t* src = ctx.image_pool + static_cast<std::size_t>(image_meta.buffer_id) * stride;

    cv::Mat frame(static_cast<int>(image_meta.height), static_cast<int>(image_meta.width), CV_8UC3);
    std::memcpy(frame.data, src, stride);

    out_frame = frame.clone();
    out_seq = image_meta.seq;
    out_ts_ns = image_meta.timestamp_ns;

    for (std::size_t i = 0; i < POSE_CHANNEL_COUNT; ++i) {
        consume_pose_channel(*ctx.meta, i);
    }

    return true;
}

void print_pose_summary(const ShmMetaRegion& meta) {
    for (std::size_t i = 0; i < POSE_CHANNEL_COUNT; ++i) {
        const auto& pose = meta.poses[i];
        const uint8_t state = pose.state.load(std::memory_order_acquire);
        const bool has_new = (state & FLAG_NEW) != 0;
        if (!has_new) {
            continue;
        }

        const uint8_t slot = state & INDEX_MASK;
        const auto& ready = pose.slots[slot];
        std::cout << "pose[" << i << "] state=0x" << std::hex << static_cast<int>(state) << std::dec
                  << " slot=" << static_cast<int>(slot)
                  << " frame_seq=" << ready.frame_seq
                  << " ts_ns=" << ready.timestamp_ns << std::endl;
    }
}

}  // namespace

int main() {
    SharedMemCtx ctx;
    const std::size_t meta_size = sizeof(ShmMetaRegion);
    const std::size_t image_pool_size = IMAGE_SIZE * 3;

    std::cout << "waiting for shared memory: " << SHM_NAME_META << " and " << SHM_NAME_IMAGE_POOL << std::endl;
    while (!map_file(SHM_NAME_META, meta_size, ctx.meta_fd, reinterpret_cast<void*&>(ctx.meta)) ||
           !map_file(SHM_NAME_IMAGE_POOL, image_pool_size, ctx.image_fd, reinterpret_cast<void*&>(ctx.image_pool))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    ctx.meta_size = meta_size;
    ctx.image_pool_size = image_pool_size;

    if (ctx.meta->header.magic != SHM_MAGIC) {
        std::cerr << "bad magic: expected 0x" << std::hex << SHM_MAGIC << " got 0x" << ctx.meta->header.magic << std::dec << std::endl;
        munmap(ctx.meta, ctx.meta_size);
        close(ctx.meta_fd);
        munmap(ctx.image_pool, ctx.image_pool_size);
        close(ctx.image_fd);
        return 1;
    }

    if (ctx.meta->header.version != SHM_VERSION) {
        std::cerr << "bad version: expected " << SHM_VERSION << " got " << ctx.meta->header.version << std::endl;
        munmap(ctx.meta, ctx.meta_size);
        close(ctx.meta_fd);
        munmap(ctx.image_pool, ctx.image_pool_size);
        close(ctx.image_fd);
        return 1;
    }

    std::cout << "header ok: magic=0x" << std::hex << ctx.meta->header.magic << std::dec
              << " version=" << ctx.meta->header.version
              << " width=" << ctx.meta->header.image_width
              << " height=" << ctx.meta->header.image_height
              << " heartbeat_ns=" << ctx.meta->header.heartbeat_ns << std::endl;

    cv::namedWindow("talos_ipc_image_pose_receiver", cv::WINDOW_NORMAL);
    cv::resizeWindow("talos_ipc_image_pose_receiver", 1280, 960);

    uint64_t last_seq = 0;
    int frame_counter = 0;
    auto start = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat frame;
        uint64_t seq = 0;
        uint64_t ts_ns = 0;

        if (try_read_image(ctx, frame, seq, ts_ns)) {
            if (seq != last_seq) {
                last_seq = seq;
                ++frame_counter;
                cv::imshow("talos_ipc_image_pose_receiver", frame);
                std::cout << "frame seq=" << seq << " ts_ns=" << ts_ns
                          << " size=" << frame.cols << "x" << frame.rows << std::endl;
                print_pose_summary(*ctx.meta);
            }
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= 1) {
            std::cout << "fps=" << frame_counter << std::endl;
            frame_counter = 0;
            start = now;
        }
    }

    cv::destroyAllWindows();

    munmap(ctx.image_pool, ctx.image_pool_size);
    close(ctx.image_fd);
    munmap(ctx.meta, ctx.meta_size);
    close(ctx.meta_fd);

    return 0;
}
