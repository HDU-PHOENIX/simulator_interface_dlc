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

namespace {

constexpr uint32_t SHM_MAGIC = 0x54414C05u;
constexpr uint32_t SHM_VERSION = 2u;
constexpr uint8_t FLAG_NEW = 0x80u;
constexpr uint8_t INDEX_MASK = 0x03u;

constexpr const char* SHM_NAME_META = "/tmp/talos_ipc_meta";

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

bool map_meta_region(const char* path, std::size_t size, void*& mapped, int& fd) {
    fd = open(path, O_RDWR);
    if (fd < 0) {
        std::cerr << "open failed: " << path << " errno=" << errno << std::endl;
        return false;
    }

    struct stat st = {};
    if (fstat(fd, &st) != 0) {
        std::cerr << "fstat failed on " << path << std::endl;
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

uint64_t now_ns() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

void init_if_needed(ShmMetaRegion* meta) {
    const uint8_t s = meta->gimbal_cmd.state.load(std::memory_order_acquire);
    if (s == 0u) {
        meta->gimbal_cmd.state.store(1u, std::memory_order_release);
        meta->gimbal_cmd.write_idx = 0u;
        meta->gimbal_cmd.read_idx = 2u;
    }
}

void publish_gimbal_cmd(ShmMetaRegion* meta, float yaw_deg, float pitch_deg, float distance_m) {
    uint8_t slot = meta->gimbal_cmd.write_idx;
    GimbalCmd& cmd = meta->gimbal_cmd.slots[slot];

    cmd.timestamp_ns = now_ns();
    cmd.yaw_deg = yaw_deg;
    cmd.pitch_deg = pitch_deg;
    cmd.distance_m = distance_m;
    cmd.fire_advice = 0;

    const uint8_t old_state = meta->gimbal_cmd.state.exchange((slot | FLAG_NEW), std::memory_order_acq_rel);
    meta->gimbal_cmd.write_idx = old_state & INDEX_MASK;
}

void print_state(const ShmMetaRegion& meta) {
    const uint8_t state = meta.gimbal_cmd.state.load(std::memory_order_acquire);
    const uint8_t slot = state & INDEX_MASK;
    const bool has_new = (state & FLAG_NEW) != 0;
    std::cout << "state=0x" << std::hex << static_cast<int>(state) << std::dec
              << " has_new=" << (has_new ? "yes" : "no")
              << " slot=" << static_cast<int>(slot)
              << " write_idx=" << static_cast<int>(meta.gimbal_cmd.write_idx)
              << " read_idx=" << static_cast<int>(meta.gimbal_cmd.read_idx)
              << std::endl;
}

}  // namespace

int main() {
    void* mapped = nullptr;
    int fd = -1;
    const std::size_t meta_size = sizeof(ShmMetaRegion);

    std::cout << "waiting for /tmp/talos_ipc_meta ..." << std::endl;
    while (!map_meta_region(SHM_NAME_META, meta_size, mapped, fd)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    auto* meta = static_cast<ShmMetaRegion*>(mapped);

    if (meta->header.magic != SHM_MAGIC) {
        std::cerr << "bad magic: expected 0x" << std::hex << SHM_MAGIC << " got 0x" << meta->header.magic << std::dec << std::endl;
        munmap(mapped, meta_size);
        close(fd);
        return 1;
    }

    if (meta->header.version != SHM_VERSION) {
        std::cerr << "bad version: expected " << SHM_VERSION << " got " << meta->header.version << std::endl;
        munmap(mapped, meta_size);
        close(fd);
        return 1;
    }

    std::cout << "shared memory ok: magic=0x" << std::hex << meta->header.magic << std::dec
              << " version=" << meta->header.version
              << " image_size=" << meta->header.image_width << "x" << meta->header.image_height
              << std::endl;

    init_if_needed(meta);

    float yaw_deg = 0.0f;
    const float yaw_step = 2.5f;
    const float pitch_deg = 0.0f;
    const float distance_m = 3.0f;

    std::cout << "starting continuous gimbal yaw rotation" << std::endl;
    std::cout << "F5 in simulator must be enabled to accept auto-aim commands" << std::endl;

    int counter = 0;
    while (true) {
        const uint8_t published_slot = meta->gimbal_cmd.write_idx;
        publish_gimbal_cmd(meta, yaw_deg, pitch_deg, distance_m);

        const auto& cmd = meta->gimbal_cmd.slots[published_slot];
        if (counter % 10 == 0) {
            std::cout << "send[" << counter << "] slot=" << static_cast<int>(published_slot)
                      << " yaw=" << yaw_deg
                      << " pitch=" << pitch_deg
                      << " distance=" << distance_m
                      << " fire=" << static_cast<int>(cmd.fire_advice) << std::endl;
            print_state(*meta);
        }

        yaw_deg += yaw_step;
        if (yaw_deg > 360.0f) {
            yaw_deg -= 360.0f;
        }

        ++counter;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    munmap(mapped, meta_size);
    close(fd);
    return 0;
}
