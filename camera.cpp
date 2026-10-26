#include "camera.hpp"

#include <stdexcept>
#include <thread>

#include "hikrobot/hikrobot.hpp"
#include "io/talos.hpp"
#include "mindvision/mindvision.hpp"
#include "simulator/simulator.hpp"
#include "tools/yaml.hpp"

namespace io
{
TalosCamera::TalosCamera() = default;

void TalosCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  for (int attempt = 0; ; ++attempt) {
    if (io::talos::read_latest_image(img, timestamp)) {
      return;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("TalosCamera: failed to read latest image after retry window");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");

  if (camera_name == "mindvision") {
    auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else if (camera_name == "simulator") {
    camera_ = std::make_unique<SimulatorCamera>();
  }

  else if (camera_name == "talos") {
    camera_ = std::make_unique<TalosCamera>();
  }

  else {
    throw std::runtime_error("Unknown camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
