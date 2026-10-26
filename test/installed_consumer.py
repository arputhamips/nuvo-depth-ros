"""Build a downstream C++ consumer using only the installed ROS package.

Run after sourcing install/setup.bash. No source-tree include paths or manual
library paths are allowed: find_package must provide the public interface.
"""
from pathlib import Path
import subprocess
import tempfile


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / 'CMakeLists.txt').write_text('''\
cmake_minimum_required(VERSION 3.25)
project(robot_camera_consumer LANGUAGES CXX)
find_package(nuvo_camera REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE nuvo_camera::nuvo_camera_component)
''')
        (root / 'main.cpp').write_text('''\
#include <nuvo_camera/camera.hpp>
#include <nuvo_camera/conversion.hpp>
#include <nuvo_camera/policy.hpp>
#include <stdexcept>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/robot/sensors", "-r", "__node:=depth"});
    options.parameter_overrides({rclcpp::Parameter("source", "replay"),
                                 rclcpp::Parameter("recording", "unused.nvr")});
    nuvo_camera::NuvoCamera camera(options);
    if (std::string(camera.get_fully_qualified_name()) != "/robot/sensors/depth") return 1;
    // Lazy startup must not open the intentionally absent recording.
    nuvo_camera::FramePolicy policy({});
    if (policy.snapshot(1).available) return 2;
    try {
      (void)nuvo_camera::convert(nuvo::client::Frame{}, std_msgs::msg::Header{});
      return 3;
    } catch (const std::invalid_argument&) {}
  }
  rclcpp::shutdown();
}
''')
        build = root / 'build'
        subprocess.run(['cmake', '-S', str(root), '-B', str(build)], check=True)
        subprocess.run(['cmake', '--build', str(build), '-j2'], check=True)
        subprocess.run([str(build / 'consumer')], check=True, timeout=20)
    print('Installed downstream C++ consumer passed')


if __name__ == '__main__':
    main()
