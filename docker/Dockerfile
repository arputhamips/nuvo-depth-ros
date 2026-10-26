ARG ROS_IMAGE=ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410
FROM ${ROS_IMAGE} AS build
SHELL ["/bin/bash", "-c"]
ARG NUVO_CLIENT_WITH_HTTPS=ON
ARG BUILD_JOBS=2
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build python3-colcon-common-extensions \
    nlohmann-json3-dev libboost-system-dev libssl-dev libc-ares-dev \
    ros-jazzy-rclcpp-components ros-jazzy-sensor-msgs ros-jazzy-diagnostic-msgs \
    ros-jazzy-rosgraph-msgs ros-jazzy-ament-cmake-gtest ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY nuvo_depth_client_vendor src/nuvo_depth_client_vendor
COPY nuvo_camera_msgs src/nuvo_camera_msgs
COPY nuvo_camera src/nuvo_camera
COPY nuvo_bringup src/nuvo_bringup
RUN source /opt/ros/jazzy/setup.bash && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --merge-install \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DNUVO_CLIENT_WITH_HTTPS=${NUVO_CLIENT_WITH_HTTPS} -DBUILD_TESTING=ON \
    && colcon test --merge-install --return-code-on-test-failure && colcon test-result --verbose
FROM ${ROS_IMAGE}
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-system1.83.0 libssl3t64 libc-ares2 \
    ros-jazzy-rclcpp-components ros-jazzy-sensor-msgs ros-jazzy-diagnostic-msgs \
    ros-jazzy-rosgraph-msgs ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY --from=build /ws/install /ws/install
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY docker/entrypoint.sh /nuvo-entrypoint.sh
RUN chmod 755 /nuvo-entrypoint.sh && useradd --uid 10001 --create-home nuvo
USER nuvo
ENTRYPOINT ["/nuvo-entrypoint.sh"]
CMD ["ros2", "launch", "nuvo_bringup", "camera.launch.py"]
