ARG ROS_IMAGE=ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410
ARG NUVO_CLIENT_WITH_HTTPS=ON
FROM ${ROS_IMAGE} AS build
SHELL ["/bin/bash", "-c"]
ARG NUVO_CLIENT_WITH_HTTPS
ARG BUILD_JOBS=2
RUN case "${NUVO_CLIENT_WITH_HTTPS}" in ON|OFF) ;; *) exit 2 ;; esac \
    && dpkg-query -W -f='${Package}\t${Version}\n' > /tmp/base-packages.tsv \
    && https_deps=() \
    && if [[ "${NUVO_CLIENT_WITH_HTTPS}" == ON ]]; then \
         https_deps=(libboost-system-dev libssl-dev libc-ares-dev); fi \
    && printf '%s\n' "${https_deps[@]}" > /tmp/requested-https-packages.txt \
    && apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build python3-colcon-common-extensions \
    nlohmann-json3-dev "${https_deps[@]}" \
    ros-jazzy-rclcpp-components ros-jazzy-sensor-msgs ros-jazzy-diagnostic-msgs \
    ros-jazzy-stereo-msgs ros-jazzy-rosgraph-msgs ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-python ros-jazzy-ament-cmake-pytest ros-jazzy-rmw-cyclonedds-cpp \
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
ARG NUVO_CLIENT_WITH_HTTPS
RUN dpkg-query -W -f='${Package}\t${Version}\n' > /tmp/base-packages.tsv \
    && https_deps=() \
    && if [[ "${NUVO_CLIENT_WITH_HTTPS}" == ON ]]; then \
         https_deps=(libboost-system1.83.0 libssl3t64 libc-ares2); fi \
    && printf '%s\n' "${https_deps[@]}" > /tmp/requested-https-packages.txt \
    && apt-get update && apt-get install -y --no-install-recommends \
    "${https_deps[@]}" \
    ros-jazzy-rclcpp-components ros-jazzy-sensor-msgs ros-jazzy-diagnostic-msgs \
    ros-jazzy-stereo-msgs ros-jazzy-rosgraph-msgs ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ARG VCS_REF=unknown
ARG DRIVER_VERSION=0.1.1
LABEL org.nuvo.client.https="${NUVO_CLIENT_WITH_HTTPS}" \
      org.nuvo.ros.launch-controls="1" \
      org.opencontainers.image.source="https://github.com/arputhamips/nuvo-depth-ros" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.version="${DRIVER_VERSION}" \
      org.opencontainers.image.licenses="GPL-3.0-or-later"
COPY LICENSE NOTICE /usr/share/doc/nuvo-depth-ros/
RUN mkdir -p /usr/share/nuvo-build \
    && mv /tmp/base-packages.tsv /usr/share/nuvo-build/runtime-base-packages.tsv \
    && mv /tmp/requested-https-packages.txt /usr/share/nuvo-build/runtime-requested-https-packages.txt
COPY --from=build /tmp/base-packages.tsv /usr/share/nuvo-build/build-base-packages.tsv
COPY --from=build /tmp/requested-https-packages.txt /usr/share/nuvo-build/build-requested-https-packages.txt
WORKDIR /ws
COPY --from=build /ws/install /ws/install
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///ws/install/share/nuvo_bringup/config/cyclonedds.xml
COPY docker/entrypoint.sh /nuvo-entrypoint.sh
RUN chmod 755 /nuvo-entrypoint.sh && useradd --uid 10001 --create-home nuvo
USER nuvo
ENTRYPOINT ["/nuvo-entrypoint.sh"]
CMD ["ros2", "launch", "nuvo_bringup", "camera.launch.py"]
