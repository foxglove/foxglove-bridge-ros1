# ROS 1 Noetic foxglove_bridge on Ubuntu 22.04 (jammy).
#
# Noetic is only released for focal (glibc 2.31), but the SDK's remote access
# support requires glibc >= 2.35, so this image builds a minimal Noetic stack
# (ros_comm + topic_tools + ros_babel_fish) from source on jammy and then
# builds foxglove_bridge against it.
#
# The Foxglove SDK is downloaded by CMake as a pinned, SHA-verified release zip
# during the build (see the FetchContent block in the package's CMakeLists.txt),
# so no SDK pre-build is needed. Build from the repo root:
#   docker build -t foxglove-bridge-ros1 .
# or via the Makefile:
#   make docker-build
#
# Run against an external rosmaster and the Foxglove platform:
#   docker run --rm --network host \
#     -e ROS_MASTER_URI=http://localhost:11311 \
#     -e FOXGLOVE_DEVICE_TOKEN=... \
#     foxglove-bridge-ros1 \
#     rosrun foxglove_bridge foxglove_bridge _remote_access:=true

# ---------------------------------------------------------------------------
# Stage 1: Noetic ros_comm (+ topic_tools, ros_babel_fish) from source.
# ---------------------------------------------------------------------------
FROM ubuntu:22.04 AS noetic-base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        curl \
        ca-certificates \
        python3-dev \
        python3-pip \
        python3-yaml \
        python3-setuptools \
        libboost-chrono-dev \
        libboost-date-time-dev \
        libboost-filesystem-dev \
        libboost-program-options-dev \
        libboost-regex-dev \
        libboost-system-dev \
        libboost-thread-dev \
        libbz2-dev \
        libconsole-bridge-dev \
        libcurl4-openssl-dev \
        libgpgme-dev \
        libgtest-dev \
        liblog4cxx-dev \
        liblz4-dev \
        libpoco-dev \
        libssl-dev \
        libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS 1 python build tooling from PyPI (the ROS apt repo has no jammy/noetic).
# empy is pinned: genmsg/em templates break with empy >= 4.
# pycryptodomex/python-gnupg are runtime deps of the rosbag python tools.
RUN pip3 install --no-cache-dir \
        rosinstall_generator \
        vcstool \
        catkin-pkg \
        rospkg \
        empy==3.3.4 \
        defusedxml \
        netifaces \
        pycryptodomex \
        python-gnupg

RUN mkdir -p /ros_ws/src \
    && rosinstall_generator ros_comm topic_tools ros_babel_fish resource_retriever \
        --rosdistro noetic --deps --tar > /ros_ws/noetic.rosinstall \
    && vcs import /ros_ws/src < /ros_ws/noetic.rosinstall

# jammy's log4cxx 0.12 changed LoggerPtr to a std::shared_ptr, which noetic's
# rosconsole (1.14.3) predates. Swap in the ROS One (ros-o) fork of rosconsole,
# which carries compatibility patches for log4cxx 0.11-0.13 on the same 1.14.3
# lineage. Pinned to the obese-devel commit with the 0.11-0.13 compat patch.
ARG ROS_O_ROSCONSOLE_COMMIT=e3753eec58bf4e76012d019fd307349f94d1d0be
RUN rm -rf /ros_ws/src/rosconsole \
    && mkdir -p /ros_ws/src/rosconsole \
    && curl -fsSL "https://github.com/ros-o/rosconsole/archive/${ROS_O_ROSCONSOLE_COMMIT}.tar.gz" \
        | tar -xz --strip-components=1 -C /ros_ws/src/rosconsole

# log4cxx 0.12 headers require C++17 (std::shared_mutex), and ros/console.h
# now exposes them to everything downstream. jammy's gcc defaults to C++17, so
# only packages that pin an older standard need fixing: ros_babel_fish pins
# -std=c++11.
RUN find /ros_ws/src/ros_babel_fish -name CMakeLists.txt \
        -exec sed -i 's/-std=c++11/-std=c++17/' {} +

RUN cd /ros_ws \
    && python3 src/catkin/bin/catkin_make_isolated \
        --install --install-space /opt/ros/noetic \
        -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# Stage 2: foxglove_bridge.
# ---------------------------------------------------------------------------
FROM noetic-base AS bridge

# Test-only build dependencies (the smoke test's ws-protocol client).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libasio-dev \
        libwebsocketpp-dev \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# The repo root is the package; .dockerignore keeps .git out of the copy.
COPY . /bridge_ws/src/foxglove_bridge

ARG FOXGLOVE_BRIDGE_REMOTE_ACCESS=ON

RUN . /opt/ros/noetic/setup.sh \
    && cd /bridge_ws \
    && catkin_make_isolated \
        --install --install-space /opt/foxglove \
        --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DFOXGLOVE_BRIDGE_REMOTE_ACCESS=${FOXGLOVE_BRIDGE_REMOTE_ACCESS}

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["rosrun", "foxglove_bridge", "foxglove_bridge"]
