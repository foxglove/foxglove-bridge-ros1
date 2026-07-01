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
# The final stage is a slim runtime image (the install spaces only). The
# `bridge` stage keeps the full build environment; `make docker-test` builds
# and uses that stage.
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

# ROS 1 python build tooling from PyPI (the ROS apt repo has no jammy/noetic),
# pinned for reproducible builds. empy must stay < 4: genmsg/em templates
# break with empy >= 4. pycryptodomex/python-gnupg are runtime deps of the
# rosbag python tools.
RUN pip3 install --no-cache-dir \
        vcstool==0.3.0 \
        catkin-pkg==1.1.0 \
        rospkg==1.6.1 \
        empy==3.3.4 \
        defusedxml==0.7.1 \
        netifaces==0.11.0 \
        pycryptodomex==3.23.0 \
        python-gnupg==0.5.6

# Pinned Noetic sources: one released tarball per package, checked into the
# repo so builds are reproducible. Regenerate with:
#   rosinstall_generator ros_comm topic_tools ros_babel_fish \
#       resource_retriever --rosdistro noetic --deps --tar > noetic.rosinstall
COPY noetic.rosinstall /ros_ws/noetic.rosinstall
RUN mkdir -p /ros_ws/src && vcs import /ros_ws/src < /ros_ws/noetic.rosinstall

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
# Stage 2: build foxglove_bridge. This stage keeps the full build environment
# (sources, build trees, compilers, test dependencies); `make docker-test`
# builds it via --target bridge and runs the test suite inside it.
# ---------------------------------------------------------------------------
FROM noetic-base AS bridge

# Test-only build dependencies (the smoke test's ws-protocol client).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libasio-dev \
        libwebsocketpp-dev \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# Pre-fetch the SDK release zip in a layer keyed only on CMakeLists.txt and
# package.xml: configuring the package without its sources runs the
# FetchContent download (with its SHA check) into FETCHCONTENT_BASE_DIR and
# then fails at add_library — by design, hence the `|| true` and the explicit
# check that the download landed. Source edits then rebuild the bridge
# without re-downloading the SDK, and the version/SHA pins stay
# single-sourced in CMakeLists.txt.
COPY CMakeLists.txt package.xml /tmp/sdk-prefetch/
RUN . /opt/ros/noetic/setup.sh \
    && mkdir -p /tmp/sdk-prefetch/build \
    && cd /tmp/sdk-prefetch/build \
    && (cmake .. -DFETCHCONTENT_BASE_DIR=/sdk/fetchcontent \
        > /tmp/sdk-prefetch.log 2>&1 || true) \
    && (test -d /sdk/fetchcontent/foxglove_sdk-src \
        || (cat /tmp/sdk-prefetch.log && false)) \
    && rm -rf /tmp/sdk-prefetch /tmp/sdk-prefetch.log

# The repo root is the package; .dockerignore keeps .git out of the copy.
COPY . /bridge_ws/src/foxglove_bridge

ARG FOXGLOVE_BRIDGE_REMOTE_ACCESS=ON
# .git is not in the build context, so the git hash for the bridge's startup
# banner must come in from outside (the Makefile passes it).
ARG FOXGLOVE_BRIDGE_GIT_HASH=

RUN . /opt/ros/noetic/setup.sh \
    && cd /bridge_ws \
    && catkin_make_isolated \
        --install --install-space /opt/foxglove \
        --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DFOXGLOVE_BRIDGE_REMOTE_ACCESS=${FOXGLOVE_BRIDGE_REMOTE_ACCESS} \
        -DFOXGLOVE_BRIDGE_GIT_HASH=${FOXGLOVE_BRIDGE_GIT_HASH} \
        -DFETCHCONTENT_BASE_DIR=/sdk/fetchcontent

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["rosrun", "foxglove_bridge", "foxglove_bridge"]

# ---------------------------------------------------------------------------
# Stage 3 (final): slim runtime image — the Noetic and bridge install spaces
# on a fresh base, without sources, build trees, compilers, or test-only
# packages. The explicit package list is the ldd closure of the install
# spaces (transitive dependencies resolve via apt); the runtime pip packages
# back the ROS 1 python tools (rosmaster, roslaunch, rosbag).
#
# ca-certificates and libbz2-1.0 are listed explicitly even though they would
# arrive transitively today (via python3-pip's dependency and libpython
# respectively): ca-certificates is data, not a linked library, so it is not
# in the ldd closure, and TLS (remote access, https assets) silently fails
# without it; libbz2 backs rosbag's bzip2 compression. Pinning both here keeps
# a future dependency change from quietly breaking them.
# ---------------------------------------------------------------------------
FROM ubuntu:22.04 AS runtime
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-yaml \
        python3-netifaces \
        libpython3.10 \
        libboost-chrono1.74.0 \
        libboost-filesystem1.74.0 \
        libboost-program-options1.74.0 \
        libboost-regex1.74.0 \
        libboost-thread1.74.0 \
        libbz2-1.0 \
        ca-certificates \
        libconsole-bridge1.0 \
        libcurl4 \
        libgpgme11 \
        liblog4cxx12 \
        liblz4-1 \
        libpocofoundation80 \
        libtinyxml2-9 \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir \
        catkin-pkg==1.1.0 \
        rospkg==1.6.1 \
        defusedxml==0.7.1 \
        pycryptodomex==3.23.0 \
        python-gnupg==0.5.6

COPY --from=bridge /opt/ros/noetic /opt/ros/noetic
COPY --from=bridge /opt/foxglove /opt/foxglove
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["rosrun", "foxglove_bridge", "foxglove_bridge"]
