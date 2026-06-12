# foxglove_bridge (ROS 1)

ROS 1 Foxglove bridge. Connects ROS 1 topics, services, and parameters to
Foxglove clients over a local WebSocket server and, when enabled, the
Foxglove remote access gateway (LiveKit/WebRTC — handled entirely by the
SDK). Deliberately a parallel implementation to the ROS 2 `foxglove_bridge`
(which lives in the [foxglove-sdk](https://github.com/foxglove/foxglove-sdk)
repository): the two packages share a name — like the legacy
`ros-foxglove-bridge`, which served both ROS versions under one name — but
no code; the transport-facing layer (transport_manager, capabilities,
logging, types, utils) is a copy of the equivalent code in the ROS 2 bridge.

## Building

Noetic is only released for Ubuntu 20.04 (focal, glibc 2.31), but the SDK's
remote access support requires glibc >= 2.35. The supported build is therefore
a from-source Noetic on Ubuntu 22.04 (jammy), via Docker. Two jammy
compatibility substitutions are made (see the Dockerfile): rosconsole comes
from the ROS One (ros-o) fork, which supports jammy's log4cxx 0.12, and
ros_babel_fish is built as C++17 (log4cxx 0.12 headers require it). From the
repo root:

```sh
make docker-build
```

The Foxglove SDK is downloaded by CMake during the build as a pinned,
SHA-verified release zip (see the FetchContent block in CMakeLists.txt).

### Testing against a locally-built SDK

To build and test against a locally-modified SDK instead of the pinned
release, run `make build-cpp-dist` in a foxglove-sdk checkout and point
`FOXGLOVE_CPP_SDK_DIR` at the resulting `cpp/dist` tree:

```sh
make docker-test-local-sdk FOXGLOVE_CPP_SDK_DIR=/path/to/foxglove-sdk/cpp/dist
```

This reuses the prebuilt image and mounts the SDK dist and the current
working tree into it, rebuilding just the bridge inside the container — so
the slow Noetic-from-source image stage is not repeated, and local edits to
both the SDK and the bridge are picked up without rebuilding the image.

Run against an external rosmaster (e.g. a robot running a stock focal
Noetic — the bridge interoperates over TCPROS; the robot side needs no
changes):

```sh
docker run --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  -e FOXGLOVE_DEVICE_TOKEN=<token> \
  foxglove-bridge-ros1 \
  rosrun foxglove_bridge foxglove_bridge _remote_access:=true
```

### Assets in a sidecar deployment

`fetchAsset` resolves `package://` URIs with resource_retriever against the
*bridge container's* filesystem, not the robot's. When the bridge runs as a
sidecar next to an existing robot, mount the robot's description packages
(URDF meshes etc.) into the container under `/opt/foxglove/share`, which is
already on the bridge's `ROS_PACKAGE_PATH`:

```sh
  -v /path/to/my_robot_description:/opt/foxglove/share/my_robot_description:ro
```

(The URDF itself usually travels as the `robot_description` parameter and
needs no mount; only the assets it references do.)

## Testing

```sh
make docker-test
```

Runs the rostest-based smoke suite (`tests/smoke.test`) in the image: a master,
the bridge, and a gtest that exercises topics, latched replay, client publish,
service calls, parameter get/set/push, asset fetching, and time broadcast over
the ws-protocol, using the test client shared with the ROS 2 bridge tests.

## Implementation notes

- **Schemas/md5sums** come from `ros_babel_fish`'s integrated description
  provider (disk lookup at advertise time), following the legacy
  `foxglove/ros-foxglove-bridge` design.
- **Topic/service/graph discovery** polls the master (`getTopicTypes`,
  `getSystemState`) with exponential backoff (100ms doubling to
  `~max_update_ms`, default 5000).
- **Subscriptions** use `topic_tools::ShapeShifter` and forward raw serialized
  bytes; **client publishers** are created from `ros::AdvertiseOptions` with
  babel_fish-provided type info, and inbound messages are republished via a
  morphed ShapeShifter.
- **Services** are called generically: the type is probed from the service
  server's connection header (`service_utils.cpp`, ported from the legacy
  bridge), the md5 looked up via babel_fish, and raw bytes forwarded with a
  dynamic-traits `GenericService`.
- **Parameters** implement the transport manager's `ParameterBackend` over `ros::param`;
  subscriptions use the master's `subscribeParam` push mechanism via a second
  `ros::XMLRPCManager` serving a `paramUpdate` endpoint (legacy bridge
  pattern).

## Known limitations / TODOs

- Remote-access QoS classification for latched topics is observational: a
  topic is only classified Reliable after a latched publisher has been seen
  (ROS 1 reveals latching only in per-connection headers).
