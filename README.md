# foxglove_bridge (ROS 1)

ROS 1 Foxglove bridge. Connects ROS 1 topics, services, and parameters to
Foxglove clients over a local WebSocket server and, when enabled, the
Foxglove remote access gateway (LiveKit/WebRTC, handled entirely by the SDK).

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

The resulting image is a slim runtime (just the Noetic and bridge install
spaces plus their runtime dependencies); the Dockerfile's `bridge` stage
keeps the full build environment, and is what `make docker-test` builds and
runs in.

The Foxglove SDK is downloaded by CMake during the build as a pinned,
SHA-verified release zip (see the FetchContent block in CMakeLists.txt). The
Noetic sources are pinned too (noetic.rosinstall, one released tarball per
package), so image builds are reproducible.

## Running

The robot side needs no changes: a stock focal Noetic robot works as-is,
because the bridge speaks ordinary TCPROS.

With the robot's rosmaster on the same host, serve local Foxglove WebSocket
connections with:

```sh
docker run --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  foxglove-bridge-ros1
```

then connect Foxglove to `ws://<bridge-host>:8765`. With `--network host`
Docker does not publish ports, so for clients on other machines, open 8765 in
the bridge host's firewall.

For a robot elsewhere on the network, point `ROS_MASTER_URI` at the robot and
set `ROS_HOSTNAME` to an address of the bridge host that the robot can reach
(ROS 1 publishers connect back to subscribers).

To use the Foxglove remote access gateway instead, pass a device token and
enable it:

```sh
docker run --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  -e FOXGLOVE_DEVICE_TOKEN=<token> \
  foxglove-bridge-ros1 \
  rosrun foxglove_bridge foxglove_bridge _remote_access:=true
```

The gateway connection is outbound from the bridge, so no inbound ports need
to be open; the device then appears under Devices in the Foxglove app.

### Parameters

All parameters are private (`~`); pass them with `_name:=value` to `rosrun`,
or via the launch file. `device_token` also falls back to the
`FOXGLOVE_DEVICE_TOKEN` environment variable.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `port` | int | `8765` | Local WebSocket server port. |
| `address` | string | `0.0.0.0` | WebSocket server bind address. |
| `tls` | bool | `false` | Serve the local WebSocket over TLS (wss). |
| `certfile` | string | `""` | TLS certificate path (required when `tls` is true). |
| `keyfile` | string | `""` | TLS private key path (required when `tls` is true). |
| `remote_access` | bool | `false` | Connect to the Foxglove remote access gateway. Requires a remote-access build. |
| `device_token` | string | `""` | Device token for remote access (else `FOXGLOVE_DEVICE_TOKEN`). |
| `foxglove_api_url` | string | `""` | Override the Foxglove API URL (empty = SDK default). |
| `capabilities` | string[] | `[assets, clientPublish, connectionGraph, services, parameters, parametersSubscribe]` | Advertised ws-protocol capabilities. |
| `topic_whitelist` | string[] | `[".*"]` | Regexes of topics to bridge. |
| `service_whitelist` | string[] | `[".*"]` | Regexes of services to bridge. |
| `param_whitelist` | string[] | `[".*"]` | Regexes of parameters to expose. |
| `asset_uri_allowlist` | string[] | (see below) | Regexes of `package://` asset URIs `fetchAsset` may serve. |
| `max_update_ms` | int | `5000` | Upper bound of the master-poll backoff, in ms. |
| `subscription_queue_length` | int | `10` | ROS subscriber/publisher queue size. |
| `message_backlog_size` | int | `1024` | SDK per-client message backlog. |
| `service_type_retrieval_timeout_ms` | int | `250` | Timeout for probing a service's type from its server. |
| `service_call_timeout_ms` | int | `5000` | Deadline for a client-initiated service call (enforced at poll granularity). |
| `sysinfo` | bool | `false` | Publish CPU/memory stats. |
| `sysinfo_topic` | string | `/foxglove_bridge/sysinfo` | Topic for sysinfo stats. |
| `sysinfo_refresh_interval` | int | `500` | Sysinfo refresh interval, in ms. |
| `debug` | bool | `false` | Enable SDK debug logging. |

The bridge also reads the global `/use_sim_time` parameter at startup: when
set, it advertises the Time capability and broadcasts `/clock` to clients.

The default `asset_uri_allowlist` permits `package://` URIs ending in a
common mesh/description extension (dae, fbx, glb, gltf, jpeg, jpg, mtl, obj,
png, stl, tif, tiff, urdf, webp, xacro); see the source for the exact regex.

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
[examples/demo_robot_description](examples/demo_robot_description) is a
runnable demo of this: a minimal description package whose single-link URDF
references an STL mesh by `package://` URI.

## Examples

- [examples/image_publisher.py](examples/image_publisher.py) — dependency-free
  rospy node that publishes a scrolling color-bar `sensor_msgs/Image` test
  pattern; runs on a stock `ros:noetic` container.
- [examples/demo_robot_description](examples/demo_robot_description) — minimal
  description package (single-link URDF, so it renders without TF, plus an
  STL mesh) for exercising `fetchAsset`; see the sidecar section above.

## Testing

```sh
make docker-test
```

Runs the rostest-based smoke suite (`tests/smoke.test`) in the image: a master,
the bridge, and a gtest that exercises topics, latched replay, client publish,
service calls, parameter get/set/push, asset fetching, and time broadcast over
the ws-protocol, using an in-repo copy of the ws-protocol test client from the
ROS 2 bridge tests.

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

## Implementation notes

- **Schemas/md5sums** come from `ros_babel_fish`'s integrated description
  provider (disk lookup at advertise time), following the legacy
  `foxglove/ros-foxglove-bridge` design.
- **Topic/service/graph discovery** polls the master (`getTopicTypes`,
  `getSystemState`) with exponential backoff (100 ms doubling up to
  `~max_update_ms`, default 5000 ms).
- **Subscriptions** use `topic_tools::ShapeShifter` and forward raw serialized
  bytes; **client publishers** are created from `ros::AdvertiseOptions` with
  babel_fish-provided type info, and inbound messages are republished via a
  morphed ShapeShifter.
- **Services** are called generically: the type is probed from the service
  server's connection header (`service_utils.cpp`, ported from the legacy
  bridge), the md5 looked up via babel_fish, and raw bytes forwarded with a
  dynamic-traits `GenericService`.
- **Parameters** implement the transport manager's `ParameterBackend` over
  `ros::param`; subscriptions use the master's `subscribeParam` push mechanism
  via a second `ros::XMLRPCManager` serving a `paramUpdate` endpoint (legacy
  bridge pattern).

## Known limitations / TODOs

- Remote-access QoS classification for latched topics is observational: a
  topic is only classified Reliable after a latched publisher has been seen
  (ROS 1 reveals latching only in per-connection headers).
