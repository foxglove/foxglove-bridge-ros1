// Smoke test for the ROS 1 foxglove_bridge, run under rostest (see smoke.test,
// which launches the master, the bridge with use_sim_time, and this gtest).
// Uses the in-repo ws-protocol test client (tests/client).

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <roscpp/GetLoggers.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <websocketpp/config/asio_client.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include "client/test_client.hpp"

namespace {

constexpr char URI[] = "ws://localhost:9876";
constexpr uint16_t PORT = 9876;

using Client = foxglove::test::Client<websocketpp::config::asio_client>;
using namespace std::chrono_literals;

// Discovery runs on the bridge's master poll, which backs off to 5s.
constexpr auto DISCOVERY_TIMEOUT = 20s;
constexpr auto DEFAULT_TIMEOUT = 10s;

std::vector<uint8_t> serializeRos1String(const std::string& text) {
  std::vector<uint8_t> buffer(4 + text.size());
  foxglove::test::WriteUint32LE(buffer.data(), static_cast<uint32_t>(text.size()));
  std::memcpy(buffer.data() + 4, text.data(), text.size());
  return buffer;
}

std::string deserializeRos1String(const std::vector<uint8_t>& buffer) {
  if (buffer.size() < 4) {
    throw std::runtime_error("Buffer too small for a ros1 string");
  }
  const uint32_t length = foxglove::test::ReadUint32LE(buffer.data());
  if (buffer.size() < 4 + length) {
    throw std::runtime_error("Buffer too small for the declared string length");
  }
  return std::string(reinterpret_cast<const char*>(buffer.data() + 4), length);
}

uint64_t readUint64LE(const uint8_t* buf) {
  uint64_t value = 0;
  for (int i = 7; i >= 0; --i) {
    value = (value << 8) | buf[i];
  }
  return value;
}

// rostest launches the bridge and this test concurrently; wait for the
// bridge's WebSocket server to accept connections before the tests run. Uses a
// plain TCP probe: a failed websocketpp connection leaves the test client in a
// state its destructor cannot handle.
bool waitForServer(uint16_t port, std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd >= 0) {
      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_port = htons(port);
      inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
      const int result = ::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
      ::close(fd);
      if (result == 0) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return false;
}

}  // namespace

TEST(SmokeTest, TopicSubscription) {
  ros::NodeHandle nh;
  auto publisher = nh.advertise<std_msgs::String>("/smoke/chatter", 1, /*latch=*/true);
  std_msgs::String rosMsg;
  rosMsg.data = "hello smoke test";
  publisher.publish(rosMsg);

  auto client = std::make_shared<Client>();
  auto channelFuture = client->waitForChannel("/smoke/chatter");
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto channel = channelFuture.get();
  EXPECT_EQ(channel.encoding, "ros1");
  EXPECT_EQ(channel.schemaName, "std_msgs/String");
  EXPECT_FALSE(channel.schema.empty());

  const foxglove::test::SubscriptionId subscriptionId = 1;
  auto msgFuture = client->waitForChannelMsg(subscriptionId);
  client->subscribe({{subscriptionId, channel.id}});
  ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(DEFAULT_TIMEOUT));
  EXPECT_EQ(deserializeRos1String(msgFuture.get()), "hello smoke test");
}

TEST(SmokeTest, LatchedTopicReplayToLateSubscriber) {
  ros::NodeHandle nh;
  auto publisher = nh.advertise<std_msgs::String>("/smoke/latched", 1, /*latch=*/true);
  std_msgs::String rosMsg;
  rosMsg.data = "latched state";
  publisher.publish(rosMsg);

  // First client subscribes; the bridge creates the shared ROS subscription
  // and the latched message arrives via the natural ROS latch resend.
  auto client1 = std::make_shared<Client>();
  auto channelFuture = client1->waitForChannel("/smoke/latched");
  ASSERT_EQ(std::future_status::ready, client1->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto channel = channelFuture.get();

  auto msg1Future = client1->waitForChannelMsg(1);
  client1->subscribe({{1, channel.id}});
  ASSERT_EQ(std::future_status::ready, msg1Future.wait_for(DEFAULT_TIMEOUT));
  EXPECT_EQ(deserializeRos1String(msg1Future.get()), "latched state");

  // Second client subscribes while the first holds the ROS subscription open;
  // it must receive the message from the bridge's latched-message cache.
  auto client2 = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client2->connect(URI).wait_for(DEFAULT_TIMEOUT));
  auto msg2Future = client2->waitForChannelMsg(2);
  client2->subscribe({{2, channel.id}});
  ASSERT_EQ(std::future_status::ready, msg2Future.wait_for(DEFAULT_TIMEOUT));
  EXPECT_EQ(deserializeRos1String(msg2Future.get()), "latched state");
}

TEST(SmokeTest, ClientPublish) {
  ros::NodeHandle nh;
  auto promise = std::make_shared<std::promise<std::string>>();
  auto future = promise->get_future();
  auto fulfilled = std::make_shared<std::atomic<bool>>(false);
  auto subscriber = nh.subscribe<std_msgs::String>(
    "/smoke/from_client",
    10,
    [promise, fulfilled](const std_msgs::String::ConstPtr& msg) {
      if (!fulfilled->exchange(true)) {
        promise->set_value(msg->data);
      }
    }
  );

  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  foxglove::test::ClientAdvertisement advertisement;
  advertisement.channelId = 1;
  advertisement.topic = "/smoke/from_client";
  advertisement.encoding = "ros1";
  advertisement.schemaName = "std_msgs/String";
  client->advertise({advertisement});

  // Publish repeatedly: ROS 1 subscriber connection setup takes a moment.
  const auto payload = serializeRos1String("hello from client");
  const auto deadline = std::chrono::steady_clock::now() + DEFAULT_TIMEOUT;
  std::future_status status = std::future_status::timeout;
  while (status != std::future_status::ready && std::chrono::steady_clock::now() < deadline) {
    client->publish(advertisement.channelId, payload.data(), payload.size());
    status = future.wait_for(500ms);
  }
  ASSERT_EQ(std::future_status::ready, status);
  EXPECT_EQ(future.get(), "hello from client");
  client->unadvertise({advertisement.channelId});
}

TEST(SmokeTest, ServiceCall) {
  // The test node's own roscpp-provided get_loggers service.
  const std::string serviceName = ros::this_node::getName() + "/get_loggers";

  auto client = std::make_shared<Client>();
  auto serviceFuture = client->waitForService(serviceName);
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, serviceFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto service = serviceFuture.get();
  EXPECT_EQ(service.type, "roscpp/GetLoggers");

  foxglove::test::ServiceRequest request;
  request.serviceId = service.id;
  request.callId = 1;
  request.encoding = "ros1";
  // GetLoggersRequest is empty.

  auto responseFuture = client->waitForServiceResponse();
  client->sendServiceRequest(request);
  ASSERT_EQ(std::future_status::ready, responseFuture.wait_for(DEFAULT_TIMEOUT));
  const auto response = responseFuture.get();
  EXPECT_EQ(response.serviceId, service.id);
  EXPECT_EQ(response.callId, request.callId);
  ASSERT_GE(response.data.size(), 4u);
  const auto numLoggers =
    foxglove::test::ReadUint32LE(reinterpret_cast<const uint8_t*>(response.data.data()));
  // The log4cxx backend provides a real logger hierarchy.
  EXPECT_GE(numLoggers, 1u);
}

TEST(SmokeTest, ServiceCallTimeout) {
  // A service that never responds must produce a serviceCallFailure within
  // the bridge's deadline (service_call_timeout_ms, 1000 in smoke.test plus
  // up to one master-poll period of sweep granularity) instead of hanging
  // the call forever.
  ros::NodeHandle nh;
  auto release = std::make_shared<std::promise<void>>();
  auto releaseFuture = release->get_future().share();
  boost::function<bool(roscpp::GetLoggers::Request&, roscpp::GetLoggers::Response&)> hungCallback =
    [releaseFuture](roscpp::GetLoggers::Request&, roscpp::GetLoggers::Response&) {
      // Block until the test releases us; bounded as a teardown backstop.
      releaseFuture.wait_for(30s);
      return true;
    };
  auto server = nh.advertiseService("/smoke/hung_service", hungCallback);

  auto client = std::make_shared<Client>();
  auto serviceFuture = client->waitForService("/smoke/hung_service");
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, serviceFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto service = serviceFuture.get();

  auto failurePromise = std::make_shared<std::promise<std::string>>();
  auto failureFuture = failurePromise->get_future();
  auto fulfilled = std::make_shared<std::atomic<bool>>(false);
  client->setTextMessageHandler([failurePromise, fulfilled](const std::string& payload) {
    const auto msg = nlohmann::json::parse(payload);
    if (msg.value("op", "") == "serviceCallFailure" && !fulfilled->exchange(true)) {
      failurePromise->set_value(msg.value("message", ""));
    }
  });

  foxglove::test::ServiceRequest request;
  request.serviceId = service.id;
  request.callId = 99;
  request.encoding = "ros1";
  client->sendServiceRequest(request);

  ASSERT_EQ(std::future_status::ready, failureFuture.wait_for(DEFAULT_TIMEOUT));
  EXPECT_NE(failureFuture.get().find("timed out"), std::string::npos);

  // Unblock the hung handler so it doesn't stall suite teardown.
  release->set_value();
}

TEST(SmokeTest, Parameters) {
  ros::NodeHandle nh;
  nh.setParam("/smoke/param", "initial");

  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  // Get.
  auto getFuture = client->waitForParameters("get-1");
  client->getParameters({"/smoke/param"}, "get-1");
  ASSERT_EQ(std::future_status::ready, getFuture.wait_for(DEFAULT_TIMEOUT));
  auto params = getFuture.get();
  ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(params[0].name(), "/smoke/param");
  EXPECT_EQ(params[0].value()->get<std::string>(), "initial");

  // Set, with echo-back of the applied value. (Parameter is move-only, so no
  // initializer list.)
  std::vector<foxglove::Parameter> parametersToSet;
  parametersToSet.emplace_back("/smoke/param", "updated");
  auto setFuture = client->waitForParameters("set-1");
  client->setParameters(parametersToSet, "set-1");
  ASSERT_EQ(std::future_status::ready, setFuture.wait_for(DEFAULT_TIMEOUT));
  params = setFuture.get();
  ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(params[0].value()->get<std::string>(), "updated");
  std::string rosValue;
  EXPECT_TRUE(nh.getParam("/smoke/param", rosValue));
  EXPECT_EQ(rosValue, "updated");

  // Subscribe; an out-of-band change must be pushed by the master. The
  // subscription travels client -> bridge -> master before a set becomes
  // observable, so set the parameter repeatedly until the push arrives
  // (mirroring the ClientPublish pattern) rather than sleeping a fixed time.
  client->subscribeParameterUpdates({"/smoke/param"});
  auto updateFuture = client->waitForParameters();
  const auto deadline = std::chrono::steady_clock::now() + DEFAULT_TIMEOUT;
  std::future_status updateStatus = std::future_status::timeout;
  while (updateStatus != std::future_status::ready && std::chrono::steady_clock::now() < deadline) {
    nh.setParam("/smoke/param", "pushed");
    updateStatus = updateFuture.wait_for(500ms);
  }
  ASSERT_EQ(std::future_status::ready, updateStatus);
  params = updateFuture.get();
  ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(params[0].name(), "/smoke/param");
  EXPECT_EQ(params[0].value()->get<std::string>(), "pushed");
}

TEST(SmokeTest, FetchAsset) {
  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  // Allowlisted asset, shipped as a test fixture in this package.
  auto responseFuture = client->waitForFetchAssetResponse();
  client->fetchAsset("package://foxglove_bridge/tests/assets/smoke.urdf", 1);
  ASSERT_EQ(std::future_status::ready, responseFuture.wait_for(DEFAULT_TIMEOUT));
  auto response = responseFuture.get();
  EXPECT_EQ(response.requestId, 1u);
  ASSERT_EQ(response.status, foxglove::test::FetchAssetStatus::Success);
  const std::string content(
    reinterpret_cast<const char*>(response.data.data()), response.data.size()
  );
  EXPECT_NE(content.find("smoke-bot"), std::string::npos);

  // Literal path traversal must be rejected.
  responseFuture = client->waitForFetchAssetResponse();
  client->fetchAsset("package://foxglove_bridge/../../../etc/passwd", 2);
  ASSERT_EQ(std::future_status::ready, responseFuture.wait_for(DEFAULT_TIMEOUT));
  response = responseFuture.get();
  EXPECT_EQ(response.requestId, 2u);
  EXPECT_EQ(response.status, foxglove::test::FetchAssetStatus::Error);

  // Percent-encoded traversal must also be rejected: resource_retriever
  // decodes %2e, so a literal-only check would let this through. This URI
  // ends in an allowlisted extension (so the allowlist passes) and, decoded,
  // traverses out of the package and back into smoke.urdf — without the
  // decode-aware ".." check it would resolve and succeed.
  responseFuture = client->waitForFetchAssetResponse();
  client->fetchAsset("package://foxglove_bridge/%2e%2e/foxglove_bridge/tests/assets/smoke.urdf", 3);
  ASSERT_EQ(std::future_status::ready, responseFuture.wait_for(DEFAULT_TIMEOUT));
  response = responseFuture.get();
  EXPECT_EQ(response.requestId, 3u);
  EXPECT_EQ(response.status, foxglove::test::FetchAssetStatus::Error);
}

// NOTE: /use_sim_time is set globally by smoke.test, so once this test
// publishes /clock, the bridge broadcasts TIME frames to every connected
// client for the remainder of the process — regardless of test order. That
// is safe because all binary-message handlers in the test client check their
// opcode before parsing; new handlers must do the same.
TEST(SmokeTest, TimeBroadcast) {
  ros::NodeHandle nh;
  auto clockPublisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  auto promise = std::make_shared<std::promise<uint64_t>>();
  auto future = promise->get_future();
  auto fulfilled = std::make_shared<std::atomic<bool>>(false);
  client->setBinaryMessageHandler([promise, fulfilled](const uint8_t* data, size_t dataLength) {
    if (dataLength >= 9 &&
        static_cast<foxglove::test::ServerBinaryOpcode>(data[0]) ==
          foxglove::test::ServerBinaryOpcode::TIME &&
        !fulfilled->exchange(true)) {
      promise->set_value(readUint64LE(data + 1));
    }
  });

  // Publish a fake sim time until the broadcast arrives.
  rosgraph_msgs::Clock clockMsg;
  clockMsg.clock = ros::Time(12345, 500000000);
  const auto deadline = std::chrono::steady_clock::now() + DEFAULT_TIMEOUT;
  std::future_status status = std::future_status::timeout;
  while (status != std::future_status::ready && std::chrono::steady_clock::now() < deadline) {
    clockPublisher.publish(clockMsg);
    status = future.wait_for(500ms);
  }
  ASSERT_EQ(std::future_status::ready, status);
  EXPECT_EQ(future.get(), clockMsg.clock.toNSec());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "smoke_test");

  if (!waitForServer(PORT, std::chrono::seconds(30))) {
    std::cerr << "Bridge WebSocket server did not come up on port " << PORT << "\n";
    return 1;
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();
  const int result = RUN_ALL_TESTS();
  spinner.stop();

  return result;
}
