// Smoke test for the ROS 1 foxglove_bridge, run under rostest (see smoke.test,
// which launches the master, the bridge with use_sim_time, and this gtest).
// Uses the in-repo ws-protocol test client (tests/client).

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <roscpp/GetLoggers.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <websocketpp/config/asio_client.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
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

// Publish `payload` on a client channel until `future` resolves (ROS 1
// subscriber connection setup takes a moment) or DEFAULT_TIMEOUT passes.
template<typename T>
std::future_status publishUntilReceived(
  Client& client, foxglove::test::ClientChannelId channelId, const std::vector<uint8_t>& payload,
  std::future<T>& future
) {
  const auto deadline = std::chrono::steady_clock::now() + DEFAULT_TIMEOUT;
  std::future_status status = std::future_status::timeout;
  while (status != std::future_status::ready && std::chrono::steady_clock::now() < deadline) {
    client.publish(channelId, payload.data(), payload.size());
    status = future.wait_for(500ms);
  }
  return status;
}

// A ROS subscriber on `topic` whose future resolves with the first message.
struct StringSink {
  std::shared_ptr<std::promise<std::string>> promise =
    std::make_shared<std::promise<std::string>>();
  std::future<std::string> future = promise->get_future();
  ros::Subscriber subscriber;

  StringSink(ros::NodeHandle& nh, const std::string& topic) {
    auto fulfilled = std::make_shared<std::atomic<bool>>(false);
    auto p = promise;
    subscriber = nh.subscribe<std_msgs::String>(
      topic,
      10,
      [p, fulfilled](const std_msgs::String::ConstPtr& msg) {
        if (!fulfilled->exchange(true)) {
          p->set_value(msg->data);
        }
      }
    );
  }
};

bool topicHasPublisher(const std::string& topic) {
  ros::master::V_TopicInfo topics;
  if (!ros::master::getTopics(topics)) {
    throw std::runtime_error("getTopics failed");
  }
  return std::any_of(topics.begin(), topics.end(), [&](const auto& info) {
    return info.name == topic;
  });
}

// Whether `node` is registered with the master as a subscriber of `topic`.
bool nodeSubscribesTo(const std::string& node, const std::string& topic) {
  XmlRpc::XmlRpcValue request = ros::this_node::getName();
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  if (!ros::master::execute("getSystemState", request, response, payload, false)) {
    throw std::runtime_error("getSystemState failed");
  }
  // payload: [publishers, subscribers, services], each [[name, [nodes...]], ...].
  XmlRpc::XmlRpcValue& subscribers = payload[1];
  for (int i = 0; i < subscribers.size(); ++i) {
    if (static_cast<std::string>(subscribers[i][0]) != topic) {
      continue;
    }
    XmlRpc::XmlRpcValue& nodes = subscribers[i][1];
    for (int j = 0; j < nodes.size(); ++j) {
      if (static_cast<std::string>(nodes[j]) == node) {
        return true;
      }
    }
  }
  return false;
}

// Polls the master until `node` no longer subscribes to `topic`.
bool waitForNodeUnsubscribed(
  const std::string& node, const std::string& topic, std::chrono::seconds timeout
) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!nodeSubscribesTo(node, topic)) {
      return true;
    }
    std::this_thread::sleep_for(100ms);
  }
  return false;
}

// Waits for a serviceCallFailure for `callId`; resolves with the message.
std::future<std::string> waitForServiceCallFailure(Client& client, uint32_t callId) {
  auto promise = std::make_shared<std::promise<std::string>>();
  auto future = promise->get_future();
  auto fulfilled = std::make_shared<std::atomic<bool>>(false);
  client.setTextMessageHandler([promise, fulfilled, callId](const std::string& payload) {
    const auto msg = nlohmann::json::parse(payload);
    if (msg.value("op", "") == "serviceCallFailure" && msg.value("callId", 0u) == callId &&
        !fulfilled->exchange(true)) {
      promise->set_value(msg.value("message", ""));
    }
  });
  return future;
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

TEST(SmokeTest, ResubscribeAfterLastUnsubscribe) {
  // When the last client unsubscribes, the bridge tears down the shared ROS
  // subscription and its latched-message cache. A subsequent subscriber gets
  // a fresh ROS subscription, and the latched message must again arrive from
  // ROS itself.
  ros::NodeHandle nh;
  auto publisher = nh.advertise<std_msgs::String>("/smoke/resubscribe", 1, /*latch=*/true);
  std_msgs::String rosMsg;
  rosMsg.data = "resubscribed";
  publisher.publish(rosMsg);

  auto client = std::make_shared<Client>();
  auto channelFuture = client->waitForChannel("/smoke/resubscribe");
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto channel = channelFuture.get();

  auto msg1Future = client->waitForChannelMsg(1);
  client->subscribe({{1, channel.id}});
  ASSERT_EQ(std::future_status::ready, msg1Future.wait_for(DEFAULT_TIMEOUT));
  EXPECT_EQ(deserializeRos1String(msg1Future.get()), "resubscribed");
  client->unsubscribe({1});

  // Wait for the master to drop the bridge's subscriber, so the next subscribe
  // can't be served from a lingering latched-message cache.
  ASSERT_TRUE(waitForNodeUnsubscribed("/foxglove_bridge", "/smoke/resubscribe", DEFAULT_TIMEOUT));

  auto msg2Future = client->waitForChannelMsg(2);
  client->subscribe({{2, channel.id}});
  ASSERT_EQ(std::future_status::ready, msg2Future.wait_for(DEFAULT_TIMEOUT));
  EXPECT_EQ(deserializeRos1String(msg2Future.get()), "resubscribed");
}

TEST(SmokeTest, TopicRemovedWhenPublisherGoes) {
  // A channel must be unadvertised once its last publisher goes away, even
  // while a client is subscribed to it.
  ros::NodeHandle nh;
  auto publisher = nh.advertise<std_msgs::String>("/smoke/vanishing", 1, /*latch=*/true);
  std_msgs::String rosMsg;
  rosMsg.data = "here for now";
  publisher.publish(rosMsg);

  auto client = std::make_shared<Client>();
  auto channelFuture = client->waitForChannel("/smoke/vanishing");
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto channel = channelFuture.get();

  auto msgFuture = client->waitForChannelMsg(1);
  client->subscribe({{1, channel.id}});
  ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(DEFAULT_TIMEOUT));

  auto unadvertisePromise = std::make_shared<std::promise<void>>();
  auto unadvertiseFuture = unadvertisePromise->get_future();
  auto fulfilled = std::make_shared<std::atomic<bool>>(false);
  client->setTextMessageHandler(
    [unadvertisePromise, fulfilled, channelId = channel.id](const std::string& payload) {
      const auto msg = nlohmann::json::parse(payload);
      if (msg.value("op", "") != "unadvertise") {
        return;
      }
      for (const auto& id : msg["channelIds"]) {
        if (id.get<foxglove::test::ChannelId>() == channelId && !fulfilled->exchange(true)) {
          unadvertisePromise->set_value();
        }
      }
    }
  );

  publisher.shutdown();
  EXPECT_EQ(std::future_status::ready, unadvertiseFuture.wait_for(DISCOVERY_TIMEOUT))
    << "channel for /smoke/vanishing was not unadvertised after its publisher left";
}

TEST(SmokeTest, TopicTypeChangeWhileSubscribed) {
  // Re-advertising a topic with a different type replaces its channel. This
  // exercises channel removal with a live client subscription: the bridge
  // must close the channel (firing onUnsubscribe for the subscriber) without
  // deadlocking, then advertise and serve the new channel.
  ros::NodeHandle nh;
  auto stringPublisher = nh.advertise<std_msgs::String>("/smoke/retyped", 1, /*latch=*/true);
  std_msgs::String stringMsg;
  stringMsg.data = "as a string";
  stringPublisher.publish(stringMsg);

  auto client = std::make_shared<Client>();
  auto channelFuture = client->waitForChannel("/smoke/retyped");
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto oldChannel = channelFuture.get();
  ASSERT_EQ(oldChannel.schemaName, "std_msgs/String");

  auto msgFuture = client->waitForChannelMsg(1);
  client->subscribe({{1, oldChannel.id}});
  ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(DEFAULT_TIMEOUT));

  // Track the old channel's unadvertisement and the new channel's
  // advertisement, which may arrive in either order.
  auto unadvertisePromise = std::make_shared<std::promise<void>>();
  auto unadvertiseFuture = unadvertisePromise->get_future();
  auto newChannelPromise = std::make_shared<std::promise<foxglove::test::Channel>>();
  auto newChannelFuture = newChannelPromise->get_future();
  auto unadvertised = std::make_shared<std::atomic<bool>>(false);
  auto advertised = std::make_shared<std::atomic<bool>>(false);
  client->setTextMessageHandler([=, oldId = oldChannel.id](const std::string& payload) {
    const auto msg = nlohmann::json::parse(payload);
    const auto op = msg.value("op", "");
    if (op == "unadvertise") {
      for (const auto& id : msg["channelIds"]) {
        if (id.get<foxglove::test::ChannelId>() == oldId && !unadvertised->exchange(true)) {
          unadvertisePromise->set_value();
        }
      }
    } else if (op == "advertise") {
      for (const auto& channel : msg["channels"].get<std::vector<foxglove::test::Channel>>()) {
        if (channel.topic == "/smoke/retyped" && channel.schemaName == "std_msgs/Int32" &&
            !advertised->exchange(true)) {
          newChannelPromise->set_value(channel);
        }
      }
    }
  });

  stringPublisher.shutdown();
  auto intPublisher = nh.advertise<std_msgs::Int32>("/smoke/retyped", 1, /*latch=*/true);
  std_msgs::Int32 intMsg;
  intMsg.data = 1234;
  intPublisher.publish(intMsg);

  ASSERT_EQ(std::future_status::ready, unadvertiseFuture.wait_for(DISCOVERY_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, newChannelFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto newChannel = newChannelFuture.get();
  EXPECT_NE(newChannel.id, oldChannel.id);

  auto intFuture = client->waitForChannelMsg(2);
  client->subscribe({{2, newChannel.id}});
  ASSERT_EQ(std::future_status::ready, intFuture.wait_for(DEFAULT_TIMEOUT));
  const auto data = intFuture.get();
  ASSERT_EQ(data.size(), 4u);
  EXPECT_EQ(foxglove::test::ReadUint32LE(data.data()), 1234u);
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

TEST(SmokeTest, InvalidClientAdvertisementsRejected) {
  // Advertisements the bridge cannot serve must not create ROS publishers,
  // and must not break the client's connection or the bridge.
  ros::NodeHandle nh;
  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  // Unsupported message encoding.
  foxglove::test::ClientAdvertisement jsonAd;
  jsonAd.channelId = 1;
  jsonAd.topic = "/smoke/rejected_json";
  jsonAd.encoding = "json";
  jsonAd.schemaName = "std_msgs/String";

  // Unknown message type.
  foxglove::test::ClientAdvertisement unknownTypeAd;
  unknownTypeAd.channelId = 2;
  unknownTypeAd.topic = "/smoke/rejected_unknown_type";
  unknownTypeAd.encoding = "ros1";
  unknownTypeAd.schemaName = "no_such_pkg/NoSuchType";

  // A valid advertisement, followed by a second one reusing its channel ID.
  foxglove::test::ClientAdvertisement validAd;
  validAd.channelId = 3;
  validAd.topic = "/smoke/accepted";
  validAd.encoding = "ros1";
  validAd.schemaName = "std_msgs/String";
  foxglove::test::ClientAdvertisement duplicateAd = validAd;
  duplicateAd.topic = "/smoke/rejected_duplicate";

  StringSink acceptedSink(nh, validAd.topic);

  client->advertise({jsonAd});
  client->advertise({unknownTypeAd});
  client->advertise({validAd});
  client->advertise({duplicateAd});

  // Publishing on the rejected JSON channel must be dropped.
  const std::string jsonPayload = R"({"data": "should be dropped"})";
  client->publish(
    jsonAd.channelId, reinterpret_cast<const uint8_t*>(jsonPayload.data()), jsonPayload.size()
  );

  // The valid channel still works, and still publishes on its original topic.
  // Client messages are handled in order, so once this arrives, every
  // advertisement above has been processed.
  const auto payload = serializeRos1String("still works");
  ASSERT_EQ(
    std::future_status::ready,
    publishUntilReceived(*client, validAd.channelId, payload, acceptedSink.future)
  );
  EXPECT_EQ(acceptedSink.future.get(), "still works");

  EXPECT_FALSE(topicHasPublisher(jsonAd.topic));
  EXPECT_FALSE(topicHasPublisher(unknownTypeAd.topic));
  EXPECT_FALSE(topicHasPublisher(duplicateAd.topic));
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

TEST(SmokeTest, ServiceCallErrors) {
  // Each failure mode must produce a serviceCallFailure for the right call.
  ros::NodeHandle nh;
  boost::function<bool(roscpp::GetLoggers::Request&, roscpp::GetLoggers::Response&)>
    failingCallback = [](roscpp::GetLoggers::Request&, roscpp::GetLoggers::Response&) {
      return false;
    };
  auto failingServer = nh.advertiseService("/smoke/failing_service", failingCallback);

  auto client = std::make_shared<Client>();
  auto serviceFuture = client->waitForService("/smoke/failing_service");
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));
  ASSERT_EQ(std::future_status::ready, serviceFuture.wait_for(DISCOVERY_TIMEOUT));
  const auto failingService = serviceFuture.get();

  // The ROS service handler reports failure.
  {
    auto failureFuture = waitForServiceCallFailure(*client, 1);
    foxglove::test::ServiceRequest request;
    request.serviceId = failingService.id;
    request.callId = 1;
    request.encoding = "ros1";
    client->sendServiceRequest(request);
    ASSERT_EQ(std::future_status::ready, failureFuture.wait_for(DEFAULT_TIMEOUT));
    EXPECT_NE(failureFuture.get().find("Failed to call service"), std::string::npos);
  }

  // A request in an encoding other than ros1.
  {
    auto failureFuture = waitForServiceCallFailure(*client, 2);
    foxglove::test::ServiceRequest request;
    request.serviceId = failingService.id;
    request.callId = 2;
    request.encoding = "json";
    const std::string body = "{}";
    for (const char c : body) {
      request.data.push_back(static_cast<std::byte>(c));
    }
    client->sendServiceRequest(request);
    ASSERT_EQ(std::future_status::ready, failureFuture.wait_for(DEFAULT_TIMEOUT));
  }

  // A service ID the bridge never advertised.
  {
    auto failureFuture = waitForServiceCallFailure(*client, 3);
    foxglove::test::ServiceRequest request;
    request.serviceId = 999999;
    request.callId = 3;
    request.encoding = "ros1";
    client->sendServiceRequest(request);
    ASSERT_EQ(std::future_status::ready, failureFuture.wait_for(DEFAULT_TIMEOUT));
  }
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

TEST(SmokeTest, BridgeOwnParametersNotExposed) {
  // The bridge runs as node "foxglove_bridge", so its private parameters
  // (including the remote-access device_token set in smoke.test) live under
  // "/foxglove_bridge/". None of them may be returned to a client, regardless
  // of the default ".*" parameter whitelist. Mirrors the ROS 2 bridge, which
  // excludes its own node when enumerating parameters.
  constexpr char kOwnPrefix[] = "/foxglove_bridge/";
  constexpr char kDeviceToken[] = "/foxglove_bridge/device_token";

  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  // Enumerate-all path: an empty request lists every parameter on the master,
  // but must omit the bridge's own.
  auto allFuture = client->waitForParameters("get-all");
  client->getParameters({}, "get-all");
  ASSERT_EQ(std::future_status::ready, allFuture.wait_for(DEFAULT_TIMEOUT));
  for (const auto& param : allFuture.get()) {
    EXPECT_NE(param.name().rfind(kOwnPrefix, 0), 0u)
      << "Bridge leaked its own parameter: " << param.name();
  }

  // Explicitly-named path: asking for the device_token by name must also
  // return nothing.
  auto tokenFuture = client->waitForParameters("get-token");
  client->getParameters({kDeviceToken}, "get-token");
  ASSERT_EQ(std::future_status::ready, tokenFuture.wait_for(DEFAULT_TIMEOUT));
  EXPECT_TRUE(tokenFuture.get().empty());

  // Sanity check: the secret really is set on the master, so the assertions
  // above exercise the exclusion rather than an absent parameter.
  ros::NodeHandle nh;
  std::string rosValue;
  ASSERT_TRUE(nh.getParam(kDeviceToken, rosValue));
  EXPECT_EQ(rosValue, "fox_dt_smoke_test_secret");
}

TEST(SmokeTest, ParameterTypes) {
  // Round-trip every non-string parameter type through both conversion
  // directions: master -> client (valueFromRosParam) on get, and
  // client -> master (toRosParam) on set.
  ros::NodeHandle nh;
  nh.setParam("/types/int", 42);
  nh.setParam("/types/double", 2.5);
  nh.setParam("/types/bool", true);
  nh.setParam("/types/array", std::vector<int>{1, 2, 3});
  XmlRpc::XmlRpcValue dict;
  dict["x"] = 7;
  dict["y"] = std::string("z");
  nh.setParam("/types/dict", dict);

  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  // Get direction.
  auto getFuture = client->waitForParameters("typeget");
  client->getParameters(
    {"/types/int", "/types/double", "/types/bool", "/types/array", "/types/dict"}, "typeget"
  );
  ASSERT_EQ(std::future_status::ready, getFuture.wait_for(DEFAULT_TIMEOUT));
  const auto params = getFuture.get();
  std::map<std::string, const foxglove::Parameter*> byName;
  for (const auto& p : params) {
    byName[std::string(p.name())] = &p;
  }
  ASSERT_EQ(byName.count("/types/int"), 1u);
  EXPECT_EQ(byName["/types/int"]->value()->get<int64_t>(), 42);
  EXPECT_DOUBLE_EQ(byName["/types/double"]->value()->get<double>(), 2.5);
  EXPECT_EQ(byName["/types/bool"]->value()->get<bool>(), true);
  const auto arr =
    byName["/types/array"]->value()->get<std::vector<foxglove::ParameterValueView>>();
  ASSERT_EQ(arr.size(), 3u);
  EXPECT_EQ(arr[0].get<int64_t>(), 1);
  EXPECT_EQ(arr[2].get<int64_t>(), 3);
  bool foundX = false;
  for (const auto& [key, value] :
       byName["/types/dict"]->value()->get<foxglove::ParameterValueView::Dict>()) {
    if (key == "x") {
      EXPECT_EQ(value.get<int64_t>(), 7);
      foundX = true;
    }
  }
  EXPECT_TRUE(foundX);

  // Set direction: set each type from the client and read it back on the
  // master. Parameter is move-only, so build the list element by element.
  std::vector<foxglove::Parameter> toSet;
  toSet.emplace_back("/types/set_int", int64_t(11));
  toSet.emplace_back("/types/set_double", 1.5);
  toSet.emplace_back("/types/set_bool", true);
  std::vector<foxglove::ParameterValue> setArr;
  setArr.emplace_back(int64_t(8));
  setArr.emplace_back(int64_t(9));
  toSet.emplace_back(
    "/types/set_array", foxglove::ParameterType::None, foxglove::ParameterValue(std::move(setArr))
  );
  std::map<std::string, foxglove::ParameterValue> setDict;
  setDict.insert({"k", foxglove::ParameterValue(int64_t(5))});
  toSet.emplace_back(
    "/types/set_dict", foxglove::ParameterType::None, foxglove::ParameterValue(std::move(setDict))
  );

  auto setFuture = client->waitForParameters("typeset");
  client->setParameters(toSet, "typeset");
  ASSERT_EQ(std::future_status::ready, setFuture.wait_for(DEFAULT_TIMEOUT));

  int mInt = 0;
  ASSERT_TRUE(nh.getParam("/types/set_int", mInt));
  EXPECT_EQ(mInt, 11);
  double mDouble = 0.0;
  ASSERT_TRUE(nh.getParam("/types/set_double", mDouble));
  EXPECT_DOUBLE_EQ(mDouble, 1.5);
  bool mBool = false;
  ASSERT_TRUE(nh.getParam("/types/set_bool", mBool));
  EXPECT_TRUE(mBool);
  std::vector<int> mArray;
  ASSERT_TRUE(nh.getParam("/types/set_array", mArray));
  ASSERT_EQ(mArray.size(), 2u);
  EXPECT_EQ(mArray[0], 8);
  XmlRpc::XmlRpcValue mDict;
  ASSERT_TRUE(nh.getParam("/types/set_dict", mDict));
  ASSERT_EQ(mDict.getType(), XmlRpc::XmlRpcValue::TypeStruct);
  EXPECT_EQ(static_cast<int>(mDict["k"]), 5);
}

TEST(SmokeTest, GetNonexistentParameters) {
  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  auto future = client->waitForParameters("get-missing");
  client->getParameters({"/smoke/no_such_param", "/smoke/no_such/nested"}, "get-missing");
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  EXPECT_TRUE(future.get().empty());
}

TEST(SmokeTest, UnsetParameter) {
  // A set with no value deletes the parameter from the master.
  ros::NodeHandle nh;
  nh.setParam("/smoke/deletable", true);

  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  std::vector<foxglove::Parameter> toUnset;
  toUnset.emplace_back("/smoke/deletable");
  auto future = client->waitForParameters("unset-1");
  client->setParameters(toUnset, "unset-1");
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  EXPECT_TRUE(future.get().empty());
  EXPECT_FALSE(nh.hasParam("/smoke/deletable"));
}

TEST(SmokeTest, SetFloatParameterWithIntegerValue) {
  // A whole-valued float may arrive as a JSON integer with a float64 type
  // hint; it must be stored on the master as a double, not an int.
  ros::NodeHandle nh;
  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  auto future = client->waitForParameters("set-float-int");
  const nlohmann::json::array_t parameters = {
    {{"name", "/smoke/float_from_int"}, {"value", 10}, {"type", "float64"}},
  };
  client->sendText(nlohmann::json{
    {"op", "setParameters"}, {"id", "set-float-int"}, {"parameters", parameters}
  }.dump());
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));

  XmlRpc::XmlRpcValue value;
  ASSERT_TRUE(nh.getParam("/smoke/float_from_int", value));
  ASSERT_EQ(value.getType(), XmlRpc::XmlRpcValue::TypeDouble);
  EXPECT_DOUBLE_EQ(static_cast<double>(value), 10.0);
}

TEST(SmokeTest, ParameterUnsubscribeStopsUpdates) {
  constexpr char kParam[] = "/smoke/unsub_param";
  ros::NodeHandle nh;
  nh.setParam(kParam, "initial");

  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  // Subscribe and confirm pushes arrive (see the Parameters test for why the
  // set is repeated).
  client->subscribeParameterUpdates({kParam});
  auto updateFuture = client->waitForParameters();
  const auto deadline = std::chrono::steady_clock::now() + DEFAULT_TIMEOUT;
  std::future_status status = std::future_status::timeout;
  while (status != std::future_status::ready && std::chrono::steady_clock::now() < deadline) {
    nh.setParam(kParam, "subscribed");
    status = updateFuture.wait_for(500ms);
  }
  ASSERT_EQ(std::future_status::ready, status);

  // Unsubscribe, then round-trip a get: the bridge serializes parameter ops
  // on one worker, so once the get is answered the master unsubscribe has
  // been issued.
  client->unsubscribeParameterUpdates({kParam});
  auto syncFuture = client->waitForParameters("unsub-sync");
  client->getParameters({kParam}, "unsub-sync");
  ASSERT_EQ(std::future_status::ready, syncFuture.wait_for(DEFAULT_TIMEOUT));

  auto staleFuture = client->waitForParameters();
  nh.setParam(kParam, "unsubscribed");
  EXPECT_EQ(std::future_status::timeout, staleFuture.wait_for(2s))
    << "received a parameter update after unsubscribing";
}

TEST(SmokeTest, BridgeOwnParametersNotWritable) {
  constexpr char kDeviceToken[] = "/foxglove_bridge/device_token";
  ros::NodeHandle nh;
  auto client = std::make_shared<Client>();
  ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(DEFAULT_TIMEOUT));

  std::vector<foxglove::Parameter> toSet;
  toSet.emplace_back(kDeviceToken, "overwritten_by_client");
  toSet.emplace_back("/foxglove_bridge/port");
  auto future = client->waitForParameters("set-own");
  client->setParameters(toSet, "set-own");
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  EXPECT_TRUE(future.get().empty());

  std::string token;
  ASSERT_TRUE(nh.getParam(kDeviceToken, token));
  EXPECT_EQ(token, "fox_dt_smoke_test_secret");
  EXPECT_TRUE(nh.hasParam("/foxglove_bridge/port"));
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
