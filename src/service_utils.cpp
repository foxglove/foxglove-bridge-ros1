#include <foxglove_bridge/service_utils.hpp>

#include <ros/connection.h>
#include <ros/connection_manager.h>
#include <ros/poll_manager.h>
#include <ros/service_manager.h>
#include <ros/this_node.h>
#include <ros/transport/transport_tcp.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>

namespace foxglove_bridge {

/**
 * Looks up the service server host & port and opens a TCP connection to it to retrieve the header
 * which contains the service type.
 *
 * The implementation is similar to how ROS does it under the hood when creating a service server
 * link:
 * https://github.com/ros/ros_comm/blob/845f74602c7464e08ef5ac6fd9e26c97d0fe42c9/clients/roscpp/src/libros/service_manager.cpp#L246-L261
 * https://github.com/ros/ros_comm/blob/845f74602c7464e08ef5ac6fd9e26c97d0fe42c9/clients/roscpp/src/libros/service_server_link.cpp#L114-L130
 */
std::string retrieveServiceType(const std::string& serviceName, std::chrono::milliseconds timeout) {
  std::string srvHost;
  uint32_t srvPort;
  if (!ros::ServiceManager::instance()->lookupService(serviceName, srvHost, srvPort)) {
    throw std::runtime_error("Failed to lookup service " + serviceName);
  }

  auto transport =
    boost::make_shared<ros::TransportTCP>(&ros::PollManager::instance()->getPollSet());
  auto connection = boost::make_shared<ros::Connection>();
  ros::ConnectionManager::instance()->addConnection(connection);
  connection->initialize(transport, false, ros::HeaderReceivedFunc());

  if (!transport->connect(srvHost, srvPort)) {
    throw std::runtime_error("Failed to connect to service server of service " + serviceName);
  }

  // The callback runs on roscpp's poll thread and can race the timeout path
  // below: Connection::drop() does not synchronize with a callback that is
  // already executing. The callback therefore shares ownership of the promise
  // (so a late invocation writes into live memory) and a fulfilled flag
  // guards against ever setting it twice.
  struct SharedState {
    std::promise<std::string> promise;
    std::atomic<bool> fulfilled{false};
  };
  auto state = std::make_shared<SharedState>();
  auto future = state->promise.get_future();

  connection->setHeaderReceivedCallback(
    [state](const ros::ConnectionPtr& conn, const ros::Header& header) {
      if (!state->fulfilled.exchange(true)) {
        std::string serviceType;
        if (header.getValue("type", serviceType)) {
          state->promise.set_value(serviceType);
        } else {
          state->promise.set_exception(std::make_exception_ptr(
            std::runtime_error("Key 'type' not found in service connection header")
          ));
        }
      }
      // Close connection since we don't need it any more.
      conn->drop(ros::Connection::DropReason::Destructing);
      return true;
    }
  );

  ros::M_string header;
  header["service"] = serviceName;
  header["md5sum"] = "*";
  header["callerid"] = ros::this_node::getName();
  header["persistent"] = "0";
  header["probe"] = "1";
  connection->writeHeader(header, [](const ros::ConnectionPtr&) {});

  if (future.wait_for(timeout) != std::future_status::ready) {
    // Stop the probe. A callback already in flight may still complete
    // concurrently; that is safe, since it owns the shared state and the
    // abandoned promise is simply never read.
    connection->drop(ros::Connection::DropReason::Destructing);
    throw std::runtime_error("Timed out when retrieving service type");
  }

  return future.get();
}

}  // namespace foxglove_bridge
