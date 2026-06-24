#pragma once

#include <foxglove_bridge/transport_manager.hpp>

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

#include <functional>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

namespace foxglove_bridge {

using ParamUpdateFunc = std::function<void(const ParameterList&)>;

/// ParameterBackend over the ROS 1 master parameter server.
///
/// Subscriptions use the master's `subscribeParam` push mechanism: a second
/// ros::XMLRPCManager instance (roscpp's own already binds `paramUpdate` for
/// its internal cache) serves a `paramUpdate` endpoint that the master calls
/// when a subscribed parameter changes. Ported from the legacy
/// foxglove/ros-foxglove-bridge (MIT).
class Ros1ParameterInterface : public ParameterBackend {
public:
  Ros1ParameterInterface(ros::NodeHandle nh, std::vector<std::regex> paramWhitelistPatterns);
  ~Ros1ParameterInterface() override;

  ParameterList getParams(
    const std::vector<std::string_view>& paramNames, const std::chrono::duration<double>& timeout
  ) override;
  void setParams(const ParameterList& params, const std::chrono::duration<double>& timeout)
    override;
  void subscribeParams(const std::vector<std::string_view>& paramNames) override;
  void unsubscribeParams(const std::vector<std::string_view>& paramNames) override;

  void setParamUpdateCallback(ParamUpdateFunc paramUpdateFunc);

  /// Unsubscribe all parameter subscriptions and stop the XML-RPC server,
  /// joining its thread: after this returns, no parameter-update callback is
  /// in flight or can start. Called by the destructor; call explicitly before
  /// destroying whatever the update callback publishes into. Idempotent.
  void shutdown();

private:
  /// `paramUpdate` XML-RPC endpoint, called by the master on parameter change.
  void parameterUpdates(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  /// Issue a master subscribeParam/unsubscribeParam call for one parameter.
  bool executeParamSubscription(const std::string& opName, const std::string& paramName);

  /// True if `name` is one of the bridge node's own parameters, i.e. lives
  /// under its private namespace. Mirrors the ROS 2 bridge, which excludes its
  /// own node when enumerating parameters, so the bridge never exposes its own
  /// configuration -- most importantly the remote-access `device_token` -- back
  /// to connected clients, regardless of the parameter whitelist.
  bool isOwnParameter(const std::string& name) const;

  ros::NodeHandle _nh;
  std::vector<std::regex> _paramWhitelistPatterns;

  /// Resolved private namespace of the bridge node (`<node_name>/`), used by
  /// isOwnParameter to identify the bridge's own parameters.
  const std::string _ownNamespacePrefix;

  ros::XMLRPCManager _xmlrpcServer;

  std::mutex _mutex;
  ParamUpdateFunc _paramUpdateFunc;
  std::unordered_set<std::string> _subscribedParams;
  bool _shutdown = false;
};

}  // namespace foxglove_bridge
