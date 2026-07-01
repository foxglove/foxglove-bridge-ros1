#include <foxglove_bridge/param_interface.hpp>
#include <foxglove_bridge/utils.hpp>

#include <ros/master.h>
#include <ros/names.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <map>

namespace foxglove_bridge {

namespace {

// Cap nesting depth in the recursive parameter conversions below, so a
// deeply nested value can't overflow the stack. Real parameters are shallow,
// so a generous cap costs nothing; the throw is caught by the callers, which
// handle each parameter individually.
//
// The client direction (toRosParam) is already bounded upstream: the SDK
// parses client messages with serde_json, whose default recursion limit
// (128) rejects over-deep values before they reach us. This cap matters most
// for the master direction (valueFromRosParam), whose XmlRpc::XmlRpcValue
// comes from roscpp's XML-RPC parser and has no such guarantee. Set to 128 to
// match serde_json's limit, so this never rejects a value the SDK accepted.
constexpr int MAX_PARAM_DEPTH = 128;

foxglove::ParameterValue valueFromRosParam(XmlRpc::XmlRpcValue& value, int depth = 0) {
  using XmlRpc::XmlRpcValue;
  if (depth > MAX_PARAM_DEPTH) {
    throw std::runtime_error(
      "Parameter value nested deeper than " + std::to_string(MAX_PARAM_DEPTH)
    );
  }
  switch (value.getType()) {
    case XmlRpcValue::TypeBoolean:
      return foxglove::ParameterValue(static_cast<bool>(value));
    case XmlRpcValue::TypeInt:
      return foxglove::ParameterValue(static_cast<int64_t>(static_cast<int>(value)));
    case XmlRpcValue::TypeDouble:
      return foxglove::ParameterValue(static_cast<double>(value));
    case XmlRpcValue::TypeString:
      return foxglove::ParameterValue(static_cast<std::string&>(value));
    case XmlRpcValue::TypeArray: {
      std::vector<foxglove::ParameterValue> values;
      values.reserve(static_cast<size_t>(value.size()));
      for (int i = 0; i < value.size(); ++i) {
        values.push_back(valueFromRosParam(value[i], depth + 1));
      }
      return foxglove::ParameterValue(std::move(values));
    }
    case XmlRpcValue::TypeStruct: {
      std::map<std::string, foxglove::ParameterValue> values;
      for (auto& [memberName, memberValue] : value) {
        values.insert({memberName, valueFromRosParam(memberValue, depth + 1)});
      }
      return foxglove::ParameterValue(std::move(values));
    }
    default:
      throw std::runtime_error("Unsupported parameter type " + std::to_string(value.getType()));
  }
}

foxglove::Parameter fromRosParam(const std::string& name, XmlRpc::XmlRpcValue& value) {
  using XmlRpc::XmlRpcValue;
  switch (value.getType()) {
    case XmlRpcValue::TypeBoolean:
      return foxglove::Parameter(name, static_cast<bool>(value));
    case XmlRpcValue::TypeInt:
      return foxglove::Parameter(name, static_cast<int64_t>(static_cast<int>(value)));
    case XmlRpcValue::TypeDouble:
      return foxglove::Parameter(name, static_cast<double>(value));
    case XmlRpcValue::TypeString:
      return foxglove::Parameter(name, static_cast<std::string&>(value));
    default:
      return foxglove::Parameter(name, foxglove::ParameterType::None, valueFromRosParam(value, 1));
  }
}

XmlRpc::XmlRpcValue toRosParam(
  const foxglove::ParameterValueView& value, foxglove::ParameterType type, int depth = 0
) {
  using XmlRpc::XmlRpcValue;
  if (depth > MAX_PARAM_DEPTH) {
    throw std::runtime_error(
      "Parameter value nested deeper than " + std::to_string(MAX_PARAM_DEPTH)
    );
  }
  if (value.is<bool>()) {
    return XmlRpcValue(value.get<bool>());
  } else if (value.is<int64_t>()) {
    const auto intValue = value.get<int64_t>();
    if (type == foxglove::ParameterType::Float64) {
      // A whole-valued float round-trips as an integer with a type hint.
      return XmlRpcValue(static_cast<double>(intValue));
    }
    return XmlRpcValue(static_cast<int>(intValue));
  } else if (value.is<double>()) {
    return XmlRpcValue(value.get<double>());
  } else if (value.is<std::string>()) {
    return XmlRpcValue(value.get<std::string>());
  } else if (value.is<foxglove::ParameterValueView::Array>()) {
    XmlRpcValue arr;
    const auto values = value.get<foxglove::ParameterValueView::Array>();
    for (size_t i = 0; i < values.size(); ++i) {
      arr[static_cast<int>(i)] = toRosParam(values[i], foxglove::ParameterType::None, depth + 1);
    }
    return arr;
  } else if (value.is<foxglove::ParameterValueView::Dict>()) {
    XmlRpcValue obj;
    for (const auto& [memberName, memberValue] : value.get<foxglove::ParameterValueView::Dict>()) {
      obj[memberName] = toRosParam(memberValue, foxglove::ParameterType::None, depth + 1);
    }
    return obj;
  }
  throw std::runtime_error("Unsupported parameter value");
}

}  // namespace

Ros1ParameterInterface::Ros1ParameterInterface(
  ros::NodeHandle nh, std::vector<std::regex> paramWhitelistPatterns
)
    : _nh(std::move(nh))
    , _paramWhitelistPatterns(std::move(paramWhitelistPatterns))
    , _ownNamespacePrefix(ros::this_node::getName() + "/") {
  _xmlrpcServer.bind(
    "paramUpdate",
    [this](XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
      parameterUpdates(params, result);
    }
  );
  _xmlrpcServer.start();
}

Ros1ParameterInterface::~Ros1ParameterInterface() {
  shutdown();
}

void Ros1ParameterInterface::shutdown() {
  // Politely drop our registrations; the master would otherwise only clean
  // them up after a failed paramUpdate notification.
  std::unordered_set<std::string> subscribed;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_shutdown) {
      return;
    }
    _shutdown = true;
    subscribed.swap(_subscribedParams);
  }
  for (const auto& paramName : subscribed) {
    executeParamSubscription("unsubscribeParam", paramName);
  }
  // Joins the XML-RPC server thread, so no parameterUpdates callback survives
  // this call.
  _xmlrpcServer.shutdown();
}

ParameterList Ros1ParameterInterface::getParams(
  const std::vector<std::string_view>& paramNames, const std::chrono::duration<double>& timeout
) {
  (void)timeout;

  const bool allParametersRequested = paramNames.empty();
  std::vector<std::string> names;
  if (allParametersRequested) {
    if (!_nh.getParamNames(names)) {
      throw std::runtime_error("Failed to retrieve parameter names from the master");
    }
  } else {
    names.reserve(paramNames.size());
    for (const auto& name : paramNames) {
      names.emplace_back(name);
    }
  }

  ParameterList params;
  for (const auto& name : names) {
    if (isOwnParameter(name)) {
      // Never expose the bridge's own parameters (e.g. device_token) to
      // clients, even if explicitly requested and regardless of the whitelist.
      continue;
    }
    if (!isWhitelisted(name, _paramWhitelistPatterns)) {
      if (!allParametersRequested) {
        ROS_ERROR("Parameter '%s' is not on the parameter whitelist", name.c_str());
      }
      continue;
    }

    try {
      XmlRpc::XmlRpcValue value;
      if (_nh.getParam(name, value)) {
        params.push_back(fromRosParam(name, value));
      } else if (!allParametersRequested) {
        ROS_WARN("Parameter '%s' is not set", name.c_str());
      }
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to read parameter '%s': %s", name.c_str(), ex.what());
    }
  }
  return params;
}

void Ros1ParameterInterface::setParams(
  const ParameterList& params, const std::chrono::duration<double>& timeout
) {
  (void)timeout;

  for (const auto& param : params) {
    const std::string name(param.name());
    if (isOwnParameter(name)) {
      // Never let clients modify the bridge's own parameters (e.g.
      // device_token), even if explicitly requested and regardless of the
      // whitelist.
      continue;
    }
    if (!isWhitelisted(name, _paramWhitelistPatterns)) {
      ROS_ERROR("Parameter '%s' is not on the parameter whitelist", name.c_str());
      continue;
    }

    try {
      const auto value = param.value();
      if (!value.has_value()) {
        _nh.deleteParam(name);
      } else {
        _nh.setParam(name, toRosParam(*value, param.type()));
      }
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to set parameter '%s': %s", name.c_str(), ex.what());
    }
  }
}

bool Ros1ParameterInterface::executeParamSubscription(
  const std::string& opName, const std::string& paramName
) {
  // Registered under a distinct caller id so the master doesn't conflate these
  // subscriptions with roscpp's own (cached-parameter) registrations.
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = ros::this_node::getName() + "2";
  params[1] = _xmlrpcServer.getServerURI();
  params[2] = ros::names::resolve(paramName);

  if (ros::master::execute(opName, params, result, payload, false)) {
    ROS_DEBUG("%s '%s'", opName.c_str(), paramName.c_str());
    return true;
  }
  ROS_WARN("Failed to %s '%s': %s", opName.c_str(), paramName.c_str(), result.toXml().c_str());
  return false;
}

bool Ros1ParameterInterface::isOwnParameter(const std::string& name) const {
  return name.compare(0, _ownNamespacePrefix.size(), _ownNamespacePrefix) == 0;
}

void Ros1ParameterInterface::subscribeParams(const std::vector<std::string_view>& paramNames) {
  {
    // After shutdown() the XML-RPC server is stopped; registering now would
    // point the master at a dead URI, and shutdown()'s idempotence guard means
    // it would never be cleaned up. Checked before executeParamSubscription so
    // no registration is issued, not just skipped in the bookkeeping. The
    // param worker (the only caller) is still alive between shutdown() and the
    // later _transports->stop() that joins it, so this path is reachable.
    std::lock_guard<std::mutex> lock(_mutex);
    if (_shutdown) {
      return;
    }
  }
  for (const auto& nameView : paramNames) {
    const std::string name(nameView);
    if (isOwnParameter(name)) {
      // A subscription pushes the parameter's value to the client on change;
      // don't let clients subscribe to the bridge's own parameters.
      continue;
    }
    if (!isWhitelisted(name, _paramWhitelistPatterns)) {
      ROS_ERROR("Parameter '%s' is not on the parameter whitelist", name.c_str());
      continue;
    }
    if (executeParamSubscription("subscribeParam", name)) {
      std::lock_guard<std::mutex> lock(_mutex);
      _subscribedParams.insert(name);
    }
  }
}

void Ros1ParameterInterface::unsubscribeParams(const std::vector<std::string_view>& paramNames) {
  for (const auto& nameView : paramNames) {
    const std::string name(nameView);
    if (executeParamSubscription("unsubscribeParam", name)) {
      std::lock_guard<std::mutex> lock(_mutex);
      _subscribedParams.erase(name);
    }
  }
}

void Ros1ParameterInterface::setParamUpdateCallback(ParamUpdateFunc paramUpdateFunc) {
  std::lock_guard<std::mutex> lock(_mutex);
  _paramUpdateFunc = std::move(paramUpdateFunc);
}

void Ros1ParameterInterface::parameterUpdates(
  XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result
) {
  result[0] = 1;
  result[1] = std::string("");
  result[2] = 0;

  if (params.size() != 3) {
    ROS_ERROR("Parameter update called with invalid parameter size: %d", params.size());
    return;
  }

  try {
    const std::string paramName = ros::names::clean(params[1]);
    XmlRpc::XmlRpcValue paramValue = params[2];
    auto param = fromRosParam(paramName, paramValue);

    ParamUpdateFunc updateFunc;
    {
      std::lock_guard<std::mutex> lock(_mutex);
      updateFunc = _paramUpdateFunc;
    }
    if (updateFunc) {
      ParameterList update;
      update.push_back(std::move(param));
      updateFunc(update);
    }
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to update parameter: %s", ex.what());
  } catch (const XmlRpc::XmlRpcException& ex) {
    ROS_ERROR("Failed to update parameter: %s", ex.getMessage().c_str());
  } catch (...) {
    ROS_ERROR("Failed to update parameter");
  }
}

}  // namespace foxglove_bridge
