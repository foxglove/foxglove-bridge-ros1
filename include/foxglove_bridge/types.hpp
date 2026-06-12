#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace foxglove_bridge {

using ClientId = uint32_t;
using SinkId = uint64_t;
using ChannelId = uint64_t;
using ChannelAndClientId = std::pair<ChannelId, ClientId>;

using MapOfSets = std::unordered_map<std::string, std::unordered_set<std::string>>;

/// Mix a hash value into a seed (the boost::hash_combine construction). A
/// plain XOR collides on symmetric pairs and clustered keys.
inline std::size_t hashCombine(std::size_t seed, std::size_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

struct PairHash {
  template<class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const {
    return hashCombine(std::hash<T1>()(pair.first), std::hash<T2>()(pair.second));
  }
};

class ClientChannelError : public std::runtime_error {
public:
  ClientChannelError(const std::string& msg)
      : std::runtime_error(msg) {}
};

/// A client-advertised channel, normalized across the two transports (the
/// WebSocket server reports foxglove::ClientChannel, the remote access gateway
/// reports foxglove::ChannelDescriptor).
///
/// The schema pointer is only valid for the duration of the delegate callback.
struct ClientChannelInfo {
  ChannelId id;
  std::string topic;
  std::string encoding;
  std::string schemaName;
  const std::byte* schemaData = nullptr;
  size_t schemaLen = 0;
};

}  // namespace foxglove_bridge
