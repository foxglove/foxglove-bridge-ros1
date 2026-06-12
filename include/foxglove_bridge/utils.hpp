#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <regex>
#include <string>
#include <vector>

namespace foxglove_bridge {

/// Clamp an int64 value to [max(min, 0), size_t::max] and convert to size_t.
inline size_t saturatingToSizeT(int64_t value, int64_t min = 0) {
  min = std::max(min, int64_t(0));
  if (value <= min) {
    return static_cast<size_t>(min);
  }
  // Check the upper bound as uint64_t to avoid wrapping on platforms where int64_t is larger than
  // size_t
  const auto u = static_cast<uint64_t>(value);
  constexpr auto kMax = static_cast<uint64_t>(std::numeric_limits<size_t>::max());
  return static_cast<size_t>(std::min(u, kMax));
}

inline bool isWhitelisted(const std::string& name, const std::vector<std::regex>& regexPatterns) {
  return std::find_if(regexPatterns.begin(), regexPatterns.end(), [name](const auto& regex) {
           return std::regex_match(name, regex);
         }) != regexPatterns.end();
}

}  // namespace foxglove_bridge
