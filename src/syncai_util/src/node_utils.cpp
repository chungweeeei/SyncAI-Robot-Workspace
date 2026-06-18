#include "syncai_util/node_utils.hpp"

#include <string>

namespace syncai_util
{

std::string add_namespaces(const std::string & top_ns, const std::string & sub_ns)
{
  if (sub_ns.empty()) {
    return top_ns;
  }

  // If top_ns ends with '/', avoid producing a double slash when joining.
  if (!top_ns.empty() && top_ns.back() == '/') {
    if (sub_ns.front() == '/') {
      return top_ns.substr(0, top_ns.size() - 1) + sub_ns;
    }
    return top_ns + sub_ns;
  }

  if (sub_ns.front() == '/') {
    return top_ns + sub_ns;
  }
  return top_ns + "/" + sub_ns;
}

}  // namespace syncai_util
