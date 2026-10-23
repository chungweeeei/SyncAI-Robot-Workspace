#include "syncai_localizer/localizers/last_pose_store.h"

#include <fcntl.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>

namespace
{
std::string utcNow()
{
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

bool writeAll(int fd, const std::string & data)
{
  std::size_t done = 0;
  while (done < data.size()) {
    const ssize_t n = ::write(fd, data.data() + done, data.size() - done);
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    done += static_cast<std::size_t>(n);
  }
  return true;
}
}  // namespace

bool saveLastPose(const std::string & path, LastPose pose, std::string & error)
{
  namespace fs = std::filesystem;
  const fs::path target(path);
  const fs::path dir = target.parent_path();
  std::error_code ec;
  if (!dir.empty()) {
    fs::create_directories(dir, ec);
    if (ec) {
      error = "cannot create " + dir.string() + ": " + ec.message();
      return false;
    }
  }

  pose.saved_at = utcNow();
  // Fixed key order and indentation, so the file reads the same every time
  // someone cats it on the robot.
  nlohmann::ordered_json j;
  j["map"] = pose.map;
  j["x"] = pose.x;
  j["y"] = pose.y;
  j["ground_z"] = pose.ground_z;
  j["yaw"] = pose.yaw;
  j["saved_at"] = pose.saved_at;
  const std::string text = j.dump(2) + "\n";

  const std::string tmp = path + ".tmp";
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    error = "cannot open " + tmp + ": " + std::strerror(errno);
    return false;
  }
  const bool ok = writeAll(fd, text) && ::fsync(fd) == 0;
  const int saved_errno = errno;
  ::close(fd);
  if (!ok) {
    error = "cannot write " + tmp + ": " + std::strerror(saved_errno);
    ::unlink(tmp.c_str());
    return false;
  }
  if (::rename(tmp.c_str(), path.c_str()) != 0) {
    error = "cannot rename " + tmp + " -> " + path + ": " + std::strerror(errno);
    ::unlink(tmp.c_str());
    return false;
  }
  // The rename is only durable once the directory entry is: without this a
  // power loss right after can bring back the old file, or none.
  const int dfd = ::open(dir.empty() ? "." : dir.c_str(), O_RDONLY | O_DIRECTORY);
  if (dfd >= 0) {
    ::fsync(dfd);
    ::close(dfd);
  }
  return true;
}

std::optional<LastPose> loadLastPose(const std::string & path, std::string & error)
{
  std::ifstream in(path);
  if (!in) {
    error = "cannot open " + path;
    return std::nullopt;
  }
  // No exceptions out of here: a truncated file is a parse error, and a
  // missing or mistyped field is a type error; both mean "no pose".
  const nlohmann::json j = nlohmann::json::parse(in, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) {
    error = "not a JSON object";
    return std::nullopt;
  }
  LastPose pose;
  if (!j.contains("map") || !j["map"].is_string() || j["map"].get<std::string>().empty()) {
    error = "missing map";
    return std::nullopt;
  }
  pose.map = j["map"].get<std::string>();
  for (const auto & [key, out] :
       {std::pair<const char *, double *>{"x", &pose.x},
        {"y", &pose.y},
        {"ground_z", &pose.ground_z},
        {"yaw", &pose.yaw}}) {
    if (!j.contains(key) || !j[key].is_number()) {
      error = std::string("missing or non-numeric ") + key;
      return std::nullopt;
    }
    *out = j[key].get<double>();
  }
  if (j.contains("saved_at") && j["saved_at"].is_string()) {
    pose.saved_at = j["saved_at"].get<std::string>();
  }
  return pose;
}

std::string mapNameFromPcdPath(const std::string & pcd_path)
{
  return std::filesystem::path(pcd_path).parent_path().filename().string();
}
