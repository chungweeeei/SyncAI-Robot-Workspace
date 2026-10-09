#include "syncai_mapping/octomap_recipe.hpp"

#include <unistd.h>

#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace syncai_mapping
{

// Moved here from PGONode (2026-10) when the sidecar became its second user.
std::string jsonEscape(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  for (const unsigned char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  return out;
}

namespace octomap_recipe
{

namespace
{

// Classic locale and 12 significant digits: "0.1" stays "0.1", and a
// process-wide locale with a decimal comma cannot produce invalid JSON.
std::ostringstream jsonStream()
{
  std::ostringstream os;
  os.imbue(std::locale::classic());
  os << std::setprecision(12);
  return os;
}

void writeHead(std::ostringstream & os, const char * status, const std::string & started_at)
{
  os << "{\"status\":\"" << status << "\",\"started_at\":\"" << jsonEscape(started_at) << "\"";
}

void writeParams(std::ostringstream & os, const octomap_builder::Params & p)
{
  os << ",\"params\":{\"resolution\":" << p.resolution << ",\"max_range\":" << p.max_range
     << ",\"min_range\":" << p.min_range << ",\"lidar_height\":" << p.lidar_height
     << ",\"floor_band\":" << p.floor_band << ",\"max_height\":" << p.max_height
     << ",\"floor_radius\":" << p.floor_radius << ",\"min_hits\":" << p.min_hits
     << ",\"dynamic_miss_ratio\":" << p.dynamic_miss_ratio
     << ",\"dynamic_min_miss\":" << p.dynamic_min_miss
     << ",\"clip_below_floor\":" << (p.clip_below_floor ? "true" : "false") << "}";
}

}  // namespace

std::string isoUtcNow()
{
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string converting(const octomap_builder::Params & params, const std::string & started_at)
{
  auto os = jsonStream();
  writeHead(os, "converting", started_at);
  writeParams(os, params);
  os << "}";
  return os.str();
}

std::string ok(
  const octomap_builder::Params & params, const std::string & started_at,
  const std::string & finished_at, const octomap_builder::Measurements & m)
{
  auto os = jsonStream();
  writeHead(os, "ok", started_at);
  os << ",\"finished_at\":\"" << jsonEscape(finished_at) << "\"";
  writeParams(os, params);
  os << ",\"measurements\":{\"keyframes\":" << m.keyframes
     << ",\"skipped_patches\":" << m.skipped_patches << ",\"points\":" << m.points
     << ",\"leaves\":" << m.leaves << ",\"occupied_leaves\":" << m.occupied_leaves
     << ",\"free_leaves\":" << m.free_leaves << ",\"road_voxels\":" << m.road_voxels
     << ",\"road_area_m2\":" << m.road_area_m2 << ",\"occupied_voxels\":" << m.occupied_voxels
     << ",\"dynamic_voxels\":" << m.dynamic_voxels << ",\"sparse_voxels\":" << m.sparse_voxels
     << ",\"map_points_removed\":" << m.map_points_removed
     << ",\"map_points_kept\":" << m.map_points_kept
     << ",\"map_pcd_rewritten\":" << (m.map_pcd_rewritten ? "true" : "false")
     << ",\"clipped_rays\":" << m.clipped_rays << ",\"elapsed_s\":" << m.elapsed_s << "}}";
  return os.str();
}

std::string failed(
  const octomap_builder::Params & params, const std::string & started_at,
  const std::string & finished_at, const std::string & error)
{
  std::string line = error;
  for (char & c : line) {
    if (c == '\n' || c == '\r') c = ' ';
  }
  auto os = jsonStream();
  writeHead(os, "failed", started_at);
  os << ",\"finished_at\":\"" << jsonEscape(finished_at) << "\"";
  writeParams(os, params);
  os << ",\"error\":\"" << jsonEscape(line) << "\"}";
  return os.str();
}

void writeAtomic(const std::filesystem::path & dir, const std::string & json)
{
  const std::filesystem::path final_path = dir / kSidecarFile;
  // Per-process temp name: pgo_node and the build it spawned both write this
  // file, and two writers sharing one .tmp would rename each other's file
  // away (the loser then fails with ENOENT).
  const std::filesystem::path tmp_path =
    dir / (std::string(kSidecarFile) + "." + std::to_string(getpid()) + ".tmp");
  {
    std::ofstream f(tmp_path, std::ios::trunc);
    f << json << "\n";
    f.close();
    if (!f) {
      std::error_code ec;
      std::filesystem::remove(tmp_path, ec);
      throw std::runtime_error(std::string("cannot write ") + kSidecarFile);
    }
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, final_path, ec);
  if (ec) {
    std::error_code rm_ec;
    std::filesystem::remove(tmp_path, rm_ec);
    throw std::runtime_error(std::string("cannot rename ") + kSidecarFile + ": " + ec.message());
  }
}

}  // namespace octomap_recipe
}  // namespace syncai_mapping
