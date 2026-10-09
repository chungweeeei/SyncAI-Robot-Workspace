// clean_map: the post-save map cleaning for one pgo save directory.
//
//   clean_map <map_dir> [--resolution 0.1] [--max-range 20] [--min-range 0.5]
//       [--lidar-height 0.481] [--floor-band 0.25] [--max-height 2.0]
//       [--floor-radius 10] [--min-hits 2] [--dynamic-miss-ratio 2.0]
//       [--dynamic-min-miss 3] [--clip-below-floor 1] [--nice 0]
//       [--lock <path>|none] [--started-at <ISO-8601>]
//
// pgo_node spawns it after a successful save_maps with save_patches
// (detached, nice 10); run by hand it re-cleans any saved map from its
// patches: `ros2 run syncai_mapping clean_map map/<name>`. It REPLACES
// map.pcd and writes map_clean.recipe.json itself, from "converting" to "ok" /
// "failed", so its outcome survives the node that started it.
//
// Exit codes: 0 ok, 1 usage, 2 no usable input (poses.txt / patches/),
// 3 clean or write failed, 4 interrupted (SIGTERM / SIGINT).
//
// A plain executable, not a ROS node: a batch job needs no DDS participant,
// and it then also runs on a workstation that has only PCL and OctoMap.

#include <fcntl.h>
#include <sys/file.h>
#include <sys/resource.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "syncai_mapping/map_clean_recipe.hpp"
#include "syncai_mapping/map_cleaner.hpp"

namespace
{

namespace fs = std::filesystem;
namespace mc = syncai_mapping::map_cleaner;
namespace recipe = syncai_mapping::map_clean_recipe;

volatile std::sig_atomic_t g_stop_signal = 0;

void onStopSignal(int sig)
{
  g_stop_signal = sig;
}

void usage()
{
  std::cerr << "usage: clean_map <map_dir> [--resolution 0.1] [--max-range 20] "
               "[--min-range 0.5]\n"
               "         [--lidar-height 0.481] [--floor-band 0.25] [--max-height 2.0] "
               "[--floor-radius 10]\n"
               "         [--min-hits 2] [--dynamic-miss-ratio 2.0] [--dynamic-min-miss 3]\n"
               "         [--clip-below-floor 1]\n"
               "         [--nice 0] [--lock /dev/shm/syncai_pgo/clean_map.lock|none] "
               "[--started-at <ISO-8601 UTC>]\n";
}

bool parseDouble(const std::string & s, double & out)
{
  try {
    size_t used = 0;
    out = std::stod(s, &used);
    return used == s.size();
  } catch (const std::exception &) {
    return false;
  }
}

// Best effort. Failing either only costs the robot some politeness.
void lowerPriority(int nice_value)
{
  if (nice_value != 0 && setpriority(PRIO_PROCESS, 0, nice_value) != 0) {
    std::cerr << "[clean_map] setpriority(" << nice_value << "): " << std::strerror(errno) << "\n";
  }
  // Make this process the OOM killer's first choice. The hit-voxel map of a
  // large site is hundreds of MB, and without this the kernel picks the
  // largest RSS -- which, mid-run, can be pgo_node holding an unsaved map.
  // Raising one's own score needs no privilege.
  std::ofstream("/proc/self/oom_score_adj") << 500;
}

// Serialise cleans on this host: two saves in a row spawn two cleans, and two
// at once would double the peak memory. /dev/shm because both robot
// containers and the backend share it (`ipc: host`), so this is one lock per
// host -- also the right memory domain. Returns the held fd, or -1 when
// running unlocked (an uncreatable lock is a warning, not a failure).
int takeLock(const std::string & lock_path)
{
  if (lock_path == "none") return -1;
  std::error_code ec;
  fs::create_directories(fs::path(lock_path).parent_path(), ec);
  const int fd = open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
  if (fd < 0) {
    std::cerr << "[clean_map] cannot open lock " << lock_path << " (" << std::strerror(errno)
              << "); running unlocked\n";
    return -1;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) == 0) return fd;
  std::cout << "[clean_map] another clean_map holds " << lock_path << "; waiting" << std::endl;
  // No SA_RESTART on the stop handlers, so a SIGTERM while queued here comes
  // back as EINTR instead of being swallowed.
  while (flock(fd, LOCK_EX) != 0) {
    if (errno != EINTR || g_stop_signal != 0) {
      close(fd);
      return g_stop_signal != 0 ? -2 : -1;
    }
  }
  return fd;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string dir_arg, lock_path = "/dev/shm/syncai_pgo/clean_map.lock", started_at;
  int nice_value = 0;
  mc::Params params;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      usage();
      return 0;
    }
    if (a.rfind("--", 0) != 0) {
      if (!dir_arg.empty()) {
        usage();
        return 1;
      }
      dir_arg = a;
      continue;
    }
    if (i + 1 >= argc) {
      usage();
      return 1;
    }
    const std::string v = argv[++i];
    double d = 0.0;
    bool good = true;
    if (a == "--resolution") {
      good = parseDouble(v, params.resolution) && params.resolution > 0.0;
    } else if (a == "--max-range") {
      good = parseDouble(v, params.max_range);
    } else if (a == "--min-range") {
      good = parseDouble(v, params.min_range);
    } else if (a == "--lidar-height") {
      good = parseDouble(v, params.lidar_height);
    } else if (a == "--floor-band") {
      good = parseDouble(v, params.floor_band);
    } else if (a == "--max-height") {
      good = parseDouble(v, params.max_height);
    } else if (a == "--floor-radius") {
      good = parseDouble(v, params.floor_radius);
    } else if (a == "--min-hits") {
      good = parseDouble(v, d) && d == static_cast<int>(d);
      params.min_hits = static_cast<int>(d);
    } else if (a == "--dynamic-miss-ratio") {
      good = parseDouble(v, params.dynamic_miss_ratio) && params.dynamic_miss_ratio > 0.0;
    } else if (a == "--dynamic-min-miss") {
      good = parseDouble(v, d) && d == static_cast<int>(d);
      params.dynamic_min_miss = static_cast<int>(d);
    } else if (a == "--clip-below-floor") {
      good = (v == "0" || v == "1");
      params.clip_below_floor = (v == "1");
    } else if (a == "--nice") {
      good = parseDouble(v, d) && d == static_cast<int>(d);
      nice_value = static_cast<int>(d);
    } else if (a == "--lock") {
      lock_path = v;
    } else if (a == "--started-at") {
      started_at = v;
    } else {
      good = false;
    }
    if (!good) {
      std::cerr << "[clean_map] bad option " << a << " " << v << "\n";
      usage();
      return 1;
    }
  }
  if (dir_arg.empty()) {
    usage();
    return 1;
  }
  const fs::path dir(dir_arg);
  if (!fs::is_directory(dir)) {
    std::cerr << "[clean_map] " << dir << " is not a directory\n";
    return 2;
  }

  // pgo_node spawns this with every signal at its default and in its own
  // session; the pane it inherited stdout/stderr from may be gone by the time
  // it prints (a mode switch kills the session), and a write to a dead pty
  // must be an EIO to ignore, not a SIGPIPE that ends the clean.
  std::signal(SIGPIPE, SIG_IGN);
  struct sigaction sa
  {
  };
  sa.sa_handler = onStopSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;  // no SA_RESTART: see takeLock
  sigaction(SIGTERM, &sa, nullptr);
  sigaction(SIGINT, &sa, nullptr);
  lowerPriority(nice_value);

  if (started_at.empty()) started_at = recipe::isoUtcNow();
  auto writeFailed = [&](const std::string & error) {
    try {
      recipe::writeAtomic(dir, recipe::failed(params, started_at, recipe::isoUtcNow(), error));
    } catch (const std::exception & e) {
      std::cerr << "[clean_map] " << e.what() << "\n";
    }
    std::cerr << "[clean_map] FAILED: " << error << std::endl;
  };
  try {
    // Again, even when pgo_node already wrote it: a hand run has nobody else
    // to, and the params here are the ones actually in force.
    recipe::writeAtomic(dir, recipe::converting(params, started_at));
  } catch (const std::exception & e) {
    std::cerr << "[clean_map] " << e.what() << "; nothing to report into, giving up\n";
    return 3;
  }

  const int lock_fd = takeLock(lock_path);
  if (lock_fd == -2) {
    writeFailed("interrupted while waiting for another clean");
    return 4;
  }
  // Once more after the lock: the clean that held it may have been an earlier
  // save of this same directory, stopped by pgo_node, and its "failed:
  // interrupted" may have landed after the "converting" above.
  try {
    recipe::writeAtomic(dir, recipe::converting(params, started_at));
  } catch (const std::exception & e) {
    std::cerr << "[clean_map] " << e.what() << "\n";
  }

  std::cout << "[clean_map] cleaning " << dir.string() << " at " << params.resolution
            << " m (max range " << params.max_range << " m)" << std::endl;
  mc::Measurements m;
  int rc = 0;
  try {
    const bool done = mc::clean(dir, params, m, [](const char * stage, size_t done, size_t total) {
      if (done % 100 == 0) {
        std::cout << "[clean_map] " << stage << " " << done << "/" << total << " keyframes"
                  << std::endl;
      }
      return g_stop_signal == 0;
    });
    if (!done) {
      writeFailed(
        std::string("interrupted (") + (g_stop_signal == SIGINT ? "SIGINT" : "SIGTERM") + ")");
      rc = 4;
    } else {
      try {
        recipe::writeAtomic(dir, recipe::ok(params, started_at, recipe::isoUtcNow(), m));
      } catch (const std::exception & e) {
        std::cerr << "[clean_map] map.pcd written but " << e.what() << "\n";
        rc = 3;
      }
      std::cout << "[clean_map] DONE keyframes=" << m.keyframes << " skipped=" << m.skipped_patches
                << " points=" << m.points << " dynamic_voxels=" << m.dynamic_voxels
                << " sparse_voxels=" << m.sparse_voxels
                << " map_points_removed=" << m.map_points_removed
                << " map_points_kept=" << m.map_points_kept << " clipped_rays=" << m.clipped_rays
                << " in " << m.elapsed_s << " s" << std::endl;
    }
  } catch (const mc::InputError & e) {
    writeFailed(e.what());
    rc = 2;
  } catch (const std::bad_alloc &) {
    writeFailed("out of memory; try a coarser resolution or a shorter max range");
    rc = 3;
  } catch (const std::exception & e) {
    writeFailed(e.what());
    rc = 3;
  }
  if (lock_fd >= 0) close(lock_fd);
  return rc;
}
