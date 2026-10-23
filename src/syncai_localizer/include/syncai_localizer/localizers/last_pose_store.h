#pragma once
#include <optional>
#include <string>

// The last trusted map pose, kept on disk so the next AUTO session starts its
// guess where the robot actually is (2026-10-23). New code, no ROS in it; in
// the global namespace like the rest of localizers/.
//
// Why a file of its own and not the INI's [initial_pose]. The instance INI is
// a single-file bind mount (docker-compose.robots.yml), so it cannot be
// replaced by tmp + rename -- a rename over a bind-mounted file fails, and an
// in-place rewrite cut by a power loss takes robot_id and [map] down with the
// pose. The backend also writes [initial_pose], and the two writers would
// overwrite each other. And sys_manager's setup_session runs at container
// start only, so rewriting the INI there would miss every AUTO session built
// by switch_mode / restart_mode. localizer.launch.py reads this file instead
// and falls back to [initial_pose] when it is missing, broken or for another
// map; the INI is never touched.
//
// Format: one JSON object (nlohmann_json here, the json module in the launch),
// {"map", "x", "y", "ground_z", "yaw", "saved_at"} -- the workspace's other
// machine-written sidecars (map_clean.recipe.json, the map_cloud_file notice)
// are JSON too. `map` is the map directory's name, never a path: a sidecar that
// embeds a path breaks the map catalogue's one-rename renames (CLAUDE.md, "No
// file inside a map directory ... holds an absolute path" -- this file is not
// in one, but it is compared against [map] name, which is the same handle).
// `ground_z` is the ground under the body (body z - lidar_height), the same
// meaning an initialpose's marked z has, so a robot shut down on 2F starts
// its guess on 2F.
struct LastPose
{
  std::string map;
  double x = 0.0;
  double y = 0.0;
  double ground_z = 0.0;
  double yaw = 0.0;
  std::string saved_at;  // UTC, ISO 8601; informational only
};

// Write `pose` to `path` atomically: `<path>.tmp`, fsync, rename, fsync of
// the directory, so a power loss leaves the old file or the new one and never
// half of either. Creates the directory if it is missing. `saved_at` is
// filled in here. On failure returns false and says why in `error`.
bool saveLastPose(const std::string & path, LastPose pose, std::string & error);

// Read a file saveLastPose wrote. nullopt (with `error`) when it is missing,
// unreadable, or lacks any of map / x / y / ground_z / yaw, or a number does
// not parse -- a truncated file must never become a guess.
std::optional<LastPose> loadLastPose(const std::string & path, std::string & error);

// The map's name as [map] name spells it, from the PCD path the localizer
// loaded: the directory holding map.pcd (`.../map/dp/map.pcd` -> "dp").
std::string mapNameFromPcdPath(const std::string & pcd_path);
