#ifndef SYNCAI_MAPPING__OCTOMAP_BUILDER_HPP_
#define SYNCAI_MAPPING__OCTOMAP_BUILDER_HPP_

// The post-save OctoMap build: a pgo save directory (patches/<i>.pcd +
// poses.txt) ray-cast into an OctoMap, plus the two display layers the
// operator console draws. Run by build_octomap (build_octomap_main.cpp), which
// pgo_node spawns after a successful save_maps; the README's "The OctoMap
// build" has the process model.
//
// This header deliberately includes no OctoMap or PCL header: pgo_node
// includes it for Params (its ROS parameters) and the output file names, and
// links neither library. Only octomap_builder.cpp does.
//
// Ported from the offline prototype scripts/octomap/{pcd2octomap,export_ply}.cpp
// (2026-10-06, removed when this replaced it) with the insertion order and
// the arithmetic unchanged, so at equal parameters the .bt is byte-identical
// to the prototype's map.bt and the layers hold the same voxels.

#include <cstddef>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

namespace syncai_mapping::octomap_builder
{

// What a save directory gains. Bare names: nothing written into a map
// directory may name the map or hold an absolute path (a rename is one
// os.rename). Each is written as <name>.tmp and renamed into place.
inline constexpr const char * kOctomapFile = "octomap.bt";
inline constexpr const char * kRoadFile = "octomap_road.pcd";
inline constexpr const char * kOccupiedFile = "octomap_occupied.pcd";

// One field per CLI flag, per pgo_node ROS parameter (octomap_<name>) and per
// key of the sidecar's "params" object. Defaults are mapping_params.yaml's.
struct Params
{
  // Voxel edge (m). 0.1, not the gridmap's 0.05: a display layer, and 0.05
  // cost 7 min / 4.5 GB on dp1f_1006 for 8x the voxels.
  double resolution = 0.1;
  // Rays are truncated here (m): free space is carved up to max_range and a
  // point beyond it is not inserted as occupied.
  double max_range = 20.0;
  // Returns closer than this to the keyframe origin are dropped (m) -- the
  // robot's own body.
  double min_range = 0.5;
  // Lidar origin above the floor (m). A keyframe's z minus this is the local
  // floor under it; both layers are cut relative to that.
  double lidar_height = 0.481;
  // Road layer: the lowest free voxel of a column within +-floor_band of the
  // local floor. The occupied layer starts at floor - floor_band too.
  double floor_band = 0.25;
  // The occupied layer stops at floor + max_height (the ceiling is cut).
  double max_height = 2.0;
  // The local floor comes from the nearest keyframe within this radius (m) of
  // a 1 m cell.
  double floor_radius = 10.0;
};

struct Measurements
{
  size_t keyframes = 0;        // patches inserted
  size_t skipped_patches = 0;  // poses.txt lines whose patch could not be read
  size_t points = 0;           // points inserted (after the range filter)
  size_t leaves = 0;
  size_t occupied_leaves = 0;
  size_t free_leaves = 0;
  size_t road_voxels = 0;
  size_t occupied_voxels = 0;
  double road_area_m2 = 0.0;
  double elapsed_s = 0.0;
};

// (keyframes done, keyframes in poses.txt, points so far) -> keep going?
// Called after every keyframe; returning false stops the build with no
// output written (build() then returns false).
using Progress = std::function<bool(size_t, size_t, size_t)>;

// The input is missing or unusable (no poses.txt, no readable patch). Its
// own type so the CLI can exit 2 rather than 3.
class InputError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Read <dir>/poses.txt + <dir>/patches/, ray-cast every patch from its
// keyframe position, and write kOctomapFile, kOccupiedFile and kRoadFile into
// <dir> (each .tmp + rename, in that order; every .tmp is removed on failure).
// Throws InputError or std::runtime_error, whose what() names files RELATIVE
// to dir only -- it ends up in the sidecar, which must hold no absolute path.
// Returns false when the progress callback asked to stop.
bool build(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress = {});

}  // namespace syncai_mapping::octomap_builder

#endif  // SYNCAI_MAPPING__OCTOMAP_BUILDER_HPP_
