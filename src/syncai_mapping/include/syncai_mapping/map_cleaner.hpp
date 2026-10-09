#ifndef SYNCAI_MAPPING__MAP_CLEANER_HPP_
#define SYNCAI_MAPPING__MAP_CLEANER_HPP_

// The post-save map cleaning: a pgo save directory (patches/<i>.pcd +
// poses.txt) ray-cast keyframe by keyframe to find the voxels that are people
// or one-off returns, and map.pcd rewritten without their points. Run by
// clean_map (clean_map_main.cpp), which pgo_node spawns after a successful
// save_maps with save_patches; the README's "Cleaning map.pcd" has the
// process model and the numbers.
//
// Offline on purpose (decided 2026-10-09: the cleanest final map over a clean
// live preview). A voxel is a person only once a LATER scan sees through it,
// often from another angle or on a revisit, and the judgement needs the
// final, loop-closed poses: before closure the two passes of dp1f_1008_2's
// east corridor were 1.2-2 m apart, and an online check on those poses would
// have deleted real wall at every revisit. Both exist only after the save.
//
// Prototyped as part of the OctoMap display build on
// feat/mapping-octomap-loop-noise (octomap_builder.cpp); this is its cleaning
// half without the OctoMap outputs. OctoMap is still used, but only for its
// key arithmetic and ray traversal: no tree is ever built, so the memory is
// the hit-voxel hash map alone.
//
// This header deliberately includes no OctoMap or PCL header: pgo_node
// includes it for Params (its ROS parameters) and the file names, and links
// neither library. Only map_cleaner.cpp does.

#include <cstddef>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

namespace syncai_mapping::map_cleaner
{

// pgo writes map.pcd at save time, raw; the clean REPLACES it with the same
// points minus the dynamic and sparse voxels below, via map.pcd.tmp + rename
// (an open reader keeps the old file). patches/ stay raw: they are the input,
// and a rerun by hand re-derives the cleaned map from them.
inline constexpr const char * kMapFile = "map.pcd";

// One field per CLI flag, per pgo_node ROS parameter (map_clean_<name>) and
// per key of the sidecar's "params" object. Defaults are mapping_params.yaml's.
struct Params
{
  // Voxel edge (m): the unit a point is judged in.
  double resolution = 0.1;
  // Rays are truncated here (m): a point beyond it counts no hit, and the
  // voxels its ray crosses past max_range count no miss.
  double max_range = 20.0;
  // Returns closer than this to the keyframe origin are ignored (m) -- the
  // robot's own body. They stay in map.pcd: nothing judged them.
  double min_range = 0.5;
  // Lidar origin above the floor (m). A keyframe's z minus this is the floor
  // under it; the rules below are cut relative to that.
  double lidar_height = 0.481;
  // The floor band (+- m around the local floor) where neither rule applies:
  // the floor is a real surface the lidar samples thinly (12 % of a
  // keyframe's points on dp1f_1008_2), so a single hit there is the norm --
  // the sparse rule took 17 % of a corridor floor before this exemption --
  // and grazing rays carve real floor voxels "free", which would read as
  // dynamic.
  double floor_band = 0.25;
  // People are looked for up to floor + max_height (m).
  double max_height = 2.0;
  // The local floor comes from keyframes within this radius (m) of a 1 m cell.
  double floor_radius = 10.0;

  // Counts are per scan -- a keyframe hits or crosses a voxel at most once --
  // never per ray, which would scale with 1/range^2.
  //
  // A voxel hit by fewer than min_hits keyframes is "sparse": smear, an edge
  // return, a reflection. 92 % of the occupied voxels outside one solid wall
  // of dp1f_1008_2's east corridor were single-keyframe; real wall voxels are
  // hit by a median of 4-5. <= 1 turns this off.
  int min_hits = 2;
  // A voxel hit H >= 1 times and passed through M times is "dynamic" when
  // M >= dynamic_min_miss and M >= dynamic_miss_ratio * H, and it lies
  // between floor + floor_band and floor + max_height: a person, hit by a
  // keyframe or two and seen through by the rest. 2.0 is about OctoMap's own
  // hit / miss log-odds ratio (0.85 / 0.41). Thin static structure (a
  // handrail, a chair leg) has the same signature; measure dynamic_voxels
  // before tightening this. dynamic_min_miss <= 0 turns this off.
  double dynamic_miss_ratio = 2.0;
  int dynamic_min_miss = 3;
  // Rays that would pass more than (floor_band - resolution) below their own
  // keyframe's floor stop there, and their returns count as "clipped" hits
  // rather than hits. About 0.4 % of the returns on dp1f_1008_2 land under
  // the floor -- long range, half the usual intensity: specular reflections
  // off a glossy floor -- and their rays would count misses for voxels that
  // are physically under a floor. A clipped return still counts toward
  // min_hits, so a one-off reflection leaves map.pcd while a lower level seen
  // from many keyframes (stairs down) stays.
  bool clip_below_floor = true;
};

struct Measurements
{
  size_t keyframes = 0;        // patches read
  size_t skipped_patches = 0;  // poses.txt lines whose patch could not be read
  size_t points = 0;           // points judged (after the min_range cut)
  size_t clipped_rays = 0;     // rays stopped at the floor
  size_t dynamic_voxels = 0;
  size_t sparse_voxels = 0;
  size_t map_points_removed = 0;
  size_t map_points_kept = 0;
  double elapsed_s = 0.0;
};

// (stage, keyframes done in it, keyframes in poses.txt) -> keep going? Stages
// "hits", "misses", "map". Called after every keyframe; returning false stops
// the clean with map.pcd untouched (clean() then returns false).
using Progress = std::function<bool(const char *, size_t, size_t)>;

// The input is missing or unusable (no poses.txt, no readable patch). Its
// own type so the CLI can exit 2 rather than 3.
class InputError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Read <dir>/poses.txt + <dir>/patches/, count and classify, and replace
// <dir>/map.pcd (map.pcd.tmp + rename, the .tmp removed on failure).
// poses.txt is fingerprinted at the start and re-checked before the rename: a
// save into the same directory mid-clean fails this one ("superseded")
// instead of letting it rename the OLD save's map over the new one. Throws
// InputError or std::runtime_error, whose what() names files RELATIVE to dir
// only -- it ends up in the sidecar, which must hold no absolute path.
bool clean(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress = {});

}  // namespace syncai_mapping::map_cleaner

#endif  // SYNCAI_MAPPING__MAP_CLEANER_HPP_
