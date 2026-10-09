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
// pgo writes map.pcd at save time, raw; the build REPLACES it with the same
// points minus the dynamic and sparse voxels below (when either filter is on),
// .tmp + rename like the rest. patches/ stay raw: they are the input, and a
// rebuild by hand re-derives everything from them.
inline constexpr const char * kMapFile = "map.pcd";

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

  // ---- Voxel cleaning (2026-10-09) ----
  //
  // OctoMap's own integration keeps a voxel occupied once it was hit and never
  // seen through, and nothing ever looks through the far side of a wall: on
  // dp1f_1008_2's east corridor 92 % of the occupied voxels outside the solid
  // wall were hit by exactly ONE keyframe (scan smear, glass, a return off an
  // edge), while real wall voxels are hit by a median of 4-5. People walking
  // the corridor show up the other way round: hit by one or two keyframes and
  // passed through by many more (89 % of the corridor's 0.3-1.8 m voxels were
  // single-keyframe). Counts are per scan -- a keyframe hits or crosses a voxel
  // at most once, as `discretize` already dedups -- never per ray, which would
  // scale with 1/range^2.
  //
  // A voxel hit by fewer than min_hits keyframes is "sparse" and is freed --
  // except inside +-floor_band of the local floor, which the lidar samples so
  // thinly that a single hit is the norm there. <= 1 turns this off.
  int min_hits = 2;
  // A voxel hit H >= 1 times and passed through M times is "dynamic" when
  // M >= dynamic_min_miss and M >= dynamic_miss_ratio * H, and it lies between
  // floor + floor_band and floor + max_height -- where people are, and above
  // the floor band, where grazing rays carve sparse floor voxels free and the
  // floor itself would otherwise read as dynamic. 2.0 is about OctoMap's own
  // hit / miss log-odds ratio (0.85 / 0.41), so the cleaned map.pcd agrees
  // with the tree's verdict. Thin static structure (a pole, a chair leg) has
  // the same signature; measure dynamic_voxels before tightening this.
  // dynamic_min_miss <= 0 turns this off.
  double dynamic_miss_ratio = 2.0;
  int dynamic_min_miss = 3;

  // Rays that would pass below the floor under their own keyframe (keyframe
  // z - lidar_height - (floor_band - resolution), one voxel inside the band)
  // are cut there: free space is carved down to
  // that height and no further, and their endpoint is not inserted. Returns
  // below the floor are a fraction of a percent (0.4 % on dp1f_1008_2) --
  // long range (median 15 m) and half the usual intensity, i.e. mostly
  // specular reflections off the floor -- but each one's ray runs metres under
  // the floor, and a single miss makes an unknown voxel free. Free space under
  // the floor disqualifies a column from the road layer (its lowest free voxel
  // must sit on something not free), so on dp1f_1008_2's east corridor 44 % of
  // the columns dropped out once the input was 3x denser. Their endpoints are
  // still counted as hits for the cleaning, so a one-off ghost leaves map.pcd
  // while a real lower level seen many times (stairs down) stays.
  bool clip_below_floor = true;

  bool cleaning() const { return min_hits > 1 || dynamic_min_miss > 0; }
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
  // Voxel cleaning (zero when it is off): voxels freed per class, and the
  // map.pcd points dropped / kept by the rewrite.
  size_t dynamic_voxels = 0;
  size_t sparse_voxels = 0;
  size_t map_points_removed = 0;
  size_t map_points_kept = 0;
  bool map_pcd_rewritten = false;
  size_t clipped_rays = 0;  // rays cut at the floor (clip_below_floor)
  double elapsed_s = 0.0;
};

// (stage, keyframes done in it, keyframes in poses.txt, points so far) ->
// keep going? Stages: "insert", then with cleaning on "count", "map". Called
// after every keyframe; returning false stops the build with no output
// written (build() then returns false).
using Progress = std::function<bool(const char *, size_t, size_t, size_t)>;

// The input is missing or unusable (no poses.txt, no readable patch). Its
// own type so the CLI can exit 2 rather than 3.
class InputError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Read <dir>/poses.txt + <dir>/patches/, ray-cast every patch from its
// keyframe position, and write kOctomapFile, kOccupiedFile and kRoadFile into
// <dir> -- and, with cleaning on, kMapFile -- each .tmp + rename, in that
// order; every .tmp is removed on failure. poses.txt is fingerprinted at the
// start and re-checked before the renames: a save into the same directory
// mid-build fails this one ("superseded") instead of letting it rename its
// outputs, the old map.pcd among them, over the new save.
// Throws InputError or std::runtime_error, whose what() names files RELATIVE
// to dir only -- it ends up in the sidecar, which must hold no absolute path.
// Returns false when the progress callback asked to stop.
bool build(
  const std::filesystem::path & dir, const Params & params, Measurements & out,
  const Progress & progress = {});

}  // namespace syncai_mapping::octomap_builder

#endif  // SYNCAI_MAPPING__OCTOMAP_BUILDER_HPP_
