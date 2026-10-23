#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__KEEPOUT_ZONES_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__KEEPOUT_ZONES_HPP_

// The keepout (forbidden) zones of a 3D map (2026-10-23): what
// map/<name>/keepout.yaml + keepout.pgm are to the 2D global costmap's
// KeepoutFilter, for a map that Octo3D plans on.
//
// Why vector polygons and not a raster mask like the 2D one. A 3D map has no
// gridmap to size a mask from (octomap.bt's header carries no bounding box),
// a site map has several storeys, and one 2D image cannot say which floor a
// pixel forbids: the corridor on 1F and the room above it share an (x, y).
// So the zones are stored the way every fleet system and quadruped vendor
// stores them -- polygons in the map frame, each with the floor it applies
// to, here as a z band -- in map/<name>/keepout.json beside octomap.bt:
//
//   { "version": 1,
//     "zones": [ { "id": "zone-7f3a", "name": "server rack 2F",
//                  "points": [{"x": 12.4, "y": 3.1}, {"x": 15.0, "y": 3.1},
//                             {"x": 15.0, "y": 5.6}, {"x": 12.4, "y": 5.6}],
//                  "z_min": 4.5, "z_max": 5.5 } ] }
//
// ONE FILE FOR 2D AND 3D (2026-10-23). This is the file the backend already
// writes for a 2D map's zones beside the keepout.yaml / keepout.pgm mask it
// rasterises from it (SyncAI-Robot-Backend, repositories/map/catalog.py:
// write_keepout) -- the same `points` shape -- with three optional keys
// added: `name`, `z_min`, `z_max`. A zone without a band is unbounded in z,
// i.e. every floor, which is what a 2D zone means; so a file written by the
// 2D editor applies unchanged on a 3D map, and the 2D mask ignores the bands
// (one image has no floors). A first cut read `"polygon": [[x, y], ...]`,
// which collided with the backend's file of the same name and version; it
// never shipped.
//
// `zones` is required (and may be empty); `version` is optional (absent = 1,
// a larger one is refused as written by a newer backend). Per zone: `id` is
// required, non-empty and unique in the file (the logs and the failure
// details name zones by it); `name` is for display; `points` is three or
// more {"x", "y"} objects in metres, map frame, implicitly closed (a last
// vertex equal to the first is dropped), either winding, even-odd if it
// crosses itself; `z_min` / `z_max` are optional (absent or null = unbounded
// on that side, z_min > z_max refused). Every number must be finite.
// Unknown keys are ignored, so a later backend may annotate. Nothing in the
// file names the map or holds a path, so a map directory still renames with
// one os.rename.
//
// What a zone means on the surface map. A surface cell is in the zone's
// CORE when its column centre lies inside the polygon and its top face lies
// in the band (z_min <= zTop <= z_max): the band picks the floor, so the same
// polygon with band 4.5..5.5 forbids 2F and leaves 1F alone (a backend
// writes floor_z +- 0.5). The core is then dilated over the surface by
// block_radius -- the robot's radius, the costmap's inscribed radius, what
// the 2D KeepoutFilter inflates its mask with -- so a zone closes any gap
// narrower than the robot exactly as a wall does and the operator draws the
// forbidden area only, no margin; and costed out to clearance_cost_radius
// beyond that with the walls' own curve. The dilation is the dynamic
// overlay's (dilateOnSurface): it follows a ramp and stops at a floor edge
// the way a mark's does, so one floor's zone never reaches the other. A stair
// link any of whose points (its own ground height, not a cell's -- a
// flight's cells are all rejected) lies in a core is CUT: the search never
// expands it, so a zone over a stair foot forbids that stair; the linked
// groups are not recomputed, so such a route fails no_path with the zones
// named, not disconnected.
//
// The result is a KeepoutMask (dynamic_overlay.hpp), read through
// BlockedView beside the dynamic overlay's mask by the goal snap, the
// search, the cost, the shortcut and is_path_valid, so nothing downstream
// knows a zone from a person -- except that a robot standing inside a core
// is refused a plan (start_in_keepout; the 2D planner fails on a lethal
// start the same way) and the operator drives it out.
//
// ROS-free (see surface_map.hpp); only keepout_zones.cpp sees nlohmann/json.

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/stair_links.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"

namespace syncai_global_planner::octomap_planner_3d
{

struct KeepoutZone
{
  std::string id, name;
  std::vector<std::pair<double, double>> polygon;  // >= 3 vertices, metres, map frame
  double z_min = -std::numeric_limits<double>::infinity();
  double z_max = std::numeric_limits<double>::infinity();
};

// Parse the file's text by the rules in the header. False with `error`
// naming the zone (by index, or id once it has one) and the key on any
// violation; `zones` is then untouched.
bool parseKeepoutJson(
  const std::string & text, std::vector<KeepoutZone> & zones, std::string & error);

// Read and parse `path`. False with `error` on an unreadable file too; the
// caller decides "absent" with std::filesystem first, so that absent and
// broken are told apart.
bool readKeepoutFile(
  const std::string & path, std::vector<KeepoutZone> & zones, std::string & error);

// Even-odd point-in-polygon on (x, y), and z within [z_min, z_max].
bool zoneContains(const KeepoutZone & z, double x, double y, double zz);

// Rasterise the zones onto `m`: every cell in a core is a seed, dilated with
// discOffsets(block_radius + m.params.clearance_cost_radius); link_cut is
// set per link of `links` (null = no links). nullptr when nothing was hit --
// no core cell and no link cut -- which is the consumers' fast path.
KeepoutMaskPtr buildKeepoutMask(
  const SurfaceMap & m, const StairLinks * links, const std::vector<KeepoutZone> & zones,
  double block_radius);

// The first zone whose core holds cell c (re-tests the polygons; for
// messages), or nullptr.
const KeepoutZone * zoneAt(const std::vector<KeepoutZone> & zones, const SurfaceMap & m, CellId c);

// "'<id>'" or "'<id>' (<name>)", for messages.
std::string zoneLabel(const KeepoutZone & z);

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__KEEPOUT_ZONES_HPP_
