#ifndef SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__STAIR_LINKS_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__STAIR_LINKS_HPP_

// Stairs the robot has demonstrably walked, as extra edges of the search.
//
// The surface map refuses every stair on purpose (surface_map.hpp): a riser
// is a step of more than one cell and a flight is steeper than max_slope_deg,
// so each floor and each landing is a component of its own. Relaxing those
// rules map-wide would make every 0.2 m kerb, drop-off and steep slab
// traversable too. Instead the stairs come from the map's own track: the
// keyframes in poses.txt are where the robot stood while the map was made,
// so a stretch of that track which leaves one component and arrives in
// another, a storey's worth of height away, is a stair the legs have taken.
// Each such stretch becomes a link between the traversable cells at its two
// ends, carrying the keyframes' ground points in between; the A* may take it
// like any move (cost = its 3D length times stair_cost_multiplier), and the
// path follows the recorded points through the flight. Links are used in
// both directions. Stairs the robot never walked stay refused.
//
// A link is made between two consecutive "anchors" of the track --
// keyframes whose ground point is within one cell (xy) and anchor_z_window
// (z) of a traversable cell in a component of at least anchor_min_area --
// when their components differ, they are at least min_rise apart in height,
// and the track between them is at most max_length long. Only large
// components anchor because a stairwell's own floor rarely survives the
// geometry rules: on dp1f_1012 the three landings of a switchback stair came
// out as a few square metres of islands or not at all (small, walled, the
// air above the landing never observed), and the floor at the stair head was
// cut into pieces by the footprint rule. So a link runs from one storey's
// floor to the next storey's floor over the whole stairwell, landings and
// all, and an island on the way neither ends nor splits it. A small rise
// between two large components (a door the footprint rule closed) makes no
// link.
//
// The track says WHERE a stair is; since 2026-10-19 it no longer says how
// to climb it. A single flight (one walked stretch of at most
// straight_max_flight of track) becomes a row of **straight links** across
// the stair's width instead of one link along the walk: the surfaces
// beside the walked stretch are fitted with a plane, whose gradient is the
// stair's uphill direction, and from every traversable cell of the lower
// floor near the walked foot a line is marched uphill along that direction,
// column by column, over surfaces that stay within one step of each other
// (traversable or not -- the flight's own cells are all rejected), until it
// steps onto a traversable cell of the upper floor. Each line that gets
// there is a link. The search then picks the entry, so a route meets the
// stair where its own straight line crosses it, climbs facing the risers
// and leaves toward the goal, instead of detouring to the one cell the
// operator happened to step off from and copying the walk's drift (on dp
// the walked line was 9 deg off the stair and the path turned 27 deg at its
// head). A stretch that is not one straight flight -- longer than
// straight_max_flight, a switchback whose landings turn, a plane fit that
// disagrees with the walk -- keeps the walked link below, and the notes say
// so.
//
// A stretch that is not one straight flight -- dp's switchback stairwell --
// is since 2026-10-21 laid as **flights and landings** read off the surface
// map, not as the walk: the walk's height profile is cut into level runs
// (landings, and the two floors) and rises (flights); every flight gets the
// uphill direction of a plane fitted to its treads (fitUphill, as a straight
// flight does) and a centre axis, the median of the cross-sections square
// to that direction (sideReach); its foot and head are where a march along
// the axis reaches the landings' heights. Consecutive flights are joined on
// the landing with explicit corners: one at the axes' intersection when the
// flights are ~90 deg apart (an L), two -- out onto the landing, across
// square to the flights, and in, as far out as leaves the body most room to
// turn in place against the walls the octree holds at body height -- when
// they are ~180 deg apart (a U), so the path climbs
// each flight straight along its centre, stops, turns in place and climbs
// the next, as the legs want to; a 0.5 m lead-in before the first foot and
// lead-out past the last head line the robot up before the first riser and
// take it clear of the last. Nothing on that polyline is smoothed: the
// corners are the point. The turn vertices are recorded in StairLink::turns
// for the controllers' stair mode. A stretch the profile cannot be read from
// (a flight whose plane does not fit, a landing whose shape is neither L nor
// U) falls back to the walked link below, and the notes say why.
//
// The points of a walked link are the keyframes' ground points with every
// loop cut out (the track coming back within loop_radius and loop_dz of an
// earlier point): a robot driven by hand turns and waits on landings, and
// the path should not. What is left is then smoothed (smooth_passes of a
// three-point moving average, the ends fixed), because a hand-driven track
// also weaves by a few centimetres from keyframe to keyframe and the body
// pitches on a flight, which the ground estimate (pose z - lidar_height)
// inherits. A straight link's points lie on its line, z interpolated
// between its two ends: the flight as one ramp, which is what the
// controller's stair mode reads it as anyway.
//
// A walked link is then **centred** in the stairwell (2026-10-20): the
// operator walks a stair wherever they like -- close to the rail, cutting
// the landing's inside corner -- and the path copied that line. At every
// interior point the cross-section square to the walk is read off the
// surface map: from the point, column by column to either side, as long as
// the column holds a surface within centre_dz of the point's own tread, in
// any state: every tread of a flight is rejected by the step and slope rules,
// and on dp most of them and every landing also by the clearance rules (the
// rail and the flight overhead in the band, the landing's floor never seen
// from above), so the state says nothing about where the well ends. A wall,
// or the drop to the neighbouring flight of a switchback, ends a side. Where
// both sides end within centre_max_half_width the point moves to the middle,
// at most centre_max_shift; where either side runs out (a floor, a landing
// open to a corridor) it stays, so the link's ends on the two floors are
// untouched. The shifts are smoothed along the link before they are applied,
// so one odd column does not kink the path.
//
// What could not be linked is reported in StairLinks::notes, for the plugin
// to log and the CLI to print: a height change on the track that stayed
// unlinked because its stretch was longer than max_length, a flight that
// fell back to its walked link and why, and large components at different
// heights that no chain of links joins.
//
// ROS-free (see surface_map.hpp).

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"

namespace syncai_global_planner::octomap_planner_3d
{

struct StairLinkParams
{
  // Height difference between the two ends for a link to be made. Above a
  // kerb (0.15) and any one stair step, below one flight between landings.
  double min_rise = 0.3;
  // Longest stretch of track one link may stand for (m, xy, before the loop
  // cut). dp1f_1012's four flights took 32 m of track with the turns and
  // waits on the landings; this only stops a long loss of anchors (the track
  // over a stretch of untraversable floor) from becoming one long edge.
  double max_length = 60.0;
  // A keyframe's ground point anchors to a traversable cell whose top face is
  // within this of it (the track's ground is pose z - lidar_height; the
  // floor's top face differs by up to a couple of cells).
  double anchor_z_window = 0.3;
  // Smallest component (m2) a link may end in. dp1f_1012: the two storeys
  // are 1200 and 330 m2, the stairwell's islands at most ~11 m2.
  double anchor_min_area = 10.0;
  // Loop cut: a later point within loop_radius (xy) and loop_dz (z) of an
  // earlier one closes a loop, and everything in between is dropped.
  double loop_radius = 0.4;
  double loop_dz = 0.15;
  // Three-point moving-average passes over the link's points (0 = raw).
  int smooth_passes = 2;
  // Straight links across the stair's width (see the header). false = one
  // link along the walk, the 2026-10-14 behaviour.
  bool straight = true;
  // Longest stretch of track (m, xy) that may be one straight flight. dp's
  // 2F flight is 1.5 m of track; a switchback stairwell is tens of metres
  // and keeps its walked link.
  double straight_max_flight = 8.0;
  // The plane is fitted to the surfaces within this of the walked stretch
  // whose height lies strictly between the two floors.
  double straight_fit_radius = 1.0;
  // Entries are tried on the lower floor within this of the walked foot:
  // sideways, along the stair's edge (its width), and a metre back and
  // forth along the climb direction, because the walked anchor is only
  // roughly on the edge.
  double straight_half_width = 2.0;
  // A line may run level for at most this long between two rises: the
  // footprint rule leaves ~robot_radius of blocked floor before the first
  // riser and after the last, and that is what this crosses; a landing
  // longer than it ends the flight (the walked link stands in).
  double straight_exit_reach = 0.6;
  // Lay a multi-flight stretch as flights and landing corners (see the
  // header). false = the walked link (centred below, when enabled).
  bool flights = true;
  // A level run of the walk at least this long (m, xy) is a landing; the
  // first and last runs are the floors whatever their length.
  double flights_landing_min_len = 0.5;
  // A segment of the walk flatter than this is level. dp's flights are
  // ~30 deg, its landings under 5 deg.
  double flights_landing_slope_deg = 15.0;
  // The straight lead-in before the first foot and lead-out past the last
  // head (m), along the flight's axis.
  double flights_lead = 0.5;
  // The disc the body sweeps turning in place at a U landing's corners:
  // the padded footprint's circumscribed radius, sqrt(0.38^2 + 0.25^2).
  // The corners go where the landing leaves the most room round it (see
  // the header); the leg between them needs robot_radius either side.
  double flights_turn_radius = 0.455;
  // Centre a walked link in the stairwell (see the header). false = the
  // operator's line, the 2026-10-19 behaviour.
  bool centre = true;
  // A side of the cross-section ends at the first column whose nearest
  // surface is more than this above or below the point's tread: more than a
  // riser's worth (a tread is level across, but the walk is not square to
  // the risers), less than the half-storey to the neighbouring flight.
  double centre_dz = 0.25;
  // A side longer than this is open (a floor or a landing that opens onto a
  // corridor), and the point is not moved.
  double centre_max_half_width = 1.5;
  // Largest shift of one point (m).
  double centre_max_shift = 0.6;
};

struct StairLink
{
  CellId a = kNoCell, b = kNoCell;  // the end cells; a is on the side walked first
  std::vector<Waypoint> pts;        // a's centre .. b's centre: the walk, or a straight line
  double length = 0.0;              // 3D length of pts
  size_t kf_a = 0, kf_b = 0;        // the keyframes that found the stair (poses.txt line index)
  bool straight = false;            // a line across the fitted flight, not the walked track
  bool flights = false;             // flights + landing corners read off the map (see the header)
  std::vector<size_t> turns;        // indices into pts where the robot turns in place (flights)
  size_t n_flights = 0;             // flights only
  std::string geometry;             // flights only: one line for the logs (widths, landings, turns)
  double up_x = 0.0, up_y = 0.0;    // the flight's uphill unit direction (straight links only)
  // Centring of a walked link (see the header): how many points moved, the
  // mean and largest shift, and the narrowest cross-section measured.
  size_t centred = 0;
  double centre_mean_shift = 0.0, centre_max_shift = 0.0, centre_min_width = 0.0;
};

// One walked flight that became a row of straight links, for the logs.
struct StairFlight
{
  size_t kf_a = 0, kf_b = 0;
  uint32_t first = 0, count = 0;  // links[first .. first + count)
  double heading_deg = 0.0;       // uphill direction, map frame
  double width = 0.0;             // spread of the entries across the stair (m)
  double z_lo = 0.0, z_hi = 0.0;
};

// One use of a link in a planned route: the move from cells[index] to
// cells[index + 1] of a PlanResult goes over link `link`, from b to a when
// reversed.
struct LinkUse
{
  size_t index = 0;
  uint32_t link = 0;
  bool reversed = false;
};

class StairLinks
{
public:
  std::vector<StairLink> links;
  std::vector<StairFlight> flights;  // the straight links, grouped by the flight they cross
  // Links touching a cell, by link index.
  std::unordered_map<CellId, std::vector<uint32_t>> at;
  // component -> group: components joined by links share a group. Sized to
  // the map's component count; a component no link touches is its own group.
  std::vector<uint32_t> group;
  // What stayed unlinked, one sentence each (see the header).
  std::vector<std::string> notes;

  uint32_t groupOf(uint32_t component) const
  {
    return component < group.size() ? group[component] : component;
  }
};

// Ground points of poses.txt (name x y z qw qx qy qz per line): (x, y,
// z - lidar_height). False with error set when the file cannot be read.
bool readTrack(
  const std::string & poses_path, double lidar_height, std::vector<Waypoint> & out,
  std::string & error);

// The links of a track over a built map. Never null; empty without links.
// tree is the map's octree, when the caller still has it: the U landings'
// corners are placed against the walls it holds at body height (see
// landingObstacle in the .cpp), which the surface map cannot tell from a
// landing whose floor was never seen. Without it they are placed against
// the surfaces alone, which misses a wall that runs into the slab above.
std::shared_ptr<const StairLinks> buildStairLinks(
  const SurfaceMap & m, const std::vector<Waypoint> & track, const StairLinkParams & params,
  const octomap::OcTree * tree = nullptr);

}  // namespace syncai_global_planner::octomap_planner_3d

#endif  // SYNCAI_GLOBAL_PLANNER__PLUGINS__OCTOMAP_PLANNER_3D__STAIR_LINKS_HPP_
