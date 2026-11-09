#pragma once
#include "syncai_pointlio/map_builder/commons.h"

// Whether a scan's points go into the local map (2026-11-09).
//
// Every MAPPING scan used to be inserted, whatever pose it had been placed
// with. On 2026-11-08 at 16:16 robot01 stood at P4 while Point-LIO ran north
// at ~2 m/s for 37 m, and every scan of the run was inserted at the wrong
// pose: ~400 new 2 m grids in 20 s, a shifted copy of the corridor that the
// next scans matched, which kept the run going and left the filter locked on
// the copy 42 m off (docs/pointlio-divergence-2026-11-08.md §3.3). Kept out
// of the map, a wrongly placed scan leaves nothing to hold the pose wrong: the
// gate cuts that loop, it does not stop the slide that starts it.
//
// A scan fails when
//  - the speed at its end is over map_gate_max_speed. The robot tops out at
//    0.8 m/s; over robot01's 11 h on 11-08 before the slides the largest
//    speed of any walking second was 1.45 m/s, and the run-away passed 1.5 at
//    16:16:03, six seconds before the robot stood still and with four grids
//    of the copy built;
//  - or of its points with map neighbours (`near`), the share that fit a plane
//    (`effective`) is under map_gate_min_effective_ratio. Over near, not over
//    all points like the health line's eff: a scan entering a place the map
//    has not seen has few near points, and judged over all of them it would
//    fail, the place would never be written and never become mapped. In the
//    same 11 h the worst walking scan of every second stayed at 86 % or
//    more (near was 100 % throughout, so both shares agree there); the
//    run-away's fell to 81 % at 16:16:04 and to 47 % ten seconds later. A scan
//    with fewer than map_gate_min_near near points is not judged on it: at 50
//    points one standard error of a 0.9 share is already 4 %, as much as the
//    margin between the threshold and a healthy scan.
// A failing scan is never inserted. Both thresholds sit close to what a healthy
// robot reaches, which is affordable because one scan left out costs nothing:
// the next one covers the same place. After map_gate_close_after failing
// scans in a row the gate closes, and then no scan is inserted, failing or not,
// until map_gate_open_after pass in a row: a filter hovering at the thresholds
// does not write every other scan, and after a run-away the pose has to hold a
// match for a while before the map trusts it again. Nothing but passing scans
// reopens it -- a gate that stays closed means Point-LIO no longer matches its
// own map, and the way out is a reset (the localizer's reset_odometry), not
// writing the scans anyway.
//
// Those 11 h replayed second by second through the defaults (each second as ten
// scans, one at its worst share and top speed, nine at the rest): no scan of
// ~72 000 while moving healthy kept out; of the ~2 800 scans from 16:15:15 to
// the end of the log, where every one went in, 139, the last at 16:16:02 --
// the gate closed at :03 and stayed shut for the four minutes after. Reopening
// after 50 rather than 10 is what holds it shut there (684 inserted at 10)
// and costs nothing on a healthy run, which never closes it. It is not a slide
// detector: the 16:05 slide that began the damage still
// writes 229 of ~420 scans, its share sitting at 88-91 % once under way --
// above the worst healthy scans, so no threshold catches it without keeping
// healthy scans out (0.92 would: 227 of them). Thresholds measured on the old,
// leaky map; the replay on the one-point-per-voxel 0.2 m map (worst walking
// scan p1 89 %) is still to confirm them.
class MapInsertGate
{
public:
  // The verdict on one scan. speed / ratio are what was judged; ratio is -1
  // when the scan had too few near points to judge it.
  struct Verdict
  {
    bool insert = true;
    bool speed_failed = false;
    bool ratio_failed = false;
    double speed = 0.0;
    double ratio = -1.0;
    int near = 0;
    int effective = 0;
    // This scan closed the gate / reopened it.
    bool closed_now = false;
    bool opened_now = false;
  };

  explicit MapInsertGate(const Config & config);

  // Judge one MAPPING scan: its speed at the end (world frame, m/s) and the
  // lidar update's near / effective counts (LidarUpdateStats).
  Verdict judge(double speed, int near, int effective);

  bool isOpen() const { return m_open; }

private:
  bool m_enabled;
  double m_max_speed;
  double m_min_effective_ratio;
  int m_min_near;
  int m_close_after;
  int m_open_after;

  bool m_open = true;
  // Failing scans in a row while open, passing scans in a row while closed.
  int m_streak = 0;
};
