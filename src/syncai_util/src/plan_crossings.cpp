#include "syncai_util/plan_crossings.hpp"

#include <nlohmann/json.hpp>

namespace syncai_util
{

std::string toJson(const PlanCrossings & c)
{
  nlohmann::ordered_json j;
  j["version"] = 1;
  j["known"] = c.known;
  j["plan_stamp"] = {{"sec", c.stamp_sec}, {"nanosec", c.stamp_nanosec}};
  j["frame_id"] = c.frame_id;
  j["poses"] = c.poses;
  nlohmann::ordered_json list = nlohmann::ordered_json::array();
  for (const PlanCrossing & x : c.crossings) {
    nlohmann::ordered_json e;
    e["kind"] = x.kind;
    e["gait"] = x.stair_gait ? "stair" : "flat";
    if (x.slow) {
      e["slow"] = true;  // absent = false, so a reader of the 2026-11-02 format reads on
    }
    e["evidence"] = x.evidence;
    e["rise"] = x.rise;
    e["first"] = x.first;
    e["last"] = x.last;
    e["turns"] = x.turns;
    nlohmann::ordered_json pts = nlohmann::ordered_json::array();
    for (const PlanCrossing::Point & p : x.pts) {
      pts.push_back({p.x, p.y, p.z});
    }
    e["pts"] = std::move(pts);
    if (x.width > 0.0) {
      e["width"] = x.width;
      e["heading"] = x.heading;
    }
    list.push_back(std::move(e));
  }
  j["crossings"] = std::move(list);
  return j.dump();
}

bool fromJson(const std::string & json, PlanCrossings & out, std::string & error)
{
  // allow_exceptions=false, as keepout_zones.cpp parses: a bad message is a
  // false with a reason, never a throw in a subscription callback.
  const nlohmann::json j = nlohmann::json::parse(json, nullptr, false);
  if (j.is_discarded() || !j.is_object()) {
    error = "not a JSON object";
    return false;
  }
  const auto ver = j.find("version");
  if (ver == j.end() || !ver->is_number_integer() || ver->get<int>() != 1) {
    error = "version is not 1";
    return false;
  }
  PlanCrossings c;
  const auto known = j.find("known");
  if (known == j.end() || !known->is_boolean()) {
    error = "'known' is not a boolean";
    return false;
  }
  c.known = known->get<bool>();
  const auto st = j.find("plan_stamp");
  if (st != j.end()) {
    if (
      !st->is_object() || !st->contains("sec") || !st->contains("nanosec") ||
      !(*st)["sec"].is_number_integer() || !(*st)["nanosec"].is_number_integer()) {
      error = "'plan_stamp' is not {sec, nanosec}";
      return false;
    }
    c.stamp_sec = (*st)["sec"].get<int32_t>();
    c.stamp_nanosec = (*st)["nanosec"].get<uint32_t>();
  }
  if (j.contains("frame_id") && j["frame_id"].is_string()) {
    c.frame_id = j["frame_id"].get<std::string>();
  }
  if (j.contains("poses") && j["poses"].is_number_unsigned()) {
    c.poses = j["poses"].get<size_t>();
  }
  const auto list = j.find("crossings");
  if (list == j.end() || !list->is_array()) {
    error = "'crossings' is not an array";
    return false;
  }
  for (size_t i = 0; i < list->size(); ++i) {
    const nlohmann::json & e = (*list)[i];
    const std::string where = "crossing " + std::to_string(i) + ": ";
    if (
      !e.is_object() || !e.contains("kind") || !e["kind"].is_string() || !e.contains("gait") ||
      !e["gait"].is_string() || !e.contains("first") || !e["first"].is_number_unsigned() ||
      !e.contains("last") || !e["last"].is_number_unsigned()) {
      error = where + "needs a string kind and gait and unsigned first and last";
      return false;
    }
    PlanCrossing x;
    x.kind = e["kind"].get<std::string>();
    const std::string gait = e["gait"].get<std::string>();
    if (gait != "stair" && gait != "flat") {
      error = where + "gait '" + gait + "' is neither stair nor flat";
      return false;
    }
    x.stair_gait = gait == "stair";
    if (e.contains("slow") && e["slow"].is_boolean()) {
      x.slow = e["slow"].get<bool>();
    }
    if (e.contains("evidence") && e["evidence"].is_string()) {
      x.evidence = e["evidence"].get<std::string>();
    }
    if (e.contains("rise") && e["rise"].is_number()) {
      x.rise = e["rise"].get<double>();
    }
    if (e.contains("width") && e["width"].is_number()) {
      x.width = e["width"].get<double>();
    }
    if (e.contains("heading") && e["heading"].is_number()) {
      x.heading = e["heading"].get<double>();
    }
    x.first = e["first"].get<size_t>();
    x.last = e["last"].get<size_t>();
    if (x.last < x.first) {
      error = where + "last is before first";
      return false;
    }
    if (e.contains("turns")) {
      if (!e["turns"].is_array()) {
        error = where + "turns is not an array";
        return false;
      }
      for (const auto & t : e["turns"]) {
        if (!t.is_number_unsigned()) {
          error = where + "a turn is not an unsigned index";
          return false;
        }
        x.turns.push_back(t.get<size_t>());
      }
    }
    if (e.contains("pts")) {
      if (!e["pts"].is_array()) {
        error = where + "pts is not an array";
        return false;
      }
      for (const auto & p : e["pts"]) {
        if (
          !p.is_array() || p.size() != 3 || !p[0].is_number() || !p[1].is_number() ||
          !p[2].is_number()) {
          error = where + "a point is not [x, y, z]";
          return false;
        }
        x.pts.push_back({p[0].get<double>(), p[1].get<double>(), p[2].get<double>()});
      }
    }
    c.crossings.push_back(std::move(x));
  }
  out = std::move(c);
  return true;
}

}  // namespace syncai_util
