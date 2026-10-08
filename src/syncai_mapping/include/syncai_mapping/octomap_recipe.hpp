#ifndef SYNCAI_MAPPING__OCTOMAP_RECIPE_HPP_
#define SYNCAI_MAPPING__OCTOMAP_RECIPE_HPP_

// octomap.recipe.json: the OctoMap build's status, beside its outputs in the
// map directory. It is the build's only status surface -- the backend reads
// it from its own container, and pgo_node may be gone (a mode switch) long
// before the build ends. Modelled on the backend's gridmap.recipe.json:
//
//   {"status":"converting","started_at":"<UTC>","params":{...}}
//   {"status":"ok","started_at":..,"finished_at":..,"params":{...},"measurements":{...}}
//   {"status":"failed","started_at":..,"finished_at":..,"params":{...},"error":"<one line>"}
//
// "converting" is written by pgo_node BEFORE it spawns the build, so a
// reader acting on the save_maps response always finds the file. A build
// that dies hard (OOM, SIGKILL, docker stop) leaves it at "converting";
// started_at is what lets a reader age that into "interrupted". Nothing in it
// names the map or holds an absolute path, like every other file there.
//
// No OctoMap and no PCL here: this and jsonEscape are all pgo_node links.

#include <filesystem>
#include <string>

#include "syncai_mapping/octomap_builder.hpp"

namespace syncai_mapping
{

// JSON string escaping for the hand-formatted documents this package writes
// (the map_cloud_file notice and the sidecar) -- it grows no JSON dependency.
std::string jsonEscape(const std::string & s);

namespace octomap_recipe
{

inline constexpr const char * kSidecarFile = "octomap.recipe.json";

// "2026-10-08T03:12:45Z".
std::string isoUtcNow();

std::string converting(const octomap_builder::Params & params, const std::string & started_at);
std::string ok(
  const octomap_builder::Params & params, const std::string & started_at,
  const std::string & finished_at, const octomap_builder::Measurements & m);
// `error` is flattened to one line.
std::string failed(
  const octomap_builder::Params & params, const std::string & started_at,
  const std::string & finished_at, const std::string & error);

// <dir>/octomap.recipe.json via .tmp + rename, so a reader polling it never
// sees half a document. Throws std::runtime_error (path-free text).
void writeAtomic(const std::filesystem::path & dir, const std::string & json);

}  // namespace octomap_recipe
}  // namespace syncai_mapping

#endif  // SYNCAI_MAPPING__OCTOMAP_RECIPE_HPP_
