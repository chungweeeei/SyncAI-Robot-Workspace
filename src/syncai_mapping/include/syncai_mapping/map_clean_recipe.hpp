#ifndef SYNCAI_MAPPING__MAP_CLEAN_RECIPE_HPP_
#define SYNCAI_MAPPING__MAP_CLEAN_RECIPE_HPP_

// map_clean.recipe.json: the map cleaning's status, beside map.pcd in the map
// directory. It is the clean's only status surface -- the backend reads it
// from its own container, and pgo_node may be gone (a mode switch) long
// before the clean ends. Modelled on the backend's gridmap.recipe.json:
//
//   {"status":"converting","started_at":"<UTC>","params":{...}}
//   {"status":"ok","started_at":..,"finished_at":..,"params":{...},"measurements":{...}}
//   {"status":"failed","started_at":..,"finished_at":..,"params":{...},"error":"<one line>"}
//
// "converting" is written by pgo_node BEFORE it spawns clean_map, so a reader
// acting on the save_maps response always finds the file: map.pcd is still
// the raw save while it says so, and the cleaned one once it says "ok". A
// clean that dies hard (OOM, SIGKILL, docker stop) leaves it at "converting"
// with the raw map.pcd in place; started_at is what lets a reader age that
// into "interrupted". Nothing in it names the map or holds an absolute path,
// like every other file there.
//
// No OctoMap and no PCL here: this is all pgo_node links of the cleaning.

#include <filesystem>
#include <string>

#include "syncai_mapping/map_cleaner.hpp"

namespace syncai_mapping::map_clean_recipe
{

inline constexpr const char * kSidecarFile = "map_clean.recipe.json";

// "2026-10-08T03:12:45Z".
std::string isoUtcNow();

std::string converting(const map_cleaner::Params & params, const std::string & started_at);
std::string ok(
  const map_cleaner::Params & params, const std::string & started_at,
  const std::string & finished_at, const map_cleaner::Measurements & m);
// `error` is flattened to one line.
std::string failed(
  const map_cleaner::Params & params, const std::string & started_at,
  const std::string & finished_at, const std::string & error);

// <dir>/map_clean.recipe.json via .tmp + rename, so a reader polling it never
// sees half a document. Throws std::runtime_error (path-free text).
void writeAtomic(const std::filesystem::path & dir, const std::string & json);

}  // namespace syncai_mapping::map_clean_recipe

#endif  // SYNCAI_MAPPING__MAP_CLEAN_RECIPE_HPP_
