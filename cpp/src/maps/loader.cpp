#include "mrmp/maps/loader.hpp"

#include <filesystem>
#include <stdexcept>

#include "mrmp/core/yaml.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "mrmp/maps/pgm.hpp"

namespace mrmp::maps {
namespace fs = std::filesystem;
using core::YamlNode;

namespace {

// Resolve a path referenced inside a yaml file relative to that file's directory.
std::string resolve(const std::string& base_file, const std::string& ref) {
  fs::path p(ref);
  if (p.is_absolute()) return fs::weakly_canonical(p).string();
  fs::path dir = fs::path(base_file).parent_path();
  return fs::weakly_canonical(dir / p).string();
}

}  // namespace

std::unique_ptr<core::MapBase> load_map(const std::string& path) {
  YamlNode root = core::parse_yaml_file(path);
  std::string type = root.at("type").as_string();
  if (type != "occupancy_grid") {
    throw std::runtime_error("load_map: unsupported map type '" + type + "'");
  }

  std::string image = resolve(path, root.at("image").as_string());
  double resolution = root.at("resolution").as_double();
  const YamlNode& origin = root.at("origin");
  double ox = origin.seq.at(0).as_double();
  double oy = origin.seq.at(1).as_double();
  // occupied_thresh is signature-only (mirrors the Python loader); free_thresh
  // alone decides traversability. Defaults mirror the Python loader too.
  double occupied_thresh = root.has("occupied_thresh") ? root.at("occupied_thresh").as_double() : 0.65;
  double free_thresh = root.has("free_thresh") ? root.at("free_thresh").as_double() : 0.196;

  PgmImage img = load_pgm(image);
  return std::make_unique<OccupancyGrid2D>(OccupancyGrid2D::from_image(
      img, resolution, ox, oy, occupied_thresh, free_thresh));
}

Scenario load_scenario(const std::string& path) {
  YamlNode root = core::parse_yaml_file(path);
  // This repo is multi-agent planning only: a scenario without an `agents:` list
  // (a single start/goal pair) belongs to the sibling navigation repo.
  if (!root.has("agents")) {
    throw std::runtime_error("load_scenario: '" + path +
                             "' declares no 'agents:' list — MRMP scenarios require one");
  }
  Scenario sc;
  sc.map_path = resolve(path, root.at("map").as_string());
  for (const YamlNode& a : root.at("agents").seq) {
    const YamlNode& start = a.at("start");
    const YamlNode& goal = a.at("goal");
    sc.agents.push_back(AgentSpec{
        core::Point{start.seq.at(0).as_double(), start.seq.at(1).as_double()},
        core::Point{goal.seq.at(0).as_double(), goal.seq.at(1).as_double()}});
  }
  return sc;
}

}  // namespace mrmp::maps
