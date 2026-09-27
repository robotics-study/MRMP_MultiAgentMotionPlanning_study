#pragma once

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "mrmp/maps/occupancy_grid.hpp"

namespace mrmp::test {

// Builds a grid from an ASCII layout: '.' = free, '#' = occupied. rows[0] is the
// top image row. resolution 0.5, origin (0,0) — enough for geometry/search tests.
inline maps::OccupancyGrid2D make_grid(const std::vector<std::string>& rows) {
  int h = static_cast<int>(rows.size());
  int w = static_cast<int>(rows.empty() ? 0 : rows[0].size());
  std::vector<bool> free_cells(static_cast<size_t>(h) * w);
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) free_cells[static_cast<size_t>(r) * w + c] = rows[r][c] == '.';
  }
  return maps::OccupancyGrid2D(h, w, 0.5, 0.0, 0.0, std::move(free_cells));
}

// Writes content to a unique temp file and returns its path (auto-cleaned by the
// OS temp dir). Used to exercise loaders/validators against real file contents.
inline std::string write_temp(const std::string& name, const std::string& content) {
  static std::atomic<int> counter{0};
  std::filesystem::path dir = std::filesystem::temp_directory_path() / "mrmp_tests";
  std::filesystem::create_directories(dir);
  std::filesystem::path p = dir / (std::to_string(counter++) + "_" + name);
  std::ofstream(p) << content;
  return p.string();
}

inline std::string repo_path(const std::string& rel) {
  return std::string(MRMP_REPO_DIR) + "/" + rel;
}

}  // namespace mrmp::test
