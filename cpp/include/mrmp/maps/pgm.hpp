#pragma once

#include <string>
#include <vector>

namespace mrmp::maps {

// Grayscale image loaded from a PGM file, row-major with row 0 = top.
struct PgmImage {
  int width = 0;
  int height = 0;
  int maxval = 0;
  std::vector<int> pixels;  // size width*height, values in [0, maxval]
};

// Reads PGM P2 (ASCII) and P5 (binary). Throws std::runtime_error on malformed
// input. PNG is out of scope (sample maps are PGM).
PgmImage load_pgm(const std::string& path);

}  // namespace mrmp::maps
