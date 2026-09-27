#include "mrmp/maps/pgm.hpp"

#include <cctype>
#include <fstream>
#include <stdexcept>

namespace mrmp::maps {
namespace {

// Reads the next whitespace-separated token, skipping '#' comment lines. PGM
// headers allow comments between any tokens.
std::string next_token(std::istream& in) {
  std::string tok;
  char c;
  while (in.get(c)) {
    if (c == '#') {
      while (in.get(c) && c != '\n') {
      }
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!tok.empty()) return tok;
      continue;
    }
    tok += c;
  }
  return tok;
}

}  // namespace

PgmImage load_pgm(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("pgm: cannot open '" + path + "'");

  std::string magic = next_token(in);
  if (magic != "P2" && magic != "P5") throw std::runtime_error("pgm: unsupported magic " + magic);

  PgmImage img;
  img.width = std::stoi(next_token(in));
  img.height = std::stoi(next_token(in));
  img.maxval = std::stoi(next_token(in));
  if (img.width <= 0 || img.height <= 0 || img.maxval <= 0 || img.maxval > 65535) {
    throw std::runtime_error("pgm: invalid header in '" + path + "'");
  }

  const size_t count = static_cast<size_t>(img.width) * static_cast<size_t>(img.height);
  img.pixels.resize(count);

  if (magic == "P2") {
    for (size_t i = 0; i < count; ++i) {
      std::string tok = next_token(in);
      if (tok.empty()) throw std::runtime_error("pgm: truncated ascii data in '" + path + "'");
      img.pixels[i] = std::stoi(tok);
    }
  } else if (img.maxval < 256) {
    // next_token already consumed the single whitespace separating maxval from the
    // raster, so the stream is positioned at the first pixel byte.
    for (size_t i = 0; i < count; ++i) {
      int c = in.get();
      if (c == EOF) throw std::runtime_error("pgm: truncated binary data in '" + path + "'");
      img.pixels[i] = c;
    }
  } else {
    // maxval >= 256 -> two bytes per sample, big-endian (Netpbm spec).
    for (size_t i = 0; i < count; ++i) {
      int hi = in.get();
      int lo = in.get();
      if (hi == EOF || lo == EOF) {
        throw std::runtime_error("pgm: truncated binary data in '" + path + "'");
      }
      img.pixels[i] = (hi << 8) | lo;
    }
  }
  return img;
}

}  // namespace mrmp::maps
