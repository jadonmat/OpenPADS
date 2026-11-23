#pragma once

#include <filesystem>

#include "core/Aircraft.hpp"

namespace opads {

/// Exports visible Aircraft meshes in world space to binary STL or Wavefront OBJ.
class MeshExporter {
public:
    static bool exportBinaryStl(const Aircraft& aircraft, const std::filesystem::path& path);
    static bool exportObj(const Aircraft& aircraft, const std::filesystem::path& path);
};

} // namespace opads
