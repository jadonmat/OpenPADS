#pragma once

#include <filesystem>

#include "core/Aircraft.hpp"
#include "core/ImportedMesh.hpp"

namespace opads {

/// Imports ASCII/Binary STL and Wavefront OBJ meshes with automatic unit & axis conversion.
class MeshImporter {
public:
    static bool importStl(
        Aircraft& aircraft,
        const std::filesystem::path& path,
        float unitScale = 1.0f,
        UpAxisConversion axisConversion = UpAxisConversion::NoneZUp
    );

    static bool importObj(
        Aircraft& aircraft,
        const std::filesystem::path& path,
        float unitScale = 1.0f,
        UpAxisConversion axisConversion = UpAxisConversion::NoneZUp
    );

    static bool importAuto(
        Aircraft& aircraft,
        const std::filesystem::path& path,
        float unitScale = 1.0f,
        UpAxisConversion axisConversion = UpAxisConversion::NoneZUp
    );
};

} // namespace opads
