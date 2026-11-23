#pragma once

#include <cstddef>
#include <vector>

#include <glm/vec3.hpp>

#include "core/Mesh.hpp"

namespace opads {

enum class EndCapType {
    None = 0,
    Flat = 1,
    Round = 2,
    Edge = 3,
    Sharp = 4
};

/// Shared procedural mesh builders for lofted rings, end caps, cylinders, and XZ mirroring.
class MeshPrimitives {
public:
    static void appendLoftedRings(
        Mesh& mesh,
        const std::vector<std::vector<glm::vec3>>& rings,
        bool closedSeam,
        bool capStart,
        bool capEnd
    );

    static void appendEndCap(
        Mesh& mesh,
        const std::vector<glm::vec3>& endRing,
        const glm::vec3& outwardAxisUnit,
        EndCapType capType,
        double capLengthRatio = 1.0,
        double capStrength = 0.5,
        bool closedSeam = true,
        bool reverseWinding = false
    );

    static void appendCylinder(
        Mesh& mesh,
        const glm::vec3& p0,
        const glm::vec3& p1,
        float radius0,
        float radius1,
        int radialSegments = 20,
        bool capEnds = true
    );

    static void mirrorSubmeshXZ(Mesh& mesh, std::size_t startVertex, std::size_t startIndex);

    static double computeSurfaceArea(const Mesh& mesh) noexcept;
    static double computeVolume(const Mesh& mesh) noexcept;
};

} // namespace opads
