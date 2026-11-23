#pragma once

#include <cstdint>

#include <glm/vec3.hpp>

#include "core/Aircraft.hpp"
#include "core/Component.hpp"

namespace opads {

struct SnapResult {
    bool snapped{false};
    glm::vec3 snappedPosition{0.0f, 0.0f, 0.0f};
    glm::vec3 sourcePoint{0.0f, 0.0f, 0.0f};
    glm::vec3 targetPoint{0.0f, 0.0f, 0.0f};
};

/// Geometry-based feature and mesh-surface snapping between Aircraft components.
class GeometrySnap {
public:
    static glm::vec3 closestPointOnTriangle(
        const glm::vec3& p,
        const glm::vec3& a,
        const glm::vec3& b,
        const glm::vec3& c
    ) noexcept;

    /// Snaps movingPart at candidatePos so its nearest geometry point contacts another part's surface/features.
    static SnapResult snapPartFree(
        const Component& movingPart,
        const glm::vec3& candidatePos,
        const Aircraft& aircraft,
        float maxSnapDistance
    );

    /// Snaps movingPart along axisDir so its nearest surface point aligns flush with another part's surface.
    static SnapResult snapPartAlongAxis(
        const Component& movingPart,
        const glm::vec3& candidatePos,
        const glm::vec3& axisDir,
        const Aircraft& aircraft,
        float maxSnapDistance
    );

    /// Snaps a single world-space point (e.g. a dragged cross-section handle) to other parts' surfaces/features.
    static SnapResult snapWorldPoint(
        const glm::vec3& worldPoint,
        std::uint64_t excludeComponentId,
        const Aircraft& aircraft,
        float maxSnapDistance
    );
};

} // namespace opads
