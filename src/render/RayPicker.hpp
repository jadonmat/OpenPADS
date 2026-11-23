#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include <glm/vec3.hpp>

#include "core/Aircraft.hpp"
#include "render/OrbitCamera.hpp"

namespace opads {

struct PickResult {
    bool hit{false};
    std::uint64_t componentId{0};
    float distance{0.0f};
    glm::vec3 worldPos{0.0f, 0.0f, 0.0f};
    glm::vec3 localPos{0.0f, 0.0f, 0.0f};
    std::optional<std::size_t> crossSectionIndex;
};

/// CPU-side Möller-Trumbore ray picker against Aircraft component meshes.
class RayPicker {
public:
    static bool intersectTriangle(
        const Ray& ray,
        const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2,
        float& tOut
    ) noexcept;

    static PickResult pickAircraft(const Ray& worldRay, const Aircraft& aircraft);
};

} // namespace opads
