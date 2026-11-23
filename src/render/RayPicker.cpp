#include "render/RayPicker.hpp"

#include <cmath>
#include <limits>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>

#include "core/Wing.hpp"

namespace opads {

bool RayPicker::intersectTriangle(
    const Ray& ray,
    const glm::vec3& v0,
    const glm::vec3& v1,
    const glm::vec3& v2,
    float& tOut
) noexcept {
    constexpr float kEpsilon = 1e-6f;
    const glm::vec3 edge1 = v1 - v0;
    const glm::vec3 edge2 = v2 - v0;
    const glm::vec3 pvec = glm::cross(ray.direction, edge2);
    const float det = glm::dot(edge1, pvec);
    if (std::abs(det) < kEpsilon) {
        return false;
    }

    const float invDet = 1.0f / det;
    const glm::vec3 tvec = ray.origin - v0;
    const float u = glm::dot(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    const glm::vec3 qvec = glm::cross(tvec, edge1);
    const float v = glm::dot(ray.direction, qvec) * invDet;
    if (v < 0.0f || (u + v) > 1.0f) {
        return false;
    }

    const float t = glm::dot(edge2, qvec) * invDet;
    if (t <= kEpsilon) {
        return false;
    }

    tOut = t;
    return true;
}

PickResult RayPicker::pickAircraft(const Ray& worldRay, const Aircraft& aircraft) {
    PickResult best;
    best.distance = std::numeric_limits<float>::max();

    for (const auto& component : aircraft.getComponents()) {
        if (!component->isVisible()) {
            continue;
        }
        const Mesh& mesh = component->getMesh();
        if (mesh.isEmpty()) {
            continue;
        }

        const glm::mat4 model = component->getTransform().getMatrix();
        const glm::mat4 invModel = glm::inverse(model);
        const glm::vec3 localOrigin = glm::vec3(invModel * glm::vec4(worldRay.origin, 1.0f));
        const glm::vec3 localDir = glm::vec3(invModel * glm::vec4(worldRay.direction, 0.0f));
        const float localDirLen = glm::length(localDir);
        if (localDirLen <= 1e-7f) {
            continue;
        }
        const Ray localRay{localOrigin, localDir / localDirLen};

        const auto& verts = mesh.getVertices();
        const auto& indices = mesh.getIndices();

        for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
            float localT = 0.0f;
            if (!intersectTriangle(
                    localRay,
                    verts[indices[i]].position,
                    verts[indices[i + 1]].position,
                    verts[indices[i + 2]].position,
                    localT
                )) {
                continue;
            }

            const glm::vec3 localHit = localRay.origin + localRay.direction * localT;
            const glm::vec3 worldHit = glm::vec3(model * glm::vec4(localHit, 1.0f));
            const float worldDist = glm::length(worldHit - worldRay.origin);
            if (worldDist < best.distance) {
                best.hit = true;
                best.componentId = component->getId();
                best.distance = worldDist;
                best.worldPos = worldHit;
                best.localPos = localHit;
                best.crossSectionIndex.reset();

                if (const auto* wing = dynamic_cast<const Wing*>(component.get())) {
                    // Match absolute Y so clicking mirrored -Y side still selects the matching XSec.
                    const glm::vec3 symPos(localHit.x, std::abs(localHit.y), localHit.z);
                    float bestSecDist = std::numeric_limits<float>::max();
                    std::size_t bestIdx = 0;
                    for (std::size_t s = 0; s < wing->getCrossSectionCount(); ++s) {
                        const glm::vec3 le = wing->getCrossSectionLeadingEdge(s);
                        const float dy = symPos.y - le.y;
                        const float dz = symPos.z - le.z;
                        const float distSq = dy * dy + dz * dz;
                        if (distSq < bestSecDist) {
                            bestSecDist = distSq;
                            bestIdx = s;
                        }
                    }
                    best.crossSectionIndex = bestIdx;
                }
            }
        }
    }

    return best;
}

} // namespace opads
