#include "render/GeometrySnap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>

#include "render/RayPicker.hpp"

namespace opads {

namespace {

struct WorldAabb {
    glm::vec3 minPt{std::numeric_limits<float>::max()};
    glm::vec3 maxPt{std::numeric_limits<float>::lowest()};
    bool valid{false};
};

WorldAabb computeComponentWorldAabb(const Component& comp, const glm::mat4& model) {
    WorldAabb box;
    const auto& verts = comp.getMesh().getVertices();
    if (verts.empty()) {
        return box;
    }
    box.valid = true;
    for (const Vertex& v : verts) {
        const glm::vec3 wp = glm::vec3(model * glm::vec4(v.position, 1.0f));
        box.minPt = glm::min(box.minPt, wp);
        box.maxPt = glm::max(box.maxPt, wp);
    }
    return box;
}

bool aabbNear(const WorldAabb& a, const WorldAabb& b, float pad) noexcept {
    if (!a.valid || !b.valid) {
        return false;
    }
    return (a.minPt.x - pad <= b.maxPt.x && a.maxPt.x + pad >= b.minPt.x) &&
           (a.minPt.y - pad <= b.maxPt.y && a.maxPt.y + pad >= b.minPt.y) &&
           (a.minPt.z - pad <= b.maxPt.z && a.maxPt.z + pad >= b.minPt.z);
}

glm::mat4 buildTransformAtPosition(const Transform& baseTransform, const glm::vec3& pos) noexcept {
    Transform shifted = baseTransform;
    shifted.setPosition(pos);
    return shifted.getMatrix();
}

} // namespace

glm::vec3 GeometrySnap::closestPointOnTriangle(
    const glm::vec3& p,
    const glm::vec3& a,
    const glm::vec3& b,
    const glm::vec3& c
) noexcept {
    const glm::vec3 ab = b - a;
    const glm::vec3 ac = c - a;
    const glm::vec3 ap = p - a;

    const float d1 = glm::dot(ab, ap);
    const float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    const glm::vec3 bp = p - b;
    const float d3 = glm::dot(ab, bp);
    const float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    const glm::vec3 cp = p - c;
    const float d5 = glm::dot(ab, cp);
    const float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + ab * v + ac * w;
}

SnapResult GeometrySnap::snapPartFree(
    const Component& movingPart,
    const glm::vec3& candidatePos,
    const Aircraft& aircraft,
    float maxSnapDistance
) {
    SnapResult best;
    best.snappedPosition = candidatePos;

    const glm::mat4 movingModel = buildTransformAtPosition(movingPart.getTransform(), candidatePos);
    const WorldAabb movingAabb = computeComponentWorldAabb(movingPart, movingModel);
    if (!movingAabb.valid) {
        return best;
    }

    const auto localSamples = movingPart.getSnapSamplePointsLocal();
    if (localSamples.empty()) {
        return best;
    }

    std::vector<glm::vec3> movingWorldPts;
    movingWorldPts.reserve(localSamples.size());
    for (const glm::vec3& lp : localSamples) {
        movingWorldPts.push_back(glm::vec3(movingModel * glm::vec4(lp, 1.0f)));
    }

    float bestDistSq = maxSnapDistance * maxSnapDistance;

    for (const auto& other : aircraft.getComponents()) {
        if (!other->isVisible() || other->getId() == movingPart.getId()) {
            continue;
        }

        const glm::mat4 otherModel = other->getTransform().getMatrix();
        const WorldAabb otherAabb = computeComponentWorldAabb(*other, otherModel);
        if (!aabbNear(movingAabb, otherAabb, maxSnapDistance)) {
            continue;
        }

        const glm::mat4 invOtherModel = glm::inverse(otherModel);
        const auto& verts = other->getMesh().getVertices();
        const auto& indices = other->getMesh().getIndices();
        const auto otherFeatures = other->getSnapSamplePointsLocal();

        for (const glm::vec3& srcWorld : movingWorldPts) {
            const glm::vec3 srcLocal = glm::vec3(invOtherModel * glm::vec4(srcWorld, 1.0f));

            // 1. Check feature-to-feature vertex/section snap (slight priority for crisp alignment)
            for (const glm::vec3& featLocal : otherFeatures) {
                const glm::vec3 featWorld = glm::vec3(otherModel * glm::vec4(featLocal, 1.0f));
                const glm::vec3 diff = featWorld - srcWorld;
                const float distSq = glm::dot(diff, diff) * 0.85f;
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best.snapped = true;
                    best.sourcePoint = srcWorld;
                    best.targetPoint = featWorld;
                    best.snappedPosition = candidatePos + diff;
                }
            }

            // 2. Check exact mesh-triangle closest surface point
            for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
                const glm::vec3 closestLocal = closestPointOnTriangle(
                    srcLocal,
                    verts[indices[i]].position,
                    verts[indices[i + 1]].position,
                    verts[indices[i + 2]].position
                );
                const glm::vec3 targetWorld = glm::vec3(otherModel * glm::vec4(closestLocal, 1.0f));
                const glm::vec3 diff = targetWorld - srcWorld;
                const float distSq = glm::dot(diff, diff);
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best.snapped = true;
                    best.sourcePoint = srcWorld;
                    best.targetPoint = targetWorld;
                    best.snappedPosition = candidatePos + diff;
                }
            }
        }
    }

    return best;
}

SnapResult GeometrySnap::snapPartAlongAxis(
    const Component& movingPart,
    const glm::vec3& candidatePos,
    const glm::vec3& axisDir,
    const Aircraft& aircraft,
    float maxSnapDistance
) {
    SnapResult best;
    best.snappedPosition = candidatePos;

    const glm::mat4 movingModel = buildTransformAtPosition(movingPart.getTransform(), candidatePos);
    const WorldAabb movingAabb = computeComponentWorldAabb(movingPart, movingModel);
    if (!movingAabb.valid) {
        return best;
    }

    const auto localSamples = movingPart.getSnapSamplePointsLocal();
    if (localSamples.empty()) {
        return best;
    }

    std::vector<glm::vec3> movingWorldPts;
    movingWorldPts.reserve(localSamples.size());
    for (const glm::vec3& lp : localSamples) {
        movingWorldPts.push_back(glm::vec3(movingModel * glm::vec4(lp, 1.0f)));
    }

    float bestAbsDelta = maxSnapDistance;

    for (const auto& other : aircraft.getComponents()) {
        if (!other->isVisible() || other->getId() == movingPart.getId()) {
            continue;
        }

        const glm::mat4 otherModel = other->getTransform().getMatrix();
        const WorldAabb otherAabb = computeComponentWorldAabb(*other, otherModel);
        if (!aabbNear(movingAabb, otherAabb, maxSnapDistance)) {
            continue;
        }

        const glm::mat4 invOtherModel = glm::inverse(otherModel);
        const glm::vec3 localAxisPos = glm::normalize(glm::vec3(invOtherModel * glm::vec4(axisDir, 0.0f)));
        const glm::vec3 localAxisNeg = -localAxisPos;
        const auto& verts = other->getMesh().getVertices();
        const auto& indices = other->getMesh().getIndices();

        for (const glm::vec3& srcWorld : movingWorldPts) {
            const glm::vec3 srcLocal = glm::vec3(invOtherModel * glm::vec4(srcWorld, 1.0f));
            const Ray rayPos{srcLocal, localAxisPos};
            const Ray rayNeg{srcLocal, localAxisNeg};

            for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
                const glm::vec3& v0 = verts[indices[i]].position;
                const glm::vec3& v1 = verts[indices[i + 1]].position;
                const glm::vec3& v2 = verts[indices[i + 2]].position;

                float localT = 0.0f;
                if (RayPicker::intersectTriangle(rayPos, v0, v1, v2, localT)) {
                    const glm::vec3 hitWorld = glm::vec3(otherModel * glm::vec4(srcLocal + localAxisPos * localT, 1.0f));
                    const float worldT = glm::dot(hitWorld - srcWorld, axisDir);
                    if (std::abs(worldT) < bestAbsDelta) {
                        bestAbsDelta = std::abs(worldT);
                        best.snapped = true;
                        best.sourcePoint = srcWorld;
                        best.targetPoint = hitWorld;
                        best.snappedPosition = candidatePos + axisDir * worldT;
                    }
                }
                if (RayPicker::intersectTriangle(rayNeg, v0, v1, v2, localT)) {
                    const glm::vec3 hitWorld = glm::vec3(otherModel * glm::vec4(srcLocal + localAxisNeg * localT, 1.0f));
                    const float worldT = glm::dot(hitWorld - srcWorld, axisDir);
                    if (std::abs(worldT) < bestAbsDelta) {
                        bestAbsDelta = std::abs(worldT);
                        best.snapped = true;
                        best.sourcePoint = srcWorld;
                        best.targetPoint = hitWorld;
                        best.snappedPosition = candidatePos + axisDir * worldT;
                    }
                }
            }
        }
    }

    return best;
}

SnapResult GeometrySnap::snapWorldPoint(
    const glm::vec3& worldPoint,
    std::uint64_t excludeComponentId,
    const Aircraft& aircraft,
    float maxSnapDistance
) {
    SnapResult best;
    best.snappedPosition = worldPoint;
    float bestDistSq = maxSnapDistance * maxSnapDistance;

    for (const auto& other : aircraft.getComponents()) {
        if (!other->isVisible() || other->getId() == excludeComponentId) {
            continue;
        }
        const glm::mat4 otherModel = other->getTransform().getMatrix();
        const WorldAabb otherAabb = computeComponentWorldAabb(*other, otherModel);
        if (!otherAabb.valid ||
            worldPoint.x < otherAabb.minPt.x - maxSnapDistance || worldPoint.x > otherAabb.maxPt.x + maxSnapDistance ||
            worldPoint.y < otherAabb.minPt.y - maxSnapDistance || worldPoint.y > otherAabb.maxPt.y + maxSnapDistance ||
            worldPoint.z < otherAabb.minPt.z - maxSnapDistance || worldPoint.z > otherAabb.maxPt.z + maxSnapDistance) {
            continue;
        }

        const glm::mat4 invOtherModel = glm::inverse(otherModel);
        const glm::vec3 localPt = glm::vec3(invOtherModel * glm::vec4(worldPoint, 1.0f));
        const auto& verts = other->getMesh().getVertices();
        const auto& indices = other->getMesh().getIndices();

        for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
            const glm::vec3 closestLocal = closestPointOnTriangle(
                localPt,
                verts[indices[i]].position,
                verts[indices[i + 1]].position,
                verts[indices[i + 2]].position
            );
            const glm::vec3 targetWorld = glm::vec3(otherModel * glm::vec4(closestLocal, 1.0f));
            const glm::vec3 diff = targetWorld - worldPoint;
            const float distSq = glm::dot(diff, diff);
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                best.snapped = true;
                best.sourcePoint = worldPoint;
                best.targetPoint = targetWorld;
                best.snappedPosition = targetWorld;
            }
        }
    }

    return best;
}

} // namespace opads
