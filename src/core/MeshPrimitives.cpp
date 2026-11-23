#include "core/MeshPrimitives.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <glm/geometric.hpp>

namespace opads {

namespace {

void appendCapFan(
    Mesh& mesh,
    const std::vector<glm::vec3>& ring,
    const glm::vec3& normal,
    bool reverseWinding
) {
    if (ring.size() < 4) {
        return;
    }
    const std::size_t uniqueCount = ring.size() - 1;
    glm::vec3 center(0.0f);
    for (std::size_t i = 0; i < uniqueCount; ++i) {
        center += ring[i];
    }
    center /= static_cast<float>(uniqueCount);

    float maxRadiusSq = 0.0f;
    for (std::size_t i = 0; i < uniqueCount; ++i) {
        const glm::vec3 d = ring[i] - center;
        maxRadiusSq = std::max(maxRadiusSq, glm::dot(d, d));
    }
    if (maxRadiusSq < 1e-8f) {
        return;
    }

    const auto centerIdx = static_cast<std::uint32_t>(mesh.getVertexCount());
    mesh.addVertex(center, normal);
    for (const glm::vec3& pt : ring) {
        mesh.addVertex(pt, normal);
    }

    for (std::size_t i = 0; i < uniqueCount; ++i) {
        const auto i0 = centerIdx + 1 + static_cast<std::uint32_t>(i);
        const auto i1 = centerIdx + 1 + static_cast<std::uint32_t>(i + 1);
        if (!reverseWinding) {
            mesh.addTriangle(centerIdx, i0, i1);
        } else {
            mesh.addTriangle(centerIdx, i1, i0);
        }
    }
}

} // namespace

void MeshPrimitives::appendLoftedRings(
    Mesh& mesh,
    const std::vector<std::vector<glm::vec3>>& rings,
    bool closedSeam,
    bool capStart,
    bool capEnd
) {
    if (rings.size() < 2) {
        return;
    }
    const std::size_t ringCount = rings.size();
    const std::size_t ptCount = rings.front().size();
    if (ptCount < 3) {
        return;
    }

    std::vector<glm::vec3> normals(ringCount * ptCount, glm::vec3(0.0f));
    const auto idxAt = [ptCount](std::size_t r, std::size_t p) {
        return r * ptCount + p;
    };

    for (std::size_t r = 0; r + 1 < ringCount; ++r) {
        for (std::size_t p = 0; p + 1 < ptCount; ++p) {
            const glm::vec3& v00 = rings[r][p];
            const glm::vec3& v10 = rings[r + 1][p];
            const glm::vec3& v11 = rings[r + 1][p + 1];
            const glm::vec3& v01 = rings[r][p + 1];

            const glm::vec3 n1 = glm::cross(v01 - v00, v10 - v00);
            const glm::vec3 n2 = glm::cross(v01 - v10, v11 - v10);

            normals[idxAt(r, p)] += n1 + n2;
            normals[idxAt(r + 1, p)] += n1 + n2;
            normals[idxAt(r + 1, p + 1)] += n1 + n2;
            normals[idxAt(r, p + 1)] += n1 + n2;
        }
    }

    if (closedSeam) {
        for (std::size_t r = 0; r < ringCount; ++r) {
            const glm::vec3 seamSum = normals[idxAt(r, 0)] + normals[idxAt(r, ptCount - 1)];
            normals[idxAt(r, 0)] = seamSum;
            normals[idxAt(r, ptCount - 1)] = seamSum;
        }
    }

    const auto base = static_cast<std::uint32_t>(mesh.getVertexCount());
    for (std::size_t r = 0; r < ringCount; ++r) {
        for (std::size_t p = 0; p < ptCount; ++p) {
            glm::vec3 n = normals[idxAt(r, p)];
            const float len = glm::length(n);
            n = (len > 1e-7f) ? (n / len) : glm::vec3(0.0f, 1.0f, 0.0f);
            mesh.addVertex(rings[r][p], n);
        }
    }

    for (std::size_t r = 0; r + 1 < ringCount; ++r) {
        for (std::size_t p = 0; p + 1 < ptCount; ++p) {
            const auto i00 = base + static_cast<std::uint32_t>(idxAt(r, p));
            const auto i10 = base + static_cast<std::uint32_t>(idxAt(r + 1, p));
            const auto i11 = base + static_cast<std::uint32_t>(idxAt(r + 1, p + 1));
            const auto i01 = base + static_cast<std::uint32_t>(idxAt(r, p + 1));

            mesh.addTriangle(i00, i01, i10);
            mesh.addTriangle(i10, i01, i11);
        }
    }

    if (capStart) {
        glm::vec3 axisDir(1.0f, 0.0f, 0.0f);
        if (ringCount >= 2) {
            const glm::vec3 d = rings[1][0] - rings[0][0];
            if (glm::length(d) > 1e-6f) {
                axisDir = glm::normalize(d);
            }
        }
        appendCapFan(mesh, rings.front(), -axisDir, true);
    }

    if (capEnd) {
        glm::vec3 axisDir(1.0f, 0.0f, 0.0f);
        if (ringCount >= 2) {
            const glm::vec3 d = rings[ringCount - 1][0] - rings[ringCount - 2][0];
            if (glm::length(d) > 1e-6f) {
                axisDir = glm::normalize(d);
            }
        }
        appendCapFan(mesh, rings.back(), axisDir, false);
    }
}

void MeshPrimitives::appendEndCap(
    Mesh& mesh,
    const std::vector<glm::vec3>& endRing,
    const glm::vec3& outwardAxisUnit,
    EndCapType capType,
    double capLengthRatio,
    double capStrength,
    bool closedSeam,
    bool reverseWinding
) {
    if (capType == EndCapType::None || endRing.size() < 4) {
        return;
    }

    const float axisLen = glm::length(outwardAxisUnit);
    const glm::vec3 axis = (axisLen > 1e-6f) ? (outwardAxisUnit / axisLen) : glm::vec3(1.0f, 0.0f, 0.0f);

    if (capType == EndCapType::Flat) {
        appendCapFan(mesh, endRing, axis, reverseWinding);
        return;
    }

    const std::size_t uniqueCount = endRing.size() - 1;
    glm::vec3 center(0.0f);
    for (std::size_t i = 0; i < uniqueCount; ++i) {
        center += endRing[i];
    }
    center /= static_cast<float>(uniqueCount);

    float avgRadius = 0.0f;
    for (std::size_t i = 0; i < uniqueCount; ++i) {
        avgRadius += glm::length(endRing[i] - center);
    }
    avgRadius /= static_cast<float>(uniqueCount);
    if (avgRadius < 1e-5f) {
        return;
    }

    const float lengthScale = static_cast<float>(std::clamp(capLengthRatio, 0.05, 8.0));
    const float protrude = lengthScale * avgRadius;
    const float str = static_cast<float>(std::clamp(capStrength, 0.1, 2.0));
    const float axialExp = std::clamp(1.0f / std::max(str, 0.25f), 0.55f, 1.8f);

    const int steps = (capType == EndCapType::Sharp) ? 3 : 6;
    std::vector<std::vector<glm::vec3>> capRings;
    capRings.reserve(static_cast<std::size_t>(steps) + 1);

    for (int s = 0; s <= steps; ++s) {
        const float u = static_cast<float>(s) / static_cast<float>(steps);
        const float axialFrac = (capType == EndCapType::Sharp)
            ? u
            : std::pow(std::sin(0.5f * std::numbers::pi_v<float> * u), axialExp);
        const float radScale = (capType == EndCapType::Sharp)
            ? (1.0f - u)
            : std::cos(0.5f * std::numbers::pi_v<float> * u);

        std::vector<glm::vec3> ring;
        ring.reserve(endRing.size());
        for (const glm::vec3& pt : endRing) {
            const glm::vec3 radialVec = pt - center;
            if (capType == EndCapType::Edge) {
                // Collapse thinner planar component while preserving wider edge.
                const float yScale = radScale;
                const glm::vec3 edgePt = center + glm::vec3(radialVec.x * yScale, radialVec.y * yScale, radialVec.z * (0.25f + 0.75f * yScale));
                ring.push_back(edgePt + axis * (protrude * axialFrac));
            } else {
                ring.push_back(center + radialVec * radScale + axis * (protrude * axialFrac));
            }
        }
        capRings.push_back(std::move(ring));
    }

    if (reverseWinding) {
        std::reverse(capRings.begin(), capRings.end());
    }
    appendLoftedRings(mesh, capRings, closedSeam, false, false);
}

void MeshPrimitives::appendCylinder(
    Mesh& mesh,
    const glm::vec3& p0,
    const glm::vec3& p1,
    float radius0,
    float radius1,
    int radialSegments,
    bool capEnds
) {
    const glm::vec3 axis = p1 - p0;
    const float len = glm::length(axis);
    if (len < 1e-6f) {
        return;
    }
    const glm::vec3 dir = axis / len;
    const glm::vec3 refUp = (std::abs(dir.z) < 0.9f) ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 u = glm::normalize(glm::cross(refUp, dir));
    const glm::vec3 v = glm::cross(dir, u);

    const int segs = std::max(radialSegments, 8);
    std::vector<std::vector<glm::vec3>> rings(2);
    rings[0].reserve(static_cast<std::size_t>(segs) + 1);
    rings[1].reserve(static_cast<std::size_t>(segs) + 1);

    for (int i = 0; i <= segs; ++i) {
        const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(segs);
        const glm::vec3 radial = u * std::cos(angle) + v * std::sin(angle);
        rings[0].push_back(p0 + radial * radius0);
        rings[1].push_back(p1 + radial * radius1);
    }

    appendLoftedRings(mesh, rings, true, capEnds, capEnds);
}

void MeshPrimitives::mirrorSubmeshXZ(Mesh& mesh, std::size_t startVertex, std::size_t startIndex) {
    const std::size_t endVertex = mesh.getVertexCount();
    const std::size_t endIndex = mesh.getIndices().size();

    for (std::size_t i = startVertex; i < endVertex; ++i) {
        const Vertex& v = mesh.getVertices()[i];
        mesh.addVertex(
            glm::vec3(v.position.x, -v.position.y, v.position.z),
            glm::vec3(v.normal.x, -v.normal.y, v.normal.z)
        );
    }

    const auto offset = static_cast<std::uint32_t>(endVertex - startVertex);
    for (std::size_t i = startIndex; i + 2 < endIndex; i += 3) {
        const auto i0 = mesh.getIndices()[i];
        const auto i1 = mesh.getIndices()[i + 1];
        const auto i2 = mesh.getIndices()[i + 2];
        mesh.addTriangle(offset + i0, offset + i2, offset + i1);
    }
}

double MeshPrimitives::computeSurfaceArea(const Mesh& mesh) noexcept {
    const auto& verts = mesh.getVertices();
    const auto& indices = mesh.getIndices();
    double area = 0.0;
    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        const glm::vec3& a = verts[indices[i]].position;
        const glm::vec3& b = verts[indices[i + 1]].position;
        const glm::vec3& c = verts[indices[i + 2]].position;
        area += 0.5 * static_cast<double>(glm::length(glm::cross(b - a, c - a)));
    }
    return area;
}

double MeshPrimitives::computeVolume(const Mesh& mesh) noexcept {
    const auto& verts = mesh.getVertices();
    const auto& indices = mesh.getIndices();
    double vol6 = 0.0;
    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        const glm::vec3& a = verts[indices[i]].position;
        const glm::vec3& b = verts[indices[i + 1]].position;
        const glm::vec3& c = verts[indices[i + 2]].position;
        vol6 += static_cast<double>(glm::dot(a, glm::cross(b, c)));
    }
    return std::abs(vol6) / 6.0;
}

} // namespace opads
