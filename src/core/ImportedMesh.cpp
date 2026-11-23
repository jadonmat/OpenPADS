#include "core/ImportedMesh.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include <glm/geometric.hpp>

#include "core/MeshPrimitives.hpp"

namespace opads {

ImportedMesh::ImportedMesh(
    std::uint64_t id,
    std::string name,
    std::vector<glm::vec3> rawPositions,
    std::vector<std::uint32_t> rawIndices,
    std::string sourcePath
)
    : Component(id, std::move(name))
    , m_rawPositions(std::move(rawPositions))
    , m_rawIndices(std::move(rawIndices))
    , m_sourcePath(std::move(sourcePath)) {
    ensureBuilt();
}

std::string ImportedMesh::getTypeName() const {
    return "ImportedMesh";
}

const std::string& ImportedMesh::getSourcePath() const noexcept {
    return m_sourcePath;
}

float ImportedMesh::getUnitScale() const noexcept {
    return m_unitScale;
}

void ImportedMesh::setUnitScale(float scale) {
    const float clamped = std::clamp(scale, 1e-5f, 1000.0f);
    if (std::abs(clamped - m_unitScale) > 1e-7f) {
        m_unitScale = clamped;
        markDirty();
    }
}

UpAxisConversion ImportedMesh::getAxisConversion() const noexcept {
    return m_axisConversion;
}

void ImportedMesh::setAxisConversion(UpAxisConversion mode) {
    if (m_axisConversion != mode) {
        m_axisConversion = mode;
        markDirty();
    }
}

bool ImportedMesh::isCenterOrigin() const noexcept {
    return m_centerOrigin;
}

void ImportedMesh::setCenterOrigin(bool center) {
    if (m_centerOrigin != center) {
        m_centerOrigin = center;
        markDirty();
    }
}

const std::vector<glm::vec3>& ImportedMesh::getRawPositions() const noexcept {
    return m_rawPositions;
}

const std::vector<std::uint32_t>& ImportedMesh::getRawIndices() const noexcept {
    return m_rawIndices;
}

double ImportedMesh::getWettedArea() const noexcept {
    return m_wettedArea;
}

void ImportedMesh::rebuildMesh() {
    m_wettedArea = 0.0;
    if (m_rawPositions.empty() || m_rawIndices.size() < 3) {
        return;
    }

    std::vector<glm::vec3> transformed(m_rawPositions.size());
    glm::vec3 minPt(std::numeric_limits<float>::max());
    glm::vec3 maxPt(std::numeric_limits<float>::lowest());

    for (std::size_t i = 0; i < m_rawPositions.size(); ++i) {
        glm::vec3 p = m_rawPositions[i] * m_unitScale;
        if (m_axisConversion == UpAxisConversion::YUpToZUp) {
            // Convert Y-up (X right, Y up, Z back) -> Aerospace Z-up
            p = glm::vec3(p.z, p.x, p.y);
        }
        transformed[i] = p;
        minPt = glm::min(minPt, p);
        maxPt = glm::max(maxPt, p);
    }

    if (m_centerOrigin) {
        const glm::vec3 center = 0.5f * (minPt + maxPt);
        for (glm::vec3& p : transformed) {
            p -= center;
        }
    }

    std::vector<glm::vec3> normals(transformed.size(), glm::vec3(0.0f));
    for (std::size_t i = 0; i + 2 < m_rawIndices.size(); i += 3) {
        const std::uint32_t i0 = m_rawIndices[i];
        const std::uint32_t i1 = m_rawIndices[i + 1];
        const std::uint32_t i2 = m_rawIndices[i + 2];
        if (i0 >= transformed.size() || i1 >= transformed.size() || i2 >= transformed.size()) {
            continue;
        }
        const glm::vec3 n = glm::cross(transformed[i1] - transformed[i0], transformed[i2] - transformed[i0]);
        normals[i0] += n;
        normals[i1] += n;
        normals[i2] += n;
    }

    for (std::size_t i = 0; i < transformed.size(); ++i) {
        const float len = glm::length(normals[i]);
        const glm::vec3 n = (len > 1e-7f) ? (normals[i] / len) : glm::vec3(0.0f, 0.0f, 1.0f);
        m_mesh.addVertex(transformed[i], n);
    }

    for (std::size_t i = 0; i + 2 < m_rawIndices.size(); i += 3) {
        const std::uint32_t i0 = m_rawIndices[i];
        const std::uint32_t i1 = m_rawIndices[i + 1];
        const std::uint32_t i2 = m_rawIndices[i + 2];
        if (i0 < transformed.size() && i1 < transformed.size() && i2 < transformed.size()) {
            m_mesh.addTriangle(i0, i1, i2);
        }
    }

    m_wettedArea = MeshPrimitives::computeSurfaceArea(m_mesh);
}

} // namespace opads
