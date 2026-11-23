#include "core/Mesh.hpp"

namespace opads {

void Mesh::clear() noexcept {
    m_vertices.clear();
    m_indices.clear();
}

void Mesh::addVertex(const glm::vec3& position, const glm::vec3& normal) {
    m_vertices.push_back(Vertex{position, normal});
}

void Mesh::addTriangle(std::uint32_t i0, std::uint32_t i1, std::uint32_t i2) {
    m_indices.push_back(i0);
    m_indices.push_back(i1);
    m_indices.push_back(i2);
}

const std::vector<Vertex>& Mesh::getVertices() const noexcept {
    return m_vertices;
}

const std::vector<std::uint32_t>& Mesh::getIndices() const noexcept {
    return m_indices;
}

std::size_t Mesh::getVertexCount() const noexcept {
    return m_vertices.size();
}

std::size_t Mesh::getTriangleCount() const noexcept {
    return m_indices.size() / 3;
}

bool Mesh::isEmpty() const noexcept {
    return m_vertices.empty() || m_indices.empty();
}

} // namespace opads
