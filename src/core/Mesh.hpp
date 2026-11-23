#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/vec3.hpp>

namespace opads {

struct Vertex {
    glm::vec3 position{0.0f, 0.0f, 0.0f};
    glm::vec3 normal{0.0f, 0.0f, 1.0f};
};

/// Indexed triangle mesh produced by parametric components.
class Mesh {
public:
    Mesh() = default;

    void clear() noexcept;
    void addVertex(const glm::vec3& position, const glm::vec3& normal);
    void addTriangle(std::uint32_t i0, std::uint32_t i1, std::uint32_t i2);

    const std::vector<Vertex>& getVertices() const noexcept;
    const std::vector<std::uint32_t>& getIndices() const noexcept;

    std::size_t getVertexCount() const noexcept;
    std::size_t getTriangleCount() const noexcept;
    bool isEmpty() const noexcept;

private:
    std::vector<Vertex> m_vertices;
    std::vector<std::uint32_t> m_indices;
};

} // namespace opads
