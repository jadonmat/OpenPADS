#pragma once

#include <cstddef>
#include <cstdint>

#include "core/Mesh.hpp"

namespace opads {

/// GPU VAO/VBO/EBO wrapper for rendering an opads::Mesh.
class MeshBuffer {
public:
    MeshBuffer() = default;
    ~MeshBuffer();

    MeshBuffer(const MeshBuffer&) = delete;
    MeshBuffer& operator=(const MeshBuffer&) = delete;
    MeshBuffer(MeshBuffer&& other) noexcept;
    MeshBuffer& operator=(MeshBuffer&& other) noexcept;

    void upload(const Mesh& mesh);
    void draw() const noexcept;
    bool isEmpty() const noexcept;

private:
    void release() noexcept;

    std::uint32_t m_vao{0};
    std::uint32_t m_vbo{0};
    std::uint32_t m_ebo{0};
    std::size_t m_indexCount{0};
};

} // namespace opads
