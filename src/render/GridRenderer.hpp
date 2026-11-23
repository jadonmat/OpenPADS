#pragma once

#include <cstdint>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "render/Shader.hpp"

namespace opads {

/// Renders the Aerospace Z=0 reference ground grid and origin RGB axis triad.
class GridRenderer {
public:
    GridRenderer() = default;
    ~GridRenderer();

    GridRenderer(const GridRenderer&) = delete;
    GridRenderer& operator=(const GridRenderer&) = delete;

    bool initialize();
    void render(const glm::mat4& viewProj, const glm::vec3& cameraPos, const glm::vec3& cameraTarget, bool showGrid, bool showAxes) const;

private:
    void release() noexcept;

    Shader m_gridShader;
    Shader m_axisShader;
    std::uint32_t m_gridVao{0};
    std::uint32_t m_gridVbo{0};
    std::uint32_t m_axisVao{0};
    std::uint32_t m_axisVbo{0};
};

} // namespace opads
