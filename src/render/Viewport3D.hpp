#pragma once

#include <cstdint>
#include <unordered_map>

#include "core/Aircraft.hpp"
#include "render/GridRenderer.hpp"
#include "render/MeshBuffer.hpp"
#include "render/OrbitCamera.hpp"
#include "render/Shader.hpp"

namespace opads {

enum class RenderMode {
    Shaded,
    Wireframe,
    ShadedWireframe
};

/// Offscreen OpenGL 3D viewport rendering the reference grid, axes, and aircraft components.
class Viewport3D {
public:
    Viewport3D() = default;
    ~Viewport3D();

    Viewport3D(const Viewport3D&) = delete;
    Viewport3D& operator=(const Viewport3D&) = delete;

    bool initialize();
    void resize(int width, int height);
    void render(Aircraft& aircraft);

    std::uint32_t getColorTextureId() const noexcept;
    int getWidth() const noexcept;
    int getHeight() const noexcept;

    OrbitCamera& getCamera() noexcept;
    const OrbitCamera& getCamera() const noexcept;

    RenderMode getRenderMode() const noexcept;
    void setRenderMode(RenderMode mode) noexcept;

    bool isGridVisible() const noexcept;
    void setGridVisible(bool visible) noexcept;

    bool isAxesVisible() const noexcept;
    void setAxesVisible(bool visible) noexcept;

private:
    void releaseFramebuffer() noexcept;
    void releaseHighlightBuffers() noexcept;
    void syncComponentMeshes(Aircraft& aircraft);
    void renderHighlightLoops(const Component& component, const glm::mat4& viewProj);

    std::uint32_t m_fbo{0};
    std::uint32_t m_colorTexture{0};
    std::uint32_t m_depthRbo{0};
    std::uint32_t m_highlightVao{0};
    std::uint32_t m_highlightVbo{0};
    int m_width{1};
    int m_height{1};

    OrbitCamera m_camera;
    GridRenderer m_gridRenderer;
    Shader m_surfaceShader;
    Shader m_highlightShader;
    std::unordered_map<std::uint64_t, MeshBuffer> m_componentBuffers;

    RenderMode m_renderMode{RenderMode::ShadedWireframe};
    bool m_showGrid{true};
    bool m_showAxes{true};
};

} // namespace opads
