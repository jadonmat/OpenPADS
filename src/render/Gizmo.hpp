#pragma once

#include <cstdint>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "core/Aircraft.hpp"
#include "render/OrbitCamera.hpp"
#include "render/Shader.hpp"

namespace opads {

enum class GizmoMode {
    Select,
    Translate,
    SectionHandle
};

enum class GizmoAxis {
    None,
    X,
    Y,
    Z
};

/// 3D axis manipulation gizmo for component translation and wing cross-section dragging.
class Gizmo {
public:
    Gizmo() = default;
    ~Gizmo();

    Gizmo(const Gizmo&) = delete;
    Gizmo& operator=(const Gizmo&) = delete;

    bool initialize();

    GizmoMode getMode() const noexcept;
    void setMode(GizmoMode mode) noexcept;

    GizmoAxis getHoveredAxis() const noexcept;
    bool isDragging() const noexcept;

    /// Computes active world-space gizmo anchor for the selected component/section.
    bool getOrigin(const Aircraft& aircraft, glm::vec3& originOut) const noexcept;

    void updateHover(const Ray& ray, const Aircraft& aircraft, const OrbitCamera& camera);
    bool beginDrag(const Ray& ray, const Aircraft& aircraft, const OrbitCamera& camera);
    void updateDrag(const Ray& ray, Aircraft& aircraft);
    void endDrag() noexcept;

    void render(const glm::mat4& viewProj, const Aircraft& aircraft, const OrbitCamera& camera);

    static glm::vec3 axisVector(GizmoAxis axis) noexcept;
    static float projectRayOntoAxis(const Ray& ray, const glm::vec3& origin, const glm::vec3& axisDir) noexcept;

private:
    void release() noexcept;
    GizmoAxis hitTest(const Ray& ray, const glm::vec3& origin, float scale) const noexcept;

    Shader m_shader;
    std::uint32_t m_vao{0};
    std::uint32_t m_vbo{0};

    GizmoMode m_mode{GizmoMode::Translate};
    GizmoAxis m_hoveredAxis{GizmoAxis::None};
    GizmoAxis m_activeAxis{GizmoAxis::None};
    bool m_dragging{false};

    glm::vec3 m_dragStartOrigin{0.0f, 0.0f, 0.0f};
    glm::vec3 m_dragStartCompPos{0.0f, 0.0f, 0.0f};
    glm::vec3 m_dragStartLocalTipLE{0.0f, 0.0f, 0.0f};
    float m_dragStartAxisT{0.0f};
};

} // namespace opads
