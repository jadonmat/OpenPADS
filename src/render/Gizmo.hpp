#pragma once

#include <cstdint>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "core/Aircraft.hpp"
#include "render/GeometrySnap.hpp"
#include "render/OrbitCamera.hpp"
#include "render/Shader.hpp"

namespace opads {

enum class GizmoMode {
    Select,
    Translate,
    Rotate,
    Scale
};

enum class GizmoTarget {
    Part,
    CrossSection
};

enum class GizmoAxis {
    None,
    X,
    Y,
    Z,
    Center
};

/// 3D manipulation gizmo supporting Move, Rotate, Scale, Cross-Section editing, and Surface Snapping.
class Gizmo {
public:
    Gizmo() = default;
    ~Gizmo();

    Gizmo(const Gizmo&) = delete;
    Gizmo& operator=(const Gizmo&) = delete;

    bool initialize();

    GizmoMode getMode() const noexcept;
    void setMode(GizmoMode mode) noexcept;

    GizmoTarget getTarget() const noexcept;
    void setTarget(GizmoTarget target) noexcept;

    bool isSurfaceSnapEnabled() const noexcept;
    void setSurfaceSnapEnabled(bool enabled) noexcept;

    GizmoAxis getHoveredAxis() const noexcept;
    bool isDragging() const noexcept;

    /// Computes active world-space gizmo anchor for the selected component or cross-section.
    bool getOrigin(const Aircraft& aircraft, glm::vec3& originOut) const noexcept;

    void updateHover(const Ray& ray, const Aircraft& aircraft, const OrbitCamera& camera);
    bool beginDrag(const Ray& ray, const Aircraft& aircraft, const OrbitCamera& camera);
    void updateDrag(const Ray& ray, Aircraft& aircraft, const OrbitCamera& camera);
    void endDrag() noexcept;

    void render(const glm::mat4& viewProj, const Aircraft& aircraft, const OrbitCamera& camera);

    static glm::vec3 axisVector(GizmoAxis axis) noexcept;
    static float projectRayOntoAxis(const Ray& ray, const glm::vec3& origin, const glm::vec3& axisDir) noexcept;
    static bool intersectRayPlane(
        const Ray& ray,
        const glm::vec3& planeOrigin,
        const glm::vec3& planeNormal,
        glm::vec3& hitOut
    ) noexcept;

private:
    void release() noexcept;
    GizmoAxis hitTest(const Ray& ray, const glm::vec3& origin, float scale) const noexcept;
    float computeRingAngleDeg(const Ray& ray, const glm::vec3& origin, const glm::vec3& axisDir) const noexcept;

    Shader m_shader;
    std::uint32_t m_vao{0};
    std::uint32_t m_vbo{0};

    GizmoMode m_mode{GizmoMode::Translate};
    GizmoTarget m_target{GizmoTarget::Part};
    bool m_surfaceSnap{false};

    GizmoAxis m_hoveredAxis{GizmoAxis::None};
    GizmoAxis m_activeAxis{GizmoAxis::None};
    bool m_dragging{false};

    glm::vec3 m_dragStartOrigin{0.0f, 0.0f, 0.0f};
    glm::vec3 m_dragStartPlaneHit{0.0f, 0.0f, 0.0f};
    glm::vec3 m_dragStartCompPos{0.0f, 0.0f, 0.0f};
    glm::vec3 m_dragStartCompRotDeg{0.0f, 0.0f, 0.0f};
    glm::vec3 m_dragStartCompScale{1.0f, 1.0f, 1.0f};
    glm::vec3 m_dragStartLocalAnchor{0.0f, 0.0f, 0.0f};
    float m_dragStartAxisT{0.0f};
    float m_dragStartAngleDeg{0.0f};
    float m_dragGizmoScale{1.0f};

    float m_dragStartChord{1.0f};
    float m_dragStartThickScale{1.0f};
    float m_dragStartTwistDeg{0.0f};
    float m_dragStartFuseWidth{1.0f};
    float m_dragStartFuseHeight{1.0f};
    SnapResult m_lastSnap;
};

} // namespace opads
