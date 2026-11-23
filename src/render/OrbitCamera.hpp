#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace opads {

enum class ViewPreset {
    Isometric,
    Top,
    Bottom,
    Front,
    Back,
    Right,
    Left
};

struct Ray {
    glm::vec3 origin{0.0f, 0.0f, 0.0f};
    glm::vec3 direction{1.0f, 0.0f, 0.0f};
};

/// CAD-style Z-up spherical orbit camera (X aft, Y starboard, Z up).
class OrbitCamera {
public:
    OrbitCamera();

    void orbit(float deltaYawDeg, float deltaPitchDeg) noexcept;
    void pan(float deltaRight, float deltaUp) noexcept;
    void zoom(float scrollSteps) noexcept;

    void setPresetView(ViewPreset preset) noexcept;
    void reset() noexcept;

    glm::vec3 getPosition() const noexcept;
    glm::vec3 getForward() const noexcept;
    glm::vec3 getRight() const noexcept;
    glm::vec3 getUp() const noexcept;

    glm::mat4 getViewMatrix() const noexcept;
    glm::mat4 getProjectionMatrix(float aspectRatio) const noexcept;
    /// Builds a world-space picking ray from normalized viewport coordinates in [-1, 1].
    Ray screenPointToRay(float ndcX, float ndcY, float aspectRatio) const noexcept;

    const glm::vec3& getTarget() const noexcept;
    float getDistance() const noexcept;
    float getYawDeg() const noexcept;
    float getPitchDeg() const noexcept;

private:
    glm::vec3 m_target{0.0f, 0.0f, 0.0f};
    float m_distance{18.0f};
    float m_yawDeg{-135.0f};
    float m_pitchDeg{25.0f};
    float m_fovDeg{45.0f};
    float m_nearPlane{0.05f};
    float m_farPlane{2000.0f};
};

} // namespace opads
