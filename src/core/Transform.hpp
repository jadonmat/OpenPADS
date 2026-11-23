#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace opads {

/// 3D rigid + scale transform in Aerospace Z-up coordinates (X aft, Y starboard, Z up).
class Transform {
public:
    Transform() = default;
    Transform(const glm::vec3& position, const glm::vec3& rotationDeg, const glm::vec3& scale);

    const glm::vec3& getPosition() const noexcept;
    void setPosition(const glm::vec3& position) noexcept;

    const glm::vec3& getRotationDeg() const noexcept;
    void setRotationDeg(const glm::vec3& rotationDeg) noexcept;

    const glm::vec3& getScale() const noexcept;
    void setScale(const glm::vec3& scale) noexcept;

    /// Builds T * Rz * Ry * Rx * S model matrix.
    glm::mat4 getMatrix() const noexcept;

private:
    glm::vec3 m_position{0.0f, 0.0f, 0.0f};
    glm::vec3 m_rotationDeg{0.0f, 0.0f, 0.0f};
    glm::vec3 m_scale{1.0f, 1.0f, 1.0f};
};

} // namespace opads
