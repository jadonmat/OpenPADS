#include "core/Transform.hpp"

#include <glm/gtc/matrix_transform.hpp>

namespace opads {

Transform::Transform(const glm::vec3& position, const glm::vec3& rotationDeg, const glm::vec3& scale)
    : m_position(position)
    , m_rotationDeg(rotationDeg)
    , m_scale(scale) {
}

const glm::vec3& Transform::getPosition() const noexcept {
    return m_position;
}

void Transform::setPosition(const glm::vec3& position) noexcept {
    m_position = position;
}

const glm::vec3& Transform::getRotationDeg() const noexcept {
    return m_rotationDeg;
}

void Transform::setRotationDeg(const glm::vec3& rotationDeg) noexcept {
    m_rotationDeg = rotationDeg;
}

const glm::vec3& Transform::getScale() const noexcept {
    return m_scale;
}

void Transform::setScale(const glm::vec3& scale) noexcept {
    m_scale = scale;
}

glm::mat4 Transform::getMatrix() const noexcept {
    glm::mat4 mat(1.0f);
    mat = glm::translate(mat, m_position);
    mat = glm::rotate(mat, glm::radians(m_rotationDeg.z), glm::vec3(0.0f, 0.0f, 1.0f));
    mat = glm::rotate(mat, glm::radians(m_rotationDeg.y), glm::vec3(0.0f, 1.0f, 0.0f));
    mat = glm::rotate(mat, glm::radians(m_rotationDeg.x), glm::vec3(1.0f, 0.0f, 0.0f));
    mat = glm::scale(mat, m_scale);
    return mat;
}

} // namespace opads
