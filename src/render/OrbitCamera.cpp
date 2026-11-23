#include "render/OrbitCamera.hpp"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace opads {

namespace {
constexpr glm::vec3 kWorldUp{0.0f, 0.0f, 1.0f};
constexpr float kMinPitchDeg = -89.5f;
constexpr float kMaxPitchDeg = 89.5f;
constexpr float kMinDistance = 0.25f;
constexpr float kMaxDistance = 1000.0f;
} // namespace

OrbitCamera::OrbitCamera() = default;

void OrbitCamera::orbit(float deltaYawDeg, float deltaPitchDeg) noexcept {
    m_yawDeg += deltaYawDeg;
    m_pitchDeg = std::clamp(m_pitchDeg + deltaPitchDeg, kMinPitchDeg, kMaxPitchDeg);
}

void OrbitCamera::pan(float deltaRight, float deltaUp) noexcept {
    const float panScale = m_distance * 0.0015f;
    m_target -= getRight() * (deltaRight * panScale);
    m_target += getUp() * (deltaUp * panScale);
}

void OrbitCamera::zoom(float scrollSteps) noexcept {
    const float factor = std::pow(0.88f, scrollSteps);
    m_distance = std::clamp(m_distance * factor, kMinDistance, kMaxDistance);
}

void OrbitCamera::setPresetView(ViewPreset preset) noexcept {
    switch (preset) {
    case ViewPreset::Isometric:
        m_yawDeg = -135.0f;
        m_pitchDeg = 25.0f;
        break;
    case ViewPreset::Top:
        m_yawDeg = -90.0f;
        m_pitchDeg = kMaxPitchDeg;
        break;
    case ViewPreset::Bottom:
        m_yawDeg = -90.0f;
        m_pitchDeg = kMinPitchDeg;
        break;
    case ViewPreset::Front:
        m_yawDeg = 180.0f;
        m_pitchDeg = 0.0f;
        break;
    case ViewPreset::Back:
        m_yawDeg = 0.0f;
        m_pitchDeg = 0.0f;
        break;
    case ViewPreset::Right:
        m_yawDeg = 90.0f;
        m_pitchDeg = 0.0f;
        break;
    case ViewPreset::Left:
        m_yawDeg = -90.0f;
        m_pitchDeg = 0.0f;
        break;
    }
}

void OrbitCamera::reset() noexcept {
    m_target = glm::vec3(0.0f, 0.0f, 0.0f);
    m_distance = 18.0f;
    setPresetView(ViewPreset::Isometric);
}

glm::vec3 OrbitCamera::getPosition() const noexcept {
    const float yawRad = glm::radians(m_yawDeg);
    const float pitchRad = glm::radians(m_pitchDeg);
    const float cosPitch = std::cos(pitchRad);

    return m_target + glm::vec3(
        m_distance * cosPitch * std::cos(yawRad),
        m_distance * cosPitch * std::sin(yawRad),
        m_distance * std::sin(pitchRad)
    );
}

glm::vec3 OrbitCamera::getForward() const noexcept {
    return glm::normalize(m_target - getPosition());
}

glm::vec3 OrbitCamera::getRight() const noexcept {
    return glm::normalize(glm::cross(getForward(), kWorldUp));
}

glm::vec3 OrbitCamera::getUp() const noexcept {
    return glm::normalize(glm::cross(getRight(), getForward()));
}

glm::mat4 OrbitCamera::getViewMatrix() const noexcept {
    return glm::lookAt(getPosition(), m_target, kWorldUp);
}

glm::mat4 OrbitCamera::getProjectionMatrix(float aspectRatio) const noexcept {
    const float safeAspect = (aspectRatio > 0.001f) ? aspectRatio : 1.0f;
    return glm::perspective(glm::radians(m_fovDeg), safeAspect, m_nearPlane, m_farPlane);
}

Ray OrbitCamera::screenPointToRay(float ndcX, float ndcY, float aspectRatio) const noexcept {
    const glm::mat4 invViewProj = glm::inverse(getProjectionMatrix(aspectRatio) * getViewMatrix());
    const glm::vec4 nearClip(ndcX, ndcY, -1.0f, 1.0f);
    const glm::vec4 farClip(ndcX, ndcY, 1.0f, 1.0f);

    glm::vec4 nearWorld = invViewProj * nearClip;
    glm::vec4 farWorld = invViewProj * farClip;
    nearWorld /= nearWorld.w;
    farWorld /= farWorld.w;

    const glm::vec3 origin(nearWorld);
    const glm::vec3 dir = glm::normalize(glm::vec3(farWorld) - origin);
    return Ray{origin, dir};
}

const glm::vec3& OrbitCamera::getTarget() const noexcept {
    return m_target;
}

float OrbitCamera::getDistance() const noexcept {
    return m_distance;
}

float OrbitCamera::getYawDeg() const noexcept {
    return m_yawDeg;
}

float OrbitCamera::getPitchDeg() const noexcept {
    return m_pitchDeg;
}

} // namespace opads
