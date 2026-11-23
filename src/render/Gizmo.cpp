#include "render/Gizmo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <glad/gl.h>
#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

#include "core/Fuselage.hpp"
#include "core/Wing.hpp"
#include "render/RayPicker.hpp"

namespace opads {

namespace {

constexpr const char* kGizmoVertexShader = R"(#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uViewProj;
out vec3 vColor;

void main() {
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

constexpr const char* kGizmoFragmentShader = R"(#version 450 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
)";

float rayToSegmentDistance(
    const Ray& ray,
    const glm::vec3& segA,
    const glm::vec3& segB
) noexcept {
    const glm::vec3 u = segB - segA;
    const glm::vec3 v = ray.direction;
    const glm::vec3 w = segA - ray.origin;

    const float a = glm::dot(u, u);
    const float b = glm::dot(u, v);
    const float c = glm::dot(v, v);
    const float d = glm::dot(u, w);
    const float e = glm::dot(v, w);
    const float denom = a * c - b * b;

    float sN = 0.0f;
    float sD = denom;
    float tN = 0.0f;
    float tD = denom;

    if (denom < 1e-6f) {
        sN = 0.0f;
        sD = 1.0f;
        tN = e;
        tD = c;
    } else {
        sN = b * e - c * d;
        tN = a * e - b * d;
        if (sN < 0.0f) {
            sN = 0.0f;
            tN = e;
            tD = c;
        } else if (sN > sD) {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0f) {
        tN = 0.0f;
        if (-d < 0.0f) {
            sN = 0.0f;
        } else if (-d > a) {
            sN = sD;
        } else {
            sN = -d;
            sD = a;
        }
    }

    const float sc = (std::abs(sN) < 1e-6f) ? 0.0f : (sN / sD);
    const float tc = (std::abs(tN) < 1e-6f) ? 0.0f : (tN / tD);
    const glm::vec3 dP = w + (sc * u) - (tc * v);
    return glm::length(dP);
}

float rayToPointDistance(const Ray& ray, const glm::vec3& pt) noexcept {
    const glm::vec3 toPt = pt - ray.origin;
    const float proj = std::max(0.0f, glm::dot(toPt, ray.direction));
    const glm::vec3 closest = ray.origin + ray.direction * proj;
    return glm::length(pt - closest);
}

void buildRingBasis(GizmoAxis axis, glm::vec3& uOut, glm::vec3& vOut) noexcept {
    if (axis == GizmoAxis::X) {
        uOut = glm::vec3(0.0f, 1.0f, 0.0f);
        vOut = glm::vec3(0.0f, 0.0f, 1.0f);
    } else if (axis == GizmoAxis::Y) {
        uOut = glm::vec3(1.0f, 0.0f, 0.0f);
        vOut = glm::vec3(0.0f, 0.0f, 1.0f);
    } else {
        uOut = glm::vec3(1.0f, 0.0f, 0.0f);
        vOut = glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

void pushLine(std::vector<float>& verts, const glm::vec3& a, const glm::vec3& b, const glm::vec3& col) {
    verts.push_back(a.x); verts.push_back(a.y); verts.push_back(a.z);
    verts.push_back(col.r); verts.push_back(col.g); verts.push_back(col.b);
    verts.push_back(b.x); verts.push_back(b.y); verts.push_back(b.z);
    verts.push_back(col.r); verts.push_back(col.g); verts.push_back(col.b);
}

void pushWireCube(std::vector<float>& verts, const glm::vec3& center, float halfExtent, const glm::vec3& col) {
    const glm::vec3 d000(-halfExtent, -halfExtent, -halfExtent);
    const glm::vec3 d100( halfExtent, -halfExtent, -halfExtent);
    const glm::vec3 d110( halfExtent,  halfExtent, -halfExtent);
    const glm::vec3 d010(-halfExtent,  halfExtent, -halfExtent);
    const glm::vec3 d001(-halfExtent, -halfExtent,  halfExtent);
    const glm::vec3 d101( halfExtent, -halfExtent,  halfExtent);
    const glm::vec3 d111( halfExtent,  halfExtent,  halfExtent);
    const glm::vec3 d011(-halfExtent,  halfExtent,  halfExtent);

    pushLine(verts, center + d000, center + d100, col);
    pushLine(verts, center + d100, center + d110, col);
    pushLine(verts, center + d110, center + d010, col);
    pushLine(verts, center + d010, center + d000, col);

    pushLine(verts, center + d001, center + d101, col);
    pushLine(verts, center + d101, center + d111, col);
    pushLine(verts, center + d111, center + d011, col);
    pushLine(verts, center + d011, center + d001, col);

    pushLine(verts, center + d000, center + d001, col);
    pushLine(verts, center + d100, center + d101, col);
    pushLine(verts, center + d110, center + d111, col);
    pushLine(verts, center + d010, center + d011, col);
}

} // namespace

Gizmo::~Gizmo() {
    release();
}

void Gizmo::release() noexcept {
    if (m_vbo != 0) {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }
    if (m_vao != 0) {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }
}

bool Gizmo::initialize() {
    if (!m_shader.compileFromSource(kGizmoVertexShader, kGizmoFragmentShader)) {
        return false;
    }
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glBindVertexArray(0);
    return true;
}

GizmoMode Gizmo::getMode() const noexcept { return m_mode; }

void Gizmo::setMode(GizmoMode mode) noexcept {
    m_mode = mode;
    if (mode == GizmoMode::Select) {
        endDrag();
        m_hoveredAxis = GizmoAxis::None;
    }
}

GizmoTarget Gizmo::getTarget() const noexcept { return m_target; }

void Gizmo::setTarget(GizmoTarget target) noexcept {
    if (m_target != target) {
        endDrag();
        m_target = target;
        m_hoveredAxis = GizmoAxis::None;
    }
}

bool Gizmo::isSurfaceSnapEnabled() const noexcept { return m_surfaceSnap; }

void Gizmo::setSurfaceSnapEnabled(bool enabled) noexcept { m_surfaceSnap = enabled; }

GizmoAxis Gizmo::getHoveredAxis() const noexcept { return m_hoveredAxis; }
bool Gizmo::isDragging() const noexcept { return m_dragging; }

glm::vec3 Gizmo::axisVector(GizmoAxis axis) noexcept {
    switch (axis) {
    case GizmoAxis::X: return glm::vec3(1.0f, 0.0f, 0.0f);
    case GizmoAxis::Y: return glm::vec3(0.0f, 1.0f, 0.0f);
    case GizmoAxis::Z: return glm::vec3(0.0f, 0.0f, 1.0f);
    case GizmoAxis::Center:
    case GizmoAxis::None:
        break;
    }
    return glm::vec3(0.0f);
}

float Gizmo::projectRayOntoAxis(const Ray& ray, const glm::vec3& origin, const glm::vec3& axisDir) noexcept {
    const glm::vec3 w0 = origin - ray.origin;
    const float b = glm::dot(axisDir, ray.direction);
    const float d = glm::dot(axisDir, w0);
    const float e = glm::dot(ray.direction, w0);
    const float denom = 1.0f - b * b;
    if (denom < 1e-4f) {
        return 0.0f;
    }
    return (b * e - d) / denom;
}

bool Gizmo::intersectRayPlane(
    const Ray& ray,
    const glm::vec3& planeOrigin,
    const glm::vec3& planeNormal,
    glm::vec3& hitOut
) noexcept {
    const float denom = glm::dot(ray.direction, planeNormal);
    if (std::abs(denom) < 1e-5f) {
        return false;
    }
    const float t = glm::dot(planeOrigin - ray.origin, planeNormal) / denom;
    if (t < 0.0f) {
        return false;
    }
    hitOut = ray.origin + ray.direction * t;
    return true;
}

bool Gizmo::getOrigin(const Aircraft& aircraft, glm::vec3& originOut) const noexcept {
    const auto selectedId = aircraft.getSelectedComponentId();
    if (!selectedId.has_value() || m_mode == GizmoMode::Select) {
        return false;
    }
    const Component* comp = aircraft.findComponent(*selectedId);
    if (comp == nullptr || !comp->isVisible()) {
        return false;
    }

    if (m_target == GizmoTarget::CrossSection) {
        if (const auto* wing = dynamic_cast<const Wing*>(comp)) {
            const std::size_t xsec = wing->getSelectedCrossSection();
            const glm::vec3 localLE = wing->getCrossSectionLeadingEdge(xsec);
            originOut = glm::vec3(wing->getTransform().getMatrix() * glm::vec4(localLE, 1.0f));
            return true;
        }
        if (const auto* fuse = dynamic_cast<const Fuselage*>(comp)) {
            if (fuse->getSectionCount() > 0) {
                const glm::vec3 localCenter = fuse->getSectionCenterLocal(fuse->getSelectedSection());
                originOut = glm::vec3(fuse->getTransform().getMatrix() * glm::vec4(localCenter, 1.0f));
                return true;
            }
        }
    }

    originOut = comp->getTransform().getPosition();
    return true;
}

GizmoAxis Gizmo::hitTest(const Ray& ray, const glm::vec3& origin, float scale) const noexcept {
    const float threshold = scale * 0.16f;

    if (m_mode == GizmoMode::Translate || m_mode == GizmoMode::Scale) {
        if (rayToPointDistance(ray, origin) < scale * 0.16f) {
            return GizmoAxis::Center;
        }

        float bestDist = threshold;
        GizmoAxis bestAxis = GizmoAxis::None;
        const std::array<GizmoAxis, 3> axes = {GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z};
        for (GizmoAxis axis : axes) {
            const glm::vec3 start = origin + axisVector(axis) * (scale * 0.14f);
            const glm::vec3 end = origin + axisVector(axis) * scale;
            const float dist = rayToSegmentDistance(ray, start, end);
            if (dist < bestDist) {
                bestDist = dist;
                bestAxis = axis;
            }
        }
        return bestAxis;
    }

    if (m_mode == GizmoMode::Rotate) {
        constexpr int kSegments = 48;
        const float ringRadius = scale * 0.85f;
        float bestDist = threshold;
        GizmoAxis bestAxis = GizmoAxis::None;

        const std::array<GizmoAxis, 3> axes = {GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z};
        for (GizmoAxis axis : axes) {
            glm::vec3 u(0.0f);
            glm::vec3 v(0.0f);
            buildRingBasis(axis, u, v);
            for (int i = 0; i < kSegments; ++i) {
                const float a0 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(kSegments);
                const float a1 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i + 1) / static_cast<float>(kSegments);
                const glm::vec3 p0 = origin + (u * std::cos(a0) + v * std::sin(a0)) * ringRadius;
                const glm::vec3 p1 = origin + (u * std::cos(a1) + v * std::sin(a1)) * ringRadius;
                const float dist = rayToSegmentDistance(ray, p0, p1);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestAxis = axis;
                }
            }
        }
        return bestAxis;
    }

    return GizmoAxis::None;
}

float Gizmo::computeRingAngleDeg(const Ray& ray, const glm::vec3& origin, const glm::vec3& axisDir) const noexcept {
    glm::vec3 hit(0.0f);
    if (!intersectRayPlane(ray, origin, axisDir, hit)) {
        return m_dragStartAngleDeg;
    }
    const glm::vec3 dir = hit - origin;
    if (glm::length(dir) < 1e-5f) {
        return m_dragStartAngleDeg;
    }

    glm::vec3 u(0.0f);
    glm::vec3 v(0.0f);
    if (std::abs(axisDir.x) > 0.5f) {
        buildRingBasis(GizmoAxis::X, u, v);
    } else if (std::abs(axisDir.y) > 0.5f) {
        buildRingBasis(GizmoAxis::Y, u, v);
    } else {
        buildRingBasis(GizmoAxis::Z, u, v);
    }
    return glm::degrees(std::atan2(glm::dot(dir, v), glm::dot(dir, u)));
}

void Gizmo::updateHover(const Ray& ray, const Aircraft& aircraft, const OrbitCamera& camera) {
    if (m_dragging) {
        return;
    }
    glm::vec3 origin(0.0f);
    if (!getOrigin(aircraft, origin)) {
        m_hoveredAxis = GizmoAxis::None;
        return;
    }
    const float scale = std::max(0.4f, glm::length(origin - camera.getPosition()) * 0.16f);
    m_hoveredAxis = hitTest(ray, origin, scale);
}

bool Gizmo::beginDrag(const Ray& ray, const Aircraft& aircraft, const OrbitCamera& camera) {
    glm::vec3 origin(0.0f);
    if (!getOrigin(aircraft, origin)) {
        return false;
    }
    const float scale = std::max(0.4f, glm::length(origin - camera.getPosition()) * 0.16f);
    const GizmoAxis hit = hitTest(ray, origin, scale);
    if (hit == GizmoAxis::None) {
        return false;
    }

    const auto selectedId = aircraft.getSelectedComponentId();
    const Component* comp = selectedId.has_value() ? aircraft.findComponent(*selectedId) : nullptr;
    if (comp == nullptr) {
        return false;
    }

    m_dragging = true;
    m_activeAxis = hit;
    m_hoveredAxis = hit;
    m_dragStartOrigin = origin;
    m_dragGizmoScale = scale;
    m_dragStartCompPos = comp->getTransform().getPosition();
    m_dragStartCompRotDeg = comp->getTransform().getRotationDeg();
    m_dragStartCompScale = comp->getTransform().getScale();

    if (!intersectRayPlane(ray, origin, camera.getForward(), m_dragStartPlaneHit)) {
        m_dragStartPlaneHit = origin;
    }

    if (hit != GizmoAxis::Center) {
        const glm::vec3 axisDir = axisVector(hit);
        m_dragStartAxisT = projectRayOntoAxis(ray, origin, axisDir);
        m_dragStartAngleDeg = computeRingAngleDeg(ray, origin, axisDir);
    }

    if (const auto* wing = dynamic_cast<const Wing*>(comp)) {
        const std::size_t xsec = wing->getSelectedCrossSection();
        m_dragStartLocalAnchor = wing->getCrossSectionLeadingEdge(xsec);
        m_dragStartChord = wing->getCrossSectionChord(xsec);
        m_dragStartTwistDeg = wing->getCrossSectionTwistDeg(xsec);
        if (xsec == 0 && wing->getSectionCount() > 0) {
            m_dragStartThickScale = wing->getSection(0).getRootAirfoil().getThicknessScale();
        } else if (xsec > 0 && xsec <= wing->getSectionCount()) {
            m_dragStartThickScale = wing->getSection(xsec - 1).getTipAirfoil().getThicknessScale();
        }
    } else if (const auto* fuse = dynamic_cast<const Fuselage*>(comp)) {
        const std::size_t xsec = fuse->getSelectedSection();
        m_dragStartLocalAnchor = fuse->getSectionCenterLocal(xsec);
        if (xsec < fuse->getSectionCount()) {
            m_dragStartFuseWidth = fuse->getSection(xsec).width().getFloat();
            m_dragStartFuseHeight = fuse->getSection(xsec).height().getFloat();
        }
    }

    return true;
}

void Gizmo::updateDrag(const Ray& ray, Aircraft& aircraft, const OrbitCamera& camera) {
    if (!m_dragging || m_activeAxis == GizmoAxis::None) {
        return;
    }
    const auto selectedId = aircraft.getSelectedComponentId();
    Component* comp = selectedId.has_value() ? aircraft.findComponent(*selectedId) : nullptr;
    if (comp == nullptr) {
        endDrag();
        return;
    }

    if (m_mode == GizmoMode::Translate) {
        m_lastSnap = SnapResult{};
        const float snapThreshold = std::max(0.25f, m_dragGizmoScale * 0.45f);

        if (m_activeAxis == GizmoAxis::Center) {
            glm::vec3 planeHit(0.0f);
            if (intersectRayPlane(ray, m_dragStartOrigin, camera.getForward(), planeHit)) {
                const glm::vec3 worldDelta = planeHit - m_dragStartPlaneHit;
                if (m_target == GizmoTarget::Part) {
                    const glm::vec3 candidatePos = m_dragStartCompPos + worldDelta;
                    if (m_surfaceSnap) {
                        m_lastSnap = GeometrySnap::snapPartFree(*comp, candidatePos, aircraft, snapThreshold);
                        comp->getTransform().setPosition(m_lastSnap.snappedPosition);
                    } else {
                        comp->getTransform().setPosition(candidatePos);
                    }
                } else if (auto* wing = dynamic_cast<Wing*>(comp)) {
                    const std::size_t xsec = wing->getSelectedCrossSection();
                    if (xsec == 0) {
                        const glm::vec3 candidatePos = m_dragStartCompPos + worldDelta;
                        if (m_surfaceSnap) {
                            m_lastSnap = GeometrySnap::snapPartFree(*comp, candidatePos, aircraft, snapThreshold);
                            comp->getTransform().setPosition(m_lastSnap.snappedPosition);
                        } else {
                            comp->getTransform().setPosition(candidatePos);
                        }
                    } else {
                        glm::vec3 targetWorld = m_dragStartOrigin + worldDelta;
                        if (m_surfaceSnap) {
                            m_lastSnap = GeometrySnap::snapWorldPoint(targetWorld, comp->getId(), aircraft, snapThreshold);
                            targetWorld = m_lastSnap.snappedPosition;
                        }
                        const glm::mat4 invModel = glm::inverse(wing->getTransform().getMatrix());
                        const glm::vec3 localTarget = glm::vec3(invModel * glm::vec4(targetWorld, 1.0f));
                        wing->moveCrossSectionLeadingEdge(xsec, localTarget);
                    }
                } else if (auto* fuse = dynamic_cast<Fuselage*>(comp)) {
                    glm::vec3 targetWorld = m_dragStartOrigin + worldDelta;
                    if (m_surfaceSnap) {
                        m_lastSnap = GeometrySnap::snapWorldPoint(targetWorld, comp->getId(), aircraft, snapThreshold);
                        targetWorld = m_lastSnap.snappedPosition;
                    }
                    const glm::mat4 invModel = glm::inverse(fuse->getTransform().getMatrix());
                    const glm::vec3 localTarget = glm::vec3(invModel * glm::vec4(targetWorld, 1.0f));
                    fuse->setSectionCenterLocal(fuse->getSelectedSection(), localTarget);
                }
            }
            return;
        }

        const glm::vec3 axisDir = axisVector(m_activeAxis);
        const float deltaT = projectRayOntoAxis(ray, m_dragStartOrigin, axisDir) - m_dragStartAxisT;
        const glm::vec3 worldDelta = axisDir * deltaT;

        if (m_target == GizmoTarget::Part) {
            const glm::vec3 candidatePos = m_dragStartCompPos + worldDelta;
            if (m_surfaceSnap) {
                m_lastSnap = GeometrySnap::snapPartAlongAxis(*comp, candidatePos, axisDir, aircraft, snapThreshold);
                comp->getTransform().setPosition(m_lastSnap.snappedPosition);
            } else {
                comp->getTransform().setPosition(candidatePos);
            }
        } else if (auto* wing = dynamic_cast<Wing*>(comp)) {
            const std::size_t xsec = wing->getSelectedCrossSection();
            if (xsec == 0) {
                const glm::vec3 candidatePos = m_dragStartCompPos + worldDelta;
                if (m_surfaceSnap) {
                    m_lastSnap = GeometrySnap::snapPartAlongAxis(*comp, candidatePos, axisDir, aircraft, snapThreshold);
                    comp->getTransform().setPosition(m_lastSnap.snappedPosition);
                } else {
                    comp->getTransform().setPosition(candidatePos);
                }
            } else {
                glm::vec3 targetWorld = m_dragStartOrigin + worldDelta;
                if (m_surfaceSnap) {
                    m_lastSnap = GeometrySnap::snapWorldPoint(targetWorld, comp->getId(), aircraft, snapThreshold);
                    targetWorld = m_lastSnap.snappedPosition;
                }
                const glm::mat4 invModel = glm::inverse(wing->getTransform().getMatrix());
                const glm::vec3 localTarget = glm::vec3(invModel * glm::vec4(targetWorld, 1.0f));
                wing->moveCrossSectionLeadingEdge(xsec, localTarget);
            }
        } else if (auto* fuse = dynamic_cast<Fuselage*>(comp)) {
            glm::vec3 targetWorld = m_dragStartOrigin + worldDelta;
            if (m_surfaceSnap) {
                m_lastSnap = GeometrySnap::snapWorldPoint(targetWorld, comp->getId(), aircraft, snapThreshold);
                targetWorld = m_lastSnap.snappedPosition;
            }
            const glm::mat4 invModel = glm::inverse(fuse->getTransform().getMatrix());
            const glm::vec3 localTarget = glm::vec3(invModel * glm::vec4(targetWorld, 1.0f));
            fuse->setSectionCenterLocal(fuse->getSelectedSection(), localTarget);
        }
        return;
    }

    if (m_mode == GizmoMode::Rotate && m_activeAxis != GizmoAxis::Center) {
        const glm::vec3 axisDir = axisVector(m_activeAxis);
        const float angleDeg = computeRingAngleDeg(ray, m_dragStartOrigin, axisDir);
        float deltaAngle = angleDeg - m_dragStartAngleDeg;
        if (deltaAngle > 180.0f) deltaAngle -= 360.0f;
        if (deltaAngle < -180.0f) deltaAngle += 360.0f;

        if (m_target == GizmoTarget::Part) {
            comp->getTransform().setRotationDeg(m_dragStartCompRotDeg + axisDir * deltaAngle);
        } else if (auto* wing = dynamic_cast<Wing*>(comp)) {
            if (m_activeAxis == GizmoAxis::Y) {
                wing->setCrossSectionTwistDeg(wing->getSelectedCrossSection(), m_dragStartTwistDeg + deltaAngle);
            }
        }
        return;
    }

    if (m_mode == GizmoMode::Scale) {
        float scaleFactor = 1.0f;
        if (m_activeAxis == GizmoAxis::Center) {
            glm::vec3 planeHit(0.0f);
            if (intersectRayPlane(ray, m_dragStartOrigin, camera.getForward(), planeHit)) {
                const glm::vec3 delta = planeHit - m_dragStartPlaneHit;
                const float projected = glm::dot(delta, camera.getRight() + camera.getUp());
                scaleFactor = std::max(0.05f, 1.0f + projected / std::max(m_dragGizmoScale, 0.1f));
            }
        } else {
            const glm::vec3 axisDir = axisVector(m_activeAxis);
            const float deltaT = projectRayOntoAxis(ray, m_dragStartOrigin, axisDir) - m_dragStartAxisT;
            scaleFactor = std::max(0.05f, 1.0f + deltaT / std::max(m_dragGizmoScale, 0.1f));
        }

        if (m_target == GizmoTarget::Part) {
            glm::vec3 newScale = m_dragStartCompScale;
            if (m_activeAxis == GizmoAxis::Center) {
                newScale *= scaleFactor;
            } else if (m_activeAxis == GizmoAxis::X) {
                newScale.x *= scaleFactor;
            } else if (m_activeAxis == GizmoAxis::Y) {
                newScale.y *= scaleFactor;
            } else if (m_activeAxis == GizmoAxis::Z) {
                newScale.z *= scaleFactor;
            }
            comp->getTransform().setScale(glm::max(newScale, glm::vec3(0.01f)));
        } else if (auto* wing = dynamic_cast<Wing*>(comp)) {
            const std::size_t xsec = wing->getSelectedCrossSection();
            if (m_activeAxis == GizmoAxis::Z) {
                if (Airfoil* foil = wing->getCrossSectionAirfoil(xsec)) {
                    if (foil->setThicknessScale(m_dragStartThickScale * scaleFactor)) {
                        wing->syncSectionsFromCrossSection(xsec);
                    }
                }
            } else {
                wing->setCrossSectionChord(xsec, std::max(0.05f, m_dragStartChord * scaleFactor));
            }
        } else if (auto* fuse = dynamic_cast<Fuselage*>(comp)) {
            const std::size_t xsec = fuse->getSelectedSection();
            if (xsec < fuse->getSectionCount()) {
                FuselageSection& sec = fuse->getSection(xsec);
                if (m_activeAxis == GizmoAxis::Y) {
                    sec.width().setFloat(std::max(0.02f, m_dragStartFuseWidth * scaleFactor));
                } else if (m_activeAxis == GizmoAxis::Z) {
                    sec.height().setFloat(std::max(0.02f, m_dragStartFuseHeight * scaleFactor));
                } else {
                    sec.width().setFloat(std::max(0.02f, m_dragStartFuseWidth * scaleFactor));
                    sec.height().setFloat(std::max(0.02f, m_dragStartFuseHeight * scaleFactor));
                }
                fuse->markDirty();
            }
        }
    }
}

void Gizmo::endDrag() noexcept {
    m_dragging = false;
    m_activeAxis = GizmoAxis::None;
    m_lastSnap = SnapResult{};
}

void Gizmo::render(const glm::mat4& viewProj, const Aircraft& aircraft, const OrbitCamera& camera) {
    if (m_vao == 0 || m_vbo == 0) {
        return;
    }
    glm::vec3 origin(0.0f);
    if (!getOrigin(aircraft, origin)) {
        return;
    }

    const float scale = std::max(0.4f, glm::length(origin - camera.getPosition()) * 0.16f);
    const GizmoAxis highlighted = m_dragging ? m_activeAxis : m_hoveredAxis;

    std::vector<float> verts;
    const auto axisColor = [&](GizmoAxis axis, const glm::vec3& base) {
        return (highlighted == axis) ? glm::vec3(1.0f, 0.92f, 0.28f) : base;
    };

    if (m_mode == GizmoMode::Translate || m_mode == GizmoMode::Scale) {
        const glm::vec3 centerCol = (highlighted == GizmoAxis::Center)
            ? glm::vec3(1.0f, 0.92f, 0.28f)
            : (m_surfaceSnap && m_mode == GizmoMode::Translate ? glm::vec3(0.30f, 0.95f, 0.95f) : glm::vec3(0.92f, 0.92f, 0.95f));
        pushWireCube(verts, origin, scale * 0.075f, centerCol);

        const auto drawAxis = [&](GizmoAxis axis, const glm::vec3& baseCol) {
            const glm::vec3 dir = axisVector(axis);
            const glm::vec3 col = axisColor(axis, baseCol);
            const glm::vec3 tip = origin + dir * scale;
            pushLine(verts, origin + dir * (scale * 0.10f), tip, col);

            if (m_mode == GizmoMode::Translate) {
                const glm::vec3 ortho1 = (axis == GizmoAxis::Z) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 0.0f, 1.0f);
                const glm::vec3 ortho2 = glm::cross(dir, ortho1);
                const glm::vec3 headBase = origin + dir * (scale * 0.84f);
                const float headR = scale * 0.06f;
                pushLine(verts, tip, headBase + ortho1 * headR, col);
                pushLine(verts, tip, headBase - ortho1 * headR, col);
                pushLine(verts, tip, headBase + ortho2 * headR, col);
                pushLine(verts, tip, headBase - ortho2 * headR, col);
            } else {
                pushWireCube(verts, tip, scale * 0.055f, col);
            }
        };

        drawAxis(GizmoAxis::X, glm::vec3(0.95f, 0.28f, 0.28f));
        drawAxis(GizmoAxis::Y, glm::vec3(0.28f, 0.88f, 0.36f));
        drawAxis(GizmoAxis::Z, glm::vec3(0.32f, 0.62f, 1.00f));
    } else if (m_mode == GizmoMode::Rotate) {
        constexpr int kSegments = 48;
        const float ringRadius = scale * 0.85f;
        const auto drawRing = [&](GizmoAxis axis, const glm::vec3& baseCol) {
            const glm::vec3 col = axisColor(axis, baseCol);
            glm::vec3 u(0.0f);
            glm::vec3 v(0.0f);
            buildRingBasis(axis, u, v);
            for (int i = 0; i < kSegments; ++i) {
                const float a0 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(kSegments);
                const float a1 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i + 1) / static_cast<float>(kSegments);
                const glm::vec3 p0 = origin + (u * std::cos(a0) + v * std::sin(a0)) * ringRadius;
                const glm::vec3 p1 = origin + (u * std::cos(a1) + v * std::sin(a1)) * ringRadius;
                pushLine(verts, p0, p1, col);
            }
        };

        drawRing(GizmoAxis::X, glm::vec3(0.95f, 0.28f, 0.28f));
        drawRing(GizmoAxis::Y, glm::vec3(0.28f, 0.88f, 0.36f));
        drawRing(GizmoAxis::Z, glm::vec3(0.32f, 0.62f, 1.00f));
    }

    if (m_dragging && m_lastSnap.snapped) {
        const glm::vec3 snapCol(0.20f, 0.96f, 0.92f);
        pushWireCube(verts, m_lastSnap.targetPoint, scale * 0.06f, snapCol);
        pushLine(verts, m_lastSnap.sourcePoint, m_lastSnap.targetPoint, snapCol);
    }

    if (verts.empty()) {
        return;
    }

    glDisable(GL_DEPTH_TEST);
    glLineWidth(2.6f);
    m_shader.bind();
    m_shader.setMat4("uViewProj", viewProj);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(float)), verts.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(verts.size() / 6));
    glBindVertexArray(0);

    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
}

} // namespace opads
