#include "render/Gizmo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <glad/gl.h>
#include <glm/geometric.hpp>

#include "core/Wing.hpp"

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

GizmoMode Gizmo::getMode() const noexcept {
    return m_mode;
}

void Gizmo::setMode(GizmoMode mode) noexcept {
    m_mode = mode;
    if (mode == GizmoMode::Select) {
        endDrag();
        m_hoveredAxis = GizmoAxis::None;
    }
}

GizmoAxis Gizmo::getHoveredAxis() const noexcept {
    return m_hoveredAxis;
}

bool Gizmo::isDragging() const noexcept {
    return m_dragging;
}

glm::vec3 Gizmo::axisVector(GizmoAxis axis) noexcept {
    switch (axis) {
    case GizmoAxis::X: return glm::vec3(1.0f, 0.0f, 0.0f);
    case GizmoAxis::Y: return glm::vec3(0.0f, 1.0f, 0.0f);
    case GizmoAxis::Z: return glm::vec3(0.0f, 0.0f, 1.0f);
    case GizmoAxis::None: break;
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

bool Gizmo::getOrigin(const Aircraft& aircraft, glm::vec3& originOut) const noexcept {
    const auto selectedId = aircraft.getSelectedComponentId();
    if (!selectedId.has_value() || m_mode == GizmoMode::Select) {
        return false;
    }
    const Component* comp = aircraft.findComponent(*selectedId);
    if (comp == nullptr || !comp->isVisible()) {
        return false;
    }

    if (m_mode == GizmoMode::SectionHandle) {
        if (const auto* wing = dynamic_cast<const Wing*>(comp)) {
            const std::size_t xsec = std::max<std::size_t>(1, wing->getSelectedCrossSection());
            if (xsec <= wing->getSectionCount()) {
                const glm::vec3 localLE = wing->getCrossSectionLeadingEdge(xsec);
                originOut = glm::vec3(wing->getTransform().getMatrix() * glm::vec4(localLE, 1.0f));
                return true;
            }
        }
    }

    originOut = comp->getTransform().getPosition();
    return true;
}

GizmoAxis Gizmo::hitTest(const Ray& ray, const glm::vec3& origin, float scale) const noexcept {
    const float threshold = scale * 0.16f;
    float bestDist = threshold;
    GizmoAxis bestAxis = GizmoAxis::None;

    const std::array<GizmoAxis, 3> axes = {GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z};
    for (GizmoAxis axis : axes) {
        const glm::vec3 end = origin + axisVector(axis) * scale;
        const float dist = rayToSegmentDistance(ray, origin + axisVector(axis) * (scale * 0.12f), end);
        if (dist < bestDist) {
            bestDist = dist;
            bestAxis = axis;
        }
    }
    return bestAxis;
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
    m_dragStartCompPos = comp->getTransform().getPosition();
    m_dragStartAxisT = projectRayOntoAxis(ray, origin, axisVector(hit));

    if (const auto* wing = dynamic_cast<const Wing*>(comp)) {
        const std::size_t xsec = std::max<std::size_t>(1, wing->getSelectedCrossSection());
        m_dragStartLocalTipLE = wing->getCrossSectionLeadingEdge(xsec);
    }
    return true;
}

void Gizmo::updateDrag(const Ray& ray, Aircraft& aircraft) {
    if (!m_dragging || m_activeAxis == GizmoAxis::None) {
        return;
    }
    const auto selectedId = aircraft.getSelectedComponentId();
    Component* comp = selectedId.has_value() ? aircraft.findComponent(*selectedId) : nullptr;
    if (comp == nullptr) {
        endDrag();
        return;
    }

    const glm::vec3 axisDir = axisVector(m_activeAxis);
    const float currentT = projectRayOntoAxis(ray, m_dragStartOrigin, axisDir);
    const float deltaT = currentT - m_dragStartAxisT;

    if (m_mode == GizmoMode::Translate) {
        comp->getTransform().setPosition(m_dragStartCompPos + axisDir * deltaT);
    } else if (m_mode == GizmoMode::SectionHandle) {
        if (auto* wing = dynamic_cast<Wing*>(comp)) {
            const std::size_t xsec = std::max<std::size_t>(1, wing->getSelectedCrossSection());
            if (xsec <= wing->getSectionCount()) {
                const glm::mat4 invModel = glm::inverse(wing->getTransform().getMatrix());
                const glm::vec3 localDelta = glm::vec3(invModel * glm::vec4(axisDir * deltaT, 0.0f));
                wing->moveCrossSectionLeadingEdge(xsec, m_dragStartLocalTipLE + localDelta);
            }
        }
    }
}

void Gizmo::endDrag() noexcept {
    m_dragging = false;
    m_activeAxis = GizmoAxis::None;
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
    verts.reserve(6 * 6 * 3);

    const auto pushLine = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& col) {
        verts.push_back(a.x); verts.push_back(a.y); verts.push_back(a.z);
        verts.push_back(col.r); verts.push_back(col.g); verts.push_back(col.b);
        verts.push_back(b.x); verts.push_back(b.y); verts.push_back(b.z);
        verts.push_back(col.r); verts.push_back(col.g); verts.push_back(col.b);
    };

    const auto drawAxis = [&](GizmoAxis axis, const glm::vec3& baseColor) {
        const glm::vec3 dir = axisVector(axis);
        const glm::vec3 col = (highlighted == axis) ? glm::vec3(1.0f, 0.90f, 0.25f) : baseColor;
        const glm::vec3 tip = origin + dir * scale;
        pushLine(origin, tip, col);

        // Simple 3D arrowhead cross ticks
        const glm::vec3 ortho1 = (axis == GizmoAxis::Z) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 ortho2 = glm::cross(dir, ortho1);
        const glm::vec3 headBase = origin + dir * (scale * 0.84f);
        const float headR = scale * 0.06f;
        pushLine(tip, headBase + ortho1 * headR, col);
        pushLine(tip, headBase - ortho1 * headR, col);
        pushLine(tip, headBase + ortho2 * headR, col);
        pushLine(tip, headBase - ortho2 * headR, col);
    };

    drawAxis(GizmoAxis::X, glm::vec3(0.95f, 0.28f, 0.28f));
    drawAxis(GizmoAxis::Y, glm::vec3(0.28f, 0.88f, 0.36f));
    drawAxis(GizmoAxis::Z, glm::vec3(0.32f, 0.62f, 1.00f));

    glDisable(GL_DEPTH_TEST);
    glLineWidth(3.0f);
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
