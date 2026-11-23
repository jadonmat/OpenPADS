#include "render/Viewport3D.hpp"

#include <algorithm>
#include <unordered_set>

#include <glad/gl.h>
#include <glm/gtc/matrix_inverse.hpp>

namespace opads {

namespace {

constexpr const char* kSurfaceVertexShader = R"(#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uViewProj;
uniform mat4 uModel;
uniform mat3 uNormalMatrix;

out vec3 vWorldPos;
out vec3 vNormal;

void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vWorldPos = worldPos.xyz;
    vNormal = normalize(uNormalMatrix * aNormal);
    gl_Position = uViewProj * worldPos;
}
)";

constexpr const char* kSurfaceFragmentShader = R"(#version 450 core
in vec3 vWorldPos;
in vec3 vNormal;

uniform vec3 uCameraPos;
uniform vec3 uBaseColor;
uniform float uWireframeMode;

out vec4 FragColor;

void main() {
    if (uWireframeMode > 0.5) {
        FragColor = vec4(uBaseColor, 1.0);
        return;
    }

    vec3 N = normalize(vNormal);
    if (!gl_FrontFacing) {
        N = -N;
    }

    vec3 keyLight = normalize(vec3(0.45, -0.55, 0.70));
    vec3 fillLight = normalize(vec3(-0.60, 0.40, 0.35));
    vec3 V = normalize(uCameraPos - vWorldPos);

    float keyDiff = max(dot(N, keyLight), 0.0);
    float fillDiff = max(dot(N, fillLight), 0.0) * 0.35;
    float hemi = 0.25 + 0.15 * N.z;

    vec3 H = normalize(keyLight + V);
    float spec = pow(max(dot(N, H), 0.0), 36.0) * 0.22;

    vec3 lit = uBaseColor * (hemi + keyDiff * 0.72 + fillDiff) + vec3(spec);
    FragColor = vec4(lit, 1.0);
}
)";

constexpr const char* kHighlightVertexShader = R"(#version 450 core
layout (location = 0) in vec3 aPos;

uniform mat4 uViewProj;
uniform mat4 uModel;

void main() {
    gl_Position = uViewProj * uModel * vec4(aPos, 1.0);
    gl_Position.z -= 0.0005 * gl_Position.w;
}
)";

constexpr const char* kHighlightFragmentShader = R"(#version 450 core
uniform vec3 uColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(uColor, 1.0);
}
)";

} // namespace

Viewport3D::~Viewport3D() {
    releaseHighlightBuffers();
    releaseFramebuffer();
}

void Viewport3D::releaseHighlightBuffers() noexcept {
    if (m_highlightVbo != 0) {
        glDeleteBuffers(1, &m_highlightVbo);
        m_highlightVbo = 0;
    }
    if (m_highlightVao != 0) {
        glDeleteVertexArrays(1, &m_highlightVao);
        m_highlightVao = 0;
    }
}

void Viewport3D::releaseFramebuffer() noexcept {
    if (m_depthRbo != 0) {
        glDeleteRenderbuffers(1, &m_depthRbo);
        m_depthRbo = 0;
    }
    if (m_colorTexture != 0) {
        glDeleteTextures(1, &m_colorTexture);
        m_colorTexture = 0;
    }
    if (m_fbo != 0) {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = 0;
    }
}

bool Viewport3D::initialize() {
    if (!m_gridRenderer.initialize()) {
        return false;
    }
    if (!m_surfaceShader.compileFromSource(kSurfaceVertexShader, kSurfaceFragmentShader)) {
        return false;
    }
    if (!m_highlightShader.compileFromSource(kHighlightVertexShader, kHighlightFragmentShader)) {
        return false;
    }

    glGenVertexArrays(1, &m_highlightVao);
    glGenBuffers(1, &m_highlightVbo);
    glBindVertexArray(m_highlightVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_highlightVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glBindVertexArray(0);

    resize(1280, 720);
    return true;
}

void Viewport3D::resize(int width, int height) {
    const int clampedW = std::max(width, 1);
    const int clampedH = std::max(height, 1);
    if (m_fbo != 0 && clampedW == m_width && clampedH == m_height) {
        return;
    }

    m_width = clampedW;
    m_height = clampedH;
    releaseFramebuffer();

    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    glGenTextures(1, &m_colorTexture);
    glBindTexture(GL_TEXTURE_2D, m_colorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_width, m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colorTexture, 0);

    glGenRenderbuffers(1, &m_depthRbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, m_width, m_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_depthRbo);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Viewport3D::syncComponentMeshes(Aircraft& aircraft) {
    std::unordered_set<std::uint64_t> activeIds;
    for (const auto& component : aircraft.getComponents()) {
        activeIds.insert(component->getId());
        const bool wasDirty = component->isDirty();
        component->ensureBuilt();
        auto& buffer = m_componentBuffers[component->getId()];
        if (wasDirty || buffer.isEmpty()) {
            buffer.upload(component->getMesh());
        }
    }

    for (auto it = m_componentBuffers.begin(); it != m_componentBuffers.end();) {
        if (!activeIds.contains(it->first)) {
            it = m_componentBuffers.erase(it);
        } else {
            ++it;
        }
    }
}

void Viewport3D::render(Aircraft& aircraft) {
    if (m_fbo == 0) {
        return;
    }

    syncComponentMeshes(aircraft);

    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, m_width, m_height);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.11f, 0.13f, 0.16f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const float aspect = static_cast<float>(m_width) / static_cast<float>(m_height);
    const glm::mat4 view = m_camera.getViewMatrix();
    const glm::mat4 proj = m_camera.getProjectionMatrix(aspect);
    const glm::mat4 viewProj = proj * view;
    const glm::vec3 camPos = m_camera.getPosition();

    // Render opaque components before transparent grid so depth occlusion works cleanly.
    const auto selectedId = aircraft.getSelectedComponentId();
    for (const auto& component : aircraft.getComponents()) {
        if (!component->isVisible()) {
            continue;
        }
        const auto it = m_componentBuffers.find(component->getId());
        if (it == m_componentBuffers.end() || it->second.isEmpty()) {
            continue;
        }

        const glm::mat4 model = component->getTransform().getMatrix();
        const glm::mat3 normalMat = glm::inverseTranspose(glm::mat3(model));
        const bool isSelected = selectedId.has_value() && *selectedId == component->getId();
        const glm::vec3 baseColor = isSelected
            ? glm::vec3(0.45f, 0.72f, 0.96f)
            : glm::vec3(0.76f, 0.79f, 0.84f);

        m_surfaceShader.bind();
        m_surfaceShader.setMat4("uViewProj", viewProj);
        m_surfaceShader.setMat4("uModel", model);
        m_surfaceShader.setMat3("uNormalMatrix", normalMat);
        m_surfaceShader.setVec3("uCameraPos", camPos);

        if (m_renderMode == RenderMode::Shaded || m_renderMode == RenderMode::ShadedWireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            m_surfaceShader.setVec3("uBaseColor", baseColor);
            m_surfaceShader.setFloat("uWireframeMode", 0.0f);
            it->second.draw();
        }

        if (m_renderMode == RenderMode::Wireframe || m_renderMode == RenderMode::ShadedWireframe) {
            glEnable(GL_POLYGON_OFFSET_LINE);
            glPolygonOffset(-1.0f, -1.0f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            m_surfaceShader.setVec3("uBaseColor", isSelected ? glm::vec3(0.95f, 0.85f, 0.35f) : glm::vec3(0.14f, 0.16f, 0.20f));
            m_surfaceShader.setFloat("uWireframeMode", 1.0f);
            it->second.draw();
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_POLYGON_OFFSET_LINE);
        }

        if (isSelected) {
            renderHighlightLoops(*component, viewProj);
        }
    }

    m_gridRenderer.render(viewProj, camPos, m_camera.getTarget(), m_showGrid, m_showAxes);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Viewport3D::renderHighlightLoops(const Component& component, const glm::mat4& viewProj) {
    if (m_highlightVao == 0 || m_highlightVbo == 0) {
        return;
    }
    const auto loops = component.getHighlightLoops();
    if (loops.empty()) {
        return;
    }

    m_highlightShader.bind();
    m_highlightShader.setMat4("uViewProj", viewProj);
    m_highlightShader.setMat4("uModel", component.getTransform().getMatrix());
    m_highlightShader.setVec3("uColor", glm::vec3(1.0f, 0.78f, 0.18f));

    glLineWidth(2.8f);
    glBindVertexArray(m_highlightVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_highlightVbo);

    for (const auto& loop : loops) {
        if (loop.size() < 2) {
            continue;
        }
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(loop.size() * sizeof(glm::vec3)),
            loop.data(),
            GL_DYNAMIC_DRAW
        );
        glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(loop.size()));
    }

    glBindVertexArray(0);
    glLineWidth(1.0f);
}

std::uint32_t Viewport3D::getColorTextureId() const noexcept {
    return m_colorTexture;
}

int Viewport3D::getWidth() const noexcept {
    return m_width;
}

int Viewport3D::getHeight() const noexcept {
    return m_height;
}

OrbitCamera& Viewport3D::getCamera() noexcept {
    return m_camera;
}

const OrbitCamera& Viewport3D::getCamera() const noexcept {
    return m_camera;
}

RenderMode Viewport3D::getRenderMode() const noexcept {
    return m_renderMode;
}

void Viewport3D::setRenderMode(RenderMode mode) noexcept {
    m_renderMode = mode;
}

bool Viewport3D::isGridVisible() const noexcept {
    return m_showGrid;
}

void Viewport3D::setGridVisible(bool visible) noexcept {
    m_showGrid = visible;
}

bool Viewport3D::isAxesVisible() const noexcept {
    return m_showAxes;
}

void Viewport3D::setAxesVisible(bool visible) noexcept {
    m_showAxes = visible;
}

} // namespace opads
