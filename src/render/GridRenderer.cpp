#include "render/GridRenderer.hpp"

#include <array>

#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>

namespace opads {

namespace {

constexpr const char* kGridVertexShader = R"(#version 450 core
layout (location = 0) in vec2 aPlanePos;

uniform mat4 uViewProj;
uniform vec2 uCenterXY;
uniform float uExtent;

out vec3 vWorldPos;

void main() {
    vec2 worldXY = uCenterXY + aPlanePos * uExtent;
    vWorldPos = vec3(worldXY, 0.0);
    gl_Position = uViewProj * vec4(vWorldPos, 1.0);
}
)";

constexpr const char* kGridFragmentShader = R"(#version 450 core
in vec3 vWorldPos;

uniform vec3 uCameraPos;
uniform float uFadeRadius;

out vec4 FragColor;

float gridLine(vec2 coord, float spacing, float lineWidth) {
    vec2 scaled = coord / spacing;
    vec2 deriv = fwidth(scaled);
    vec2 grid = abs(fract(scaled - 0.5) - 0.5) / max(deriv, vec2(1e-5));
    float d = min(grid.x, grid.y);
    return 1.0 - clamp(d / lineWidth, 0.0, 1.0);
}

void main() {
    vec2 xy = vWorldPos.xy;
    float minorMask = gridLine(xy, 1.0, 1.15);
    float majorMask = gridLine(xy, 5.0, 1.35);

    vec2 deriv = max(fwidth(xy), vec2(1e-5));
    float xAxisMask = 1.0 - clamp(abs(xy.y) / (deriv.y * 1.6), 0.0, 1.0);
    float yAxisMask = 1.0 - clamp(abs(xy.x) / (deriv.x * 1.6), 0.0, 1.0);

    vec3 color = vec3(0.24, 0.27, 0.32);
    float alpha = minorMask * 0.24;

    if (majorMask > 0.01) {
        color = mix(color, vec3(0.38, 0.43, 0.50), majorMask);
        alpha = max(alpha, majorMask * 0.45);
    }

    if (xAxisMask > 0.01) {
        color = mix(color, vec3(0.85, 0.26, 0.26), xAxisMask);
        alpha = max(alpha, xAxisMask * 0.80);
    }
    if (yAxisMask > 0.01) {
        color = mix(color, vec3(0.28, 0.82, 0.34), yAxisMask);
        alpha = max(alpha, yAxisMask * 0.80);
    }

    float dist = length(vWorldPos.xy - uCameraPos.xy);
    float fade = 1.0 - smoothstep(uFadeRadius * 0.35, uFadeRadius, dist);
    alpha *= fade;

    if (alpha < 0.005) {
        discard;
    }
    FragColor = vec4(color, alpha);
}
)";

constexpr const char* kAxisVertexShader = R"(#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uViewProj;

out vec3 vColor;

void main() {
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

constexpr const char* kAxisFragmentShader = R"(#version 450 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
)";

} // namespace

GridRenderer::~GridRenderer() {
    release();
}

void GridRenderer::release() noexcept {
    if (m_axisVbo != 0) {
        glDeleteBuffers(1, &m_axisVbo);
        m_axisVbo = 0;
    }
    if (m_axisVao != 0) {
        glDeleteVertexArrays(1, &m_axisVao);
        m_axisVao = 0;
    }
    if (m_gridVbo != 0) {
        glDeleteBuffers(1, &m_gridVbo);
        m_gridVbo = 0;
    }
    if (m_gridVao != 0) {
        glDeleteVertexArrays(1, &m_gridVao);
        m_gridVao = 0;
    }
}

bool GridRenderer::initialize() {
    if (!m_gridShader.compileFromSource(kGridVertexShader, kGridFragmentShader)) {
        return false;
    }
    if (!m_axisShader.compileFromSource(kAxisVertexShader, kAxisFragmentShader)) {
        return false;
    }

    constexpr std::array<float, 12> kQuadVerts = {
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };

    glGenVertexArrays(1, &m_gridVao);
    glGenBuffers(1, &m_gridVbo);
    glBindVertexArray(m_gridVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kQuadVerts), kQuadVerts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);

    constexpr float kAxisLength = 2.0f;
    constexpr std::array<float, 36> kAxisVerts = {
        // X axis (Red - Aft)
        0.0f, 0.0f, 0.0f,        0.92f, 0.25f, 0.25f,
        kAxisLength, 0.0f, 0.0f, 0.92f, 0.25f, 0.25f,
        // Y axis (Green - Starboard)
        0.0f, 0.0f, 0.0f,        0.25f, 0.85f, 0.35f,
        0.0f, kAxisLength, 0.0f, 0.25f, 0.85f, 0.35f,
        // Z axis (Blue - Up)
        0.0f, 0.0f, 0.0f,        0.28f, 0.56f, 0.96f,
        0.0f, 0.0f, kAxisLength, 0.28f, 0.56f, 0.96f
    };

    glGenVertexArrays(1, &m_axisVao);
    glGenBuffers(1, &m_axisVbo);
    glBindVertexArray(m_axisVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_axisVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kAxisVerts), kAxisVerts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);
    return true;
}

void GridRenderer::render(
    const glm::mat4& viewProj,
    const glm::vec3& cameraPos,
    const glm::vec3& cameraTarget,
    bool showGrid,
    bool showAxes
) const {
    if (showGrid && m_gridVao != 0) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);

        m_gridShader.bind();
        m_gridShader.setMat4("uViewProj", viewProj);
        m_gridShader.setVec3("uCameraPos", cameraPos);
        m_gridShader.setVec2("uCenterXY", glm::vec2(cameraTarget.x, cameraTarget.y));
        m_gridShader.setFloat("uExtent", 250.0f);
        m_gridShader.setFloat("uFadeRadius", 140.0f);

        glBindVertexArray(m_gridVao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    if (showAxes && m_axisVao != 0) {
        glLineWidth(2.5f);
        m_axisShader.bind();
        m_axisShader.setMat4("uViewProj", viewProj);
        glBindVertexArray(m_axisVao);
        glDrawArrays(GL_LINES, 0, 6);
        glBindVertexArray(0);
        glLineWidth(1.0f);
    }
}

} // namespace opads
