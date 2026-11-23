#include "render/Shader.hpp"

#include <iostream>
#include <string>
#include <vector>

#include <glad/gl.h>
#include <glm/gtc/type_ptr.hpp>

namespace opads {

Shader::~Shader() {
    release();
}

Shader::Shader(Shader&& other) noexcept
    : m_programId(other.m_programId) {
    other.m_programId = 0;
}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        release();
        m_programId = other.m_programId;
        other.m_programId = 0;
    }
    return *this;
}

void Shader::release() noexcept {
    if (m_programId != 0) {
        glDeleteProgram(m_programId);
        m_programId = 0;
    }
}

std::uint32_t Shader::compileStage(std::uint32_t stageType, std::string_view source) {
    const std::uint32_t shader = glCreateShader(stageType);
    const char* srcPtr = source.data();
    const auto srcLen = static_cast<GLint>(source.size());
    glShaderSource(shader, 1, &srcPtr, &srcLen);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_FALSE) {
        GLint logLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> infoLog(static_cast<std::size_t>(std::max(logLength, 1)));
        glGetShaderInfoLog(shader, logLength, nullptr, infoLog.data());
        std::cerr << "[OpenPADS::Shader] Compile error: " << infoLog.data() << '\n';
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool Shader::compileFromSource(std::string_view vertexSource, std::string_view fragmentSource) {
    release();

    const std::uint32_t vs = compileStage(GL_VERTEX_SHADER, vertexSource);
    if (vs == 0) {
        return false;
    }

    const std::uint32_t fs = compileStage(GL_FRAGMENT_SHADER, fragmentSource);
    if (fs == 0) {
        glDeleteShader(vs);
        return false;
    }

    const std::uint32_t program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint linked = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked == GL_FALSE) {
        GLint logLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> infoLog(static_cast<std::size_t>(std::max(logLength, 1)));
        glGetProgramInfoLog(program, logLength, nullptr, infoLog.data());
        std::cerr << "[OpenPADS::Shader] Link error: " << infoLog.data() << '\n';
        glDeleteProgram(program);
        return false;
    }

    m_programId = program;
    return true;
}

void Shader::bind() const noexcept {
    glUseProgram(m_programId);
}

bool Shader::isValid() const noexcept {
    return m_programId != 0;
}

void Shader::setInt(std::string_view name, int value) const noexcept {
    const std::string key(name);
    glUniform1i(glGetUniformLocation(m_programId, key.c_str()), value);
}

void Shader::setFloat(std::string_view name, float value) const noexcept {
    const std::string key(name);
    glUniform1f(glGetUniformLocation(m_programId, key.c_str()), value);
}

void Shader::setVec2(std::string_view name, const glm::vec2& value) const noexcept {
    const std::string key(name);
    glUniform2fv(glGetUniformLocation(m_programId, key.c_str()), 1, glm::value_ptr(value));
}

void Shader::setVec3(std::string_view name, const glm::vec3& value) const noexcept {
    const std::string key(name);
    glUniform3fv(glGetUniformLocation(m_programId, key.c_str()), 1, glm::value_ptr(value));
}

void Shader::setVec4(std::string_view name, const glm::vec4& value) const noexcept {
    const std::string key(name);
    glUniform4fv(glGetUniformLocation(m_programId, key.c_str()), 1, glm::value_ptr(value));
}

void Shader::setMat3(std::string_view name, const glm::mat3& value) const noexcept {
    const std::string key(name);
    glUniformMatrix3fv(glGetUniformLocation(m_programId, key.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}

void Shader::setMat4(std::string_view name, const glm::mat4& value) const noexcept {
    const std::string key(name);
    glUniformMatrix4fv(glGetUniformLocation(m_programId, key.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}

} // namespace opads
