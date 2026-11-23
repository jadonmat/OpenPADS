#pragma once

#include <cstdint>
#include <string_view>

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace opads {

/// RAII wrapper around an OpenGL shader program.
class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    bool compileFromSource(std::string_view vertexSource, std::string_view fragmentSource);
    void bind() const noexcept;
    bool isValid() const noexcept;

    void setInt(std::string_view name, int value) const noexcept;
    void setFloat(std::string_view name, float value) const noexcept;
    void setVec2(std::string_view name, const glm::vec2& value) const noexcept;
    void setVec3(std::string_view name, const glm::vec3& value) const noexcept;
    void setVec4(std::string_view name, const glm::vec4& value) const noexcept;
    void setMat3(std::string_view name, const glm::mat3& value) const noexcept;
    void setMat4(std::string_view name, const glm::mat4& value) const noexcept;

private:
    void release() noexcept;
    static std::uint32_t compileStage(std::uint32_t stageType, std::string_view source);

    std::uint32_t m_programId{0};
};

} // namespace opads
