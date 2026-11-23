#pragma once

#include <filesystem>
#include <optional>
#include <string>

struct GLFWwindow;

namespace opads {

/// Native OS open/save file dialog helpers.
class FileDialog {
public:
    static std::optional<std::filesystem::path> openFile(
        GLFWwindow* owner,
        const char* filterSpec,
        const char* title
    );

    static std::optional<std::filesystem::path> saveFile(
        GLFWwindow* owner,
        const char* filterSpec,
        const char* defaultExtension,
        const char* title
    );
};

} // namespace opads
