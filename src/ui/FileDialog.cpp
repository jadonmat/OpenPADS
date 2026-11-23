#include "ui/FileDialog.hpp"

#include <array>

#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#endif

namespace opads {

std::optional<std::filesystem::path> FileDialog::openFile(
    GLFWwindow* owner,
    const char* filterSpec,
    const char* title
) {
#ifdef _WIN32
    std::array<char, 1024> fileBuf{};
    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = (owner != nullptr) ? glfwGetWin32Window(owner) : nullptr;
    ofn.lpstrFilter = filterSpec;
    ofn.lpstrFile = fileBuf.data();
    ofn.nMaxFile = static_cast<DWORD>(fileBuf.size());
    ofn.lpstrTitle = title;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&ofn) != FALSE) {
        return std::filesystem::path(fileBuf.data());
    }
#else
    (void)owner;
    (void)filterSpec;
    (void)title;
#endif
    return std::nullopt;
}

std::optional<std::filesystem::path> FileDialog::saveFile(
    GLFWwindow* owner,
    const char* filterSpec,
    const char* defaultExtension,
    const char* title
) {
#ifdef _WIN32
    std::array<char, 1024> fileBuf{};
    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = (owner != nullptr) ? glfwGetWin32Window(owner) : nullptr;
    ofn.lpstrFilter = filterSpec;
    ofn.lpstrFile = fileBuf.data();
    ofn.nMaxFile = static_cast<DWORD>(fileBuf.size());
    ofn.lpstrDefExt = defaultExtension;
    ofn.lpstrTitle = title;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;

    if (GetSaveFileNameA(&ofn) != FALSE) {
        return std::filesystem::path(fileBuf.data());
    }
#else
    (void)owner;
    (void)filterSpec;
    (void)defaultExtension;
    (void)title;
#endif
    return std::nullopt;
}

} // namespace opads
