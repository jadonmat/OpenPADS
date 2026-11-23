#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include "core/Aircraft.hpp"
#include "render/Viewport3D.hpp"
#include "ui/Panel.hpp"

struct GLFWwindow;

namespace opads {

/// Top-level application owning the GLFW window, Dear ImGui dockspace, and editor panels.
class Application {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    bool initialize();
    int run();

private:
    void shutdown();
    void applyTheme();
    void renderDockspace();
    void renderMenuBar();
    void buildDefaultDockLayout(unsigned int dockspaceId);
    void actionNewProject();
    void actionOpenProject();
    void actionSaveProject(bool forceSaveAs);
    void actionExportStl();
    void actionExportObj();

    GLFWwindow* m_window{nullptr};
    Aircraft m_aircraft;
    Viewport3D m_viewport;
    std::vector<std::unique_ptr<Panel>> m_panels;
    std::optional<std::filesystem::path> m_currentProjectPath;
    bool m_dockLayoutBuilt{false};
};

} // namespace opads
