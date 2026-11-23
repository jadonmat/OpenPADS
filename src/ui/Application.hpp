#pragma once

#include <memory>
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

    GLFWwindow* m_window{nullptr};
    Aircraft m_aircraft;
    Viewport3D m_viewport;
    std::vector<std::unique_ptr<Panel>> m_panels;
    bool m_dockLayoutBuilt{false};
};

} // namespace opads
