#include "ui/Application.hpp"

#include <iostream>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>
#include <implot.h>

#include "core/MeshExporter.hpp"
#include "core/ProjectSerializer.hpp"
#include "ui/AirfoilEditorPanel.hpp"
#include "ui/FileDialog.hpp"
#include "ui/InspectorPanel.hpp"
#include "ui/ModelTreePanel.hpp"
#include "ui/ViewportPanel.hpp"

namespace opads {

Application::Application()
    : m_aircraft("OpenPADS Concept") {
}

Application::~Application() {
    shutdown();
}

bool Application::initialize() {
    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "[OpenPADS] Failed to initialize GLFW.\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    m_window = glfwCreateWindow(1600, 960, "OpenPADS — Parametric Aircraft Design Suite", nullptr, nullptr);
    if (m_window == nullptr) {
        std::cerr << "[OpenPADS] Failed to create GLFW window.\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    if (gladLoadGL(glfwGetProcAddress) == 0) {
        std::cerr << "[OpenPADS] Failed to load OpenGL 4.5 via GLAD.\n";
        return false;
    }

    if (!m_viewport.initialize()) {
        std::cerr << "[OpenPADS] Failed to initialize 3D viewport.\n";
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    applyTheme();

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 450");

    m_panels.push_back(std::make_unique<ModelTreePanel>(m_aircraft, m_undoStack));
    m_panels.push_back(std::make_unique<ViewportPanel>(m_aircraft, m_viewport, m_undoStack));
    m_panels.push_back(std::make_unique<AirfoilEditorPanel>(m_aircraft, m_undoStack, m_window));
    m_panels.push_back(std::make_unique<InspectorPanel>(m_aircraft, m_undoStack, m_window));

    return true;
}

int Application::run() {
    if (!initialize()) {
        return 1;
    }

    while (glfwWindowShouldClose(m_window) == GLFW_FALSE) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        handleGlobalShortcuts();
        renderDockspace();
        for (const auto& panel : m_panels) {
            panel->onRender();
        }

        ImGui::Render();

        int displayW = 0;
        int displayH = 0;
        glfwGetFramebufferSize(m_window, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
    }

    return 0;
}

void Application::shutdown() {
    m_panels.clear();

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
    }

    if (m_window != nullptr) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
}

void Application::applyTheme() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.TabRounding = 3.0f;
    style.ScrollbarRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.FramePadding = ImVec2(6.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 5.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.12f, 0.13f, 0.16f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.22f, 0.32f, 0.45f, 0.70f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.42f, 0.60f, 0.85f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.32f, 0.49f, 0.70f, 1.00f);
    colors[ImGuiCol_Tab] = ImVec4(0.14f, 0.16f, 0.20f, 1.00f);
    colors[ImGuiCol_TabActive] = ImVec4(0.22f, 0.30f, 0.42f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.28f, 0.38f, 0.54f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.14f, 0.16f, 0.20f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.20f, 0.24f, 0.30f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.36f, 0.48f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.34f, 0.46f, 0.62f, 1.00f);
}

void Application::renderDockspace() {
    const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(mainViewport->WorkPos);
    ImGui::SetNextWindowSize(mainViewport->WorkSize);
    ImGui::SetNextWindowViewport(mainViewport->ID);

    const ImGuiWindowFlags hostFlags =
        ImGuiWindowFlags_MenuBar |
        ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoNavFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("OpenPADS_DockHost", nullptr, hostFlags);
    ImGui::PopStyleVar(3);

    renderMenuBar();

    const ImGuiID dockspaceId = ImGui::GetID("OpenPADS_MainDockspace");
    ImGui::DockSpace(dockspaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);

    if (!m_dockLayoutBuilt) {
        buildDefaultDockLayout(dockspaceId);
        m_dockLayoutBuilt = true;
    }

    ImGui::End();
}

void Application::renderMenuBar() {
    if (!ImGui::BeginMenuBar()) {
        return;
    }

    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New Aircraft")) {
            actionNewProject();
        }
        if (ImGui::MenuItem("Open Project...")) {
            actionOpenProject();
        }
        if (ImGui::MenuItem("Save Project")) {
            actionSaveProject(false);
        }
        if (ImGui::MenuItem("Save Project As...")) {
            actionSaveProject(true);
        }
        ImGui::Separator();
        if (ImGui::BeginMenu("Export")) {
            if (ImGui::MenuItem("Binary STL (.stl)...")) {
                actionExportStl();
            }
            if (ImGui::MenuItem("Wavefront OBJ (.obj)...")) {
                actionExportObj();
            }
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Exit", "Alt+F4")) {
            glfwSetWindowShouldClose(m_window, GLFW_TRUE);
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit")) {
        const std::string undoText = m_undoStack.canUndo()
            ? ("Undo " + m_undoStack.getUndoLabel())
            : std::string("Undo");
        if (ImGui::MenuItem(undoText.c_str(), "Ctrl+Z", false, m_undoStack.canUndo())) {
            m_undoStack.undo(m_aircraft);
        }

        const std::string redoText = m_undoStack.canRedo()
            ? ("Redo " + m_undoStack.getRedoLabel())
            : std::string("Redo");
        if (ImGui::MenuItem(redoText.c_str(), "Ctrl+Y", false, m_undoStack.canRedo())) {
            m_undoStack.redo(m_aircraft);
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        for (const auto& panel : m_panels) {
            bool open = panel->isOpen();
            if (ImGui::MenuItem(panel->getTitle().c_str(), nullptr, &open)) {
                panel->setOpen(open);
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Reset Layout")) {
            m_dockLayoutBuilt = false;
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Camera")) {
        OrbitCamera& cam = m_viewport.getCamera();
        if (ImGui::MenuItem("Isometric")) {
            cam.setPresetView(ViewPreset::Isometric);
        }
        if (ImGui::MenuItem("Top")) {
            cam.setPresetView(ViewPreset::Top);
        }
        if (ImGui::MenuItem("Front")) {
            cam.setPresetView(ViewPreset::Front);
        }
        if (ImGui::MenuItem("Right Side")) {
            cam.setPresetView(ViewPreset::Right);
        }
        if (ImGui::MenuItem("Reset Camera")) {
            cam.reset();
        }
        ImGui::EndMenu();
    }

    ImGui::EndMenuBar();
}

void Application::buildDefaultDockLayout(unsigned int dockspaceId) {
    ImGui::DockBuilderRemoveNode(dockspaceId);
    ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspaceId, ImGui::GetMainViewport()->WorkSize);

    ImGuiID centerId = dockspaceId;
    ImGuiID leftId = ImGui::DockBuilderSplitNode(centerId, ImGuiDir_Left, 0.20f, nullptr, &centerId);
    ImGuiID rightId = ImGui::DockBuilderSplitNode(centerId, ImGuiDir_Right, 0.26f, nullptr, &centerId);

    ImGui::DockBuilderDockWindow("Model Tree", leftId);
    ImGui::DockBuilderDockWindow("3D Viewport", centerId);
    ImGui::DockBuilderDockWindow("Airfoil Editor", centerId);
    ImGui::DockBuilderDockWindow("Inspector", rightId);
    ImGui::DockBuilderFinish(dockspaceId);
}

void Application::handleGlobalShortcuts() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput || !io.KeyCtrl) {
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
        if (io.KeyShift) {
            m_undoStack.redo(m_aircraft);
        } else {
            m_undoStack.undo(m_aircraft);
        }
    } else if (ImGui::IsKeyPressed(ImGuiKey_Y, false)) {
        m_undoStack.redo(m_aircraft);
    }
}

void Application::actionNewProject() {
    m_aircraft = Aircraft("Untitled Aircraft");
    m_currentProjectPath.reset();
    m_undoStack.clear();
}

void Application::actionOpenProject() {
    const auto path = FileDialog::openFile(
        m_window,
        "OpenPADS Project (*.opads;*.json)\0*.opads;*.json\0All Files (*.*)\0*.*\0",
        "Open OpenPADS Aircraft Project"
    );
    if (!path.has_value()) {
        return;
    }
    if (ProjectSerializer::loadFromFile(m_aircraft, *path)) {
        m_currentProjectPath = *path;
        m_undoStack.clear();
    }
}

void Application::actionSaveProject(bool forceSaveAs) {
    std::optional<std::filesystem::path> target = m_currentProjectPath;
    if (forceSaveAs || !target.has_value()) {
        target = FileDialog::saveFile(
            m_window,
            "OpenPADS Project (*.opads)\0*.opads\0JSON Files (*.json)\0*.json\0",
            "opads",
            "Save OpenPADS Aircraft Project"
        );
    }
    if (!target.has_value()) {
        return;
    }
    if (ProjectSerializer::saveToFile(m_aircraft, *target)) {
        m_currentProjectPath = *target;
    }
}

void Application::actionExportStl() {
    const auto path = FileDialog::saveFile(
        m_window,
        "STL Mesh (*.stl)\0*.stl\0",
        "stl",
        "Export Binary STL"
    );
    if (path.has_value()) {
        m_aircraft.rebuildDirty();
        MeshExporter::exportBinaryStl(m_aircraft, *path);
    }
}

void Application::actionExportObj() {
    const auto path = FileDialog::saveFile(
        m_window,
        "Wavefront OBJ (*.obj)\0*.obj\0",
        "obj",
        "Export Wavefront OBJ"
    );
    if (path.has_value()) {
        m_aircraft.rebuildDirty();
        MeshExporter::exportObj(m_aircraft, *path);
    }
}

} // namespace opads
