#include "ui/ViewportPanel.hpp"

#include <cstdint>

#include <imgui.h>

namespace opads {

ViewportPanel::ViewportPanel(Aircraft& aircraft, Viewport3D& viewport)
    : Panel("3D Viewport")
    , m_aircraft(aircraft)
    , m_viewport(viewport) {
}

void ViewportPanel::onRender() {
    if (!m_open) {
        return;
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    if (!ImGui::Begin(m_title.c_str(), &m_open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
        ImGui::End();
        ImGui::PopStyleVar();
        return;
    }
    ImGui::PopStyleVar();

    renderToolbar();

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const int viewW = static_cast<int>(avail.x);
    const int viewH = static_cast<int>(avail.y);

    if (viewW > 0 && viewH > 0) {
        m_viewport.resize(viewW, viewH);
        m_viewport.render(m_aircraft);

        const auto texId = static_cast<ImTextureID>(static_cast<std::uintptr_t>(m_viewport.getColorTextureId()));
        ImGui::Image(texId, avail, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));

        handleCameraInput(ImGui::IsItemHovered());
    }

    ImGui::End();
}

void ViewportPanel::renderToolbar() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.0f, 4.0f));
    ImGui::BeginChild("##ViewportToolbar", ImVec2(0.0f, 34.0f), false, ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPos(ImVec2(8.0f, 5.0f));

    OrbitCamera& cam = m_viewport.getCamera();
    if (ImGui::Button("Iso")) {
        cam.setPresetView(ViewPreset::Isometric);
    }
    ImGui::SameLine();
    if (ImGui::Button("Top")) {
        cam.setPresetView(ViewPreset::Top);
    }
    ImGui::SameLine();
    if (ImGui::Button("Front")) {
        cam.setPresetView(ViewPreset::Front);
    }
    ImGui::SameLine();
    if (ImGui::Button("Side")) {
        cam.setPresetView(ViewPreset::Right);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        cam.reset();
    }

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    bool showGrid = m_viewport.isGridVisible();
    if (ImGui::Checkbox("Grid", &showGrid)) {
        m_viewport.setGridVisible(showGrid);
    }
    ImGui::SameLine();

    bool showAxes = m_viewport.isAxesVisible();
    if (ImGui::Checkbox("Axes", &showAxes)) {
        m_viewport.setAxesVisible(showAxes);
    }
    ImGui::SameLine();

    const char* modes[] = {"Shaded", "Wireframe", "Shaded + Wire"};
    int modeIdx = static_cast<int>(m_viewport.getRenderMode());
    ImGui::SetNextItemWidth(130.0f);
    if (ImGui::Combo("##RenderMode", &modeIdx, modes, 3)) {
        m_viewport.setRenderMode(static_cast<RenderMode>(modeIdx));
    }

    ImGui::SameLine();
    ImGui::TextDisabled("| Dist: %.1fm | %.0f FPS", cam.getDistance(), ImGui::GetIO().Framerate);

    ImGui::EndChild();
    ImGui::PopStyleVar();
}

void ViewportPanel::handleCameraInput(bool imageHovered) {
    if (!imageHovered) {
        return;
    }

    ImGuiIO& io = ImGui::GetIO();
    OrbitCamera& cam = m_viewport.getCamera();

    if (io.MouseWheel != 0.0f) {
        cam.zoom(io.MouseWheel);
    }

    const bool orbitDrag =
        ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0f) && !io.KeyShift;
    const bool altLeftOrbit =
        ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f) && io.KeyAlt;
    const bool panDrag =
        ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f) ||
        (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0f) && io.KeyShift);

    if (orbitDrag || altLeftOrbit) {
        cam.orbit(-io.MouseDelta.x * 0.35f, io.MouseDelta.y * 0.35f);
    } else if (panDrag) {
        cam.pan(io.MouseDelta.x, io.MouseDelta.y);
    }
}

} // namespace opads
