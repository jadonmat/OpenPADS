#include "ui/ViewportPanel.hpp"

#include <cstdint>
#include <string>

#include <imgui.h>

#include "core/Fuselage.hpp"
#include "core/Wing.hpp"
#include "render/RayPicker.hpp"

namespace opads {

namespace {

bool toolbarModeButton(const char* label, bool active) {
    if (active) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.28f, 0.48f, 0.72f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.34f, 0.56f, 0.82f, 1.0f));
    }
    const bool clicked = ImGui::Button(label);
    if (active) {
        ImGui::PopStyleColor(2);
    }
    return clicked;
}

} // namespace

ViewportPanel::ViewportPanel(Aircraft& aircraft, Viewport3D& viewport, UndoStack& undoStack)
    : Panel("3D Viewport")
    , m_aircraft(aircraft)
    , m_viewport(viewport)
    , m_undoStack(undoStack) {
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

        const ImVec2 rectMin = ImGui::GetItemRectMin();
        const ImVec2 rectSize = ImGui::GetItemRectSize();
        handleViewportInput(
            ImGui::IsItemHovered(),
            glm::vec2(rectMin.x, rectMin.y),
            glm::vec2(rectSize.x, rectSize.y)
        );
        renderOverlayHud(glm::vec2(rectMin.x, rectMin.y), glm::vec2(rectSize.x, rectSize.y));
    }

    ImGui::End();
}

void ViewportPanel::renderToolbar() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 4.0f));
    ImGui::BeginChild("##ViewportToolbar", ImVec2(0.0f, 60.0f), false, ImGuiWindowFlags_NoScrollbar);

    // Row 1: Editing tools, Geometry Snap, and Undo/Redo
    ImGui::SetCursorPos(ImVec2(8.0f, 4.0f));
    Gizmo& gizmo = m_viewport.getGizmo();
    const GizmoTarget target = gizmo.getTarget();
    if (toolbarModeButton("Part", target == GizmoTarget::Part)) {
        gizmo.setTarget(GizmoTarget::Part);
    }
    ImGui::SameLine();
    if (toolbarModeButton("XSec (Tab)", target == GizmoTarget::CrossSection)) {
        gizmo.setTarget(GizmoTarget::CrossSection);
    }

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    const GizmoMode mode = gizmo.getMode();
    if (toolbarModeButton("Select (Q)", mode == GizmoMode::Select)) {
        gizmo.setMode(GizmoMode::Select);
    }
    ImGui::SameLine();
    if (toolbarModeButton("Move (W)", mode == GizmoMode::Translate)) {
        gizmo.setMode(GizmoMode::Translate);
    }
    ImGui::SameLine();
    if (toolbarModeButton("Rotate (R)", mode == GizmoMode::Rotate)) {
        gizmo.setMode(GizmoMode::Rotate);
    }
    ImGui::SameLine();
    if (toolbarModeButton("Scale (S)", mode == GizmoMode::Scale)) {
        gizmo.setMode(GizmoMode::Scale);
    }

    ImGui::SameLine();
    bool surfSnap = gizmo.isSurfaceSnapEnabled();
    if (ImGui::Checkbox("Geometry Snap", &surfSnap)) {
        gizmo.setSurfaceSnapEnabled(surfSnap);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Snap moving part or XSec flush to neighboring part geometry");
    }

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    if (!m_undoStack.canUndo()) ImGui::BeginDisabled();
    if (ImGui::Button("Undo")) {
        m_undoStack.undo(m_aircraft);
    }
    if (!m_undoStack.canUndo()) ImGui::EndDisabled();

    ImGui::SameLine();
    if (!m_undoStack.canRedo()) ImGui::BeginDisabled();
    if (ImGui::Button("Redo")) {
        m_undoStack.redo(m_aircraft);
    }
    if (!m_undoStack.canRedo()) ImGui::EndDisabled();

    // Row 2: Camera presets & Display options
    ImGui::SetCursorPos(ImVec2(8.0f, 32.0f));
    OrbitCamera& cam = m_viewport.getCamera();
    if (ImGui::Button("Iso")) cam.setPresetView(ViewPreset::Isometric);
    ImGui::SameLine();
    if (ImGui::Button("Top")) cam.setPresetView(ViewPreset::Top);
    ImGui::SameLine();
    if (ImGui::Button("Front")) cam.setPresetView(ViewPreset::Front);
    ImGui::SameLine();
    if (ImGui::Button("Side")) cam.setPresetView(ViewPreset::Right);
    ImGui::SameLine();
    if (ImGui::Button("Reset")) cam.reset();

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    const char* modes[] = {"Shaded", "Wireframe", "Shaded + Wire"};
    int modeIdx = static_cast<int>(m_viewport.getRenderMode());
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::Combo("##RenderMode", &modeIdx, modes, 3)) {
        m_viewport.setRenderMode(static_cast<RenderMode>(modeIdx));
    }

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
    ImGui::TextDisabled("| %.0f FPS", ImGui::GetIO().Framerate);

    ImGui::EndChild();
    ImGui::PopStyleVar();
}

void ViewportPanel::renderOverlayHud(const glm::vec2& imageMin, const glm::vec2& imageSize) {
    if (imageSize.x < 260.0f || imageSize.y < 100.0f) {
        return;
    }

    std::string status = "No selection";
    const auto selectedId = m_aircraft.getSelectedComponentId();
    if (selectedId.has_value()) {
        if (const Component* comp = m_aircraft.findComponent(*selectedId)) {
            status = comp->getName() + " (" + comp->getTypeName() + ")";
            if (const auto* wing = dynamic_cast<const Wing*>(comp)) {
                status += " -> XSec " + std::to_string(wing->getSelectedCrossSection());
            } else if (const auto* fuse = dynamic_cast<const Fuselage*>(comp)) {
                status += " -> XSec " + std::to_string(fuse->getSelectedSection());
            }
        }
    }

    const Gizmo& gizmo = m_viewport.getGizmo();
    const char* targetStr = (gizmo.getTarget() == GizmoTarget::Part) ? "Part" : "XSec";
    const char* snapStr = gizmo.isSurfaceSnapEnabled() ? " | Snap ON" : "";
    const std::string hudText = status + "  |  Target: " + targetStr + snapStr +
        "\nLMB: Pick/Gizmo   RMB: Orbit   MMB: Pan   Ctrl+Z/Y: Undo/Redo";

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 textSize = ImGui::CalcTextSize(hudText.c_str());
    const ImVec2 boxMin(imageMin.x + 12.0f, imageMin.y + imageSize.y - textSize.y - 22.0f);
    const ImVec2 boxMax(boxMin.x + textSize.x + 18.0f, boxMin.y + textSize.y + 12.0f);
    drawList->AddRectFilled(boxMin, boxMax, IM_COL32(18, 21, 27, 195), 5.0f);
    drawList->AddRect(boxMin, boxMax, IM_COL32(70, 82, 98, 200), 5.0f);
    drawList->AddText(ImVec2(boxMin.x + 9.0f, boxMin.y + 6.0f), IM_COL32(215, 224, 236, 255), hudText.c_str());
}

void ViewportPanel::handleViewportInput(bool imageHovered, const glm::vec2& imageMin, const glm::vec2& imageSize) {
    ImGuiIO& io = ImGui::GetIO();
    OrbitCamera& cam = m_viewport.getCamera();
    Gizmo& gizmo = m_viewport.getGizmo();

    if (imageHovered && !io.WantTextInput && !io.KeyCtrl) {
        if (ImGui::IsKeyPressed(ImGuiKey_Tab)) {
            gizmo.setTarget(
                gizmo.getTarget() == GizmoTarget::Part ? GizmoTarget::CrossSection : GizmoTarget::Part
            );
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Q)) {
            gizmo.setMode(GizmoMode::Select);
        } else if (ImGui::IsKeyPressed(ImGuiKey_W)) {
            gizmo.setMode(GizmoMode::Translate);
        } else if (ImGui::IsKeyPressed(ImGuiKey_R)) {
            gizmo.setMode(GizmoMode::Rotate);
        } else if (ImGui::IsKeyPressed(ImGuiKey_S)) {
            gizmo.setMode(GizmoMode::Scale);
        }
    }

    if (imageSize.x <= 1.0f || imageSize.y <= 1.0f) {
        return;
    }

    const float ndcX = 2.0f * (io.MousePos.x - imageMin.x) / imageSize.x - 1.0f;
    const float ndcY = 1.0f - 2.0f * (io.MousePos.y - imageMin.y) / imageSize.y;
    const float aspect = imageSize.x / imageSize.y;
    const Ray ray = cam.screenPointToRay(ndcX, ndcY, aspect);

    if (gizmo.isDragging()) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            gizmo.updateDrag(ray, m_aircraft, cam);
        } else {
            gizmo.endDrag();
            m_undoStack.commitTransaction(m_aircraft);
        }
        return;
    }

    if (!imageHovered) {
        return;
    }

    gizmo.updateHover(ray, m_aircraft, cam);

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.KeyAlt) {
        if (gizmo.beginDrag(ray, m_aircraft, cam)) {
            m_undoStack.beginTransaction(m_aircraft, "3D Gizmo Edit");
        } else {
            const PickResult pick = RayPicker::pickAircraft(ray, m_aircraft);
            if (pick.hit) {
                m_aircraft.setSelectedComponentId(pick.componentId);
                if (pick.crossSectionIndex.has_value()) {
                    Component* pickedComp = m_aircraft.findComponent(pick.componentId);
                    if (auto* wing = dynamic_cast<Wing*>(pickedComp)) {
                        wing->setSelectedCrossSection(*pick.crossSectionIndex);
                    } else if (auto* fuse = dynamic_cast<Fuselage*>(pickedComp)) {
                        fuse->setSelectedSection(*pick.crossSectionIndex);
                    }
                }
            } else {
                m_aircraft.setSelectedComponentId(std::nullopt);
            }
        }
    }

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
