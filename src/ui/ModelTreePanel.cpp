#include "ui/ModelTreePanel.hpp"

#include <imgui.h>

namespace opads {

ModelTreePanel::ModelTreePanel(Aircraft& aircraft)
    : Panel("Model Tree")
    , m_aircraft(aircraft) {
}

void ModelTreePanel::onRender() {
    if (!m_open) {
        return;
    }

    if (!ImGui::Begin(m_title.c_str(), &m_open)) {
        ImGui::End();
        return;
    }

    if (ImGui::Button("+ Add Component")) {
        ImGui::OpenPopup("AddComponentPopup");
    }

    if (ImGui::BeginPopup("AddComponentPopup")) {
        ImGui::TextDisabled("Component Types");
        ImGui::Separator();
        ImGui::MenuItem("Wing (Milestone 2)", nullptr, false, false);
        ImGui::MenuItem("Fuselage (Milestone 4)", nullptr, false, false);
        ImGui::EndPopup();
    }

    ImGui::Separator();

    const ImGuiTreeNodeFlags rootFlags =
        ImGuiTreeNodeFlags_DefaultOpen |
        ImGuiTreeNodeFlags_OpenOnArrow |
        ImGuiTreeNodeFlags_SpanAvailWidth |
        (!m_aircraft.getSelectedComponentId().has_value() ? ImGuiTreeNodeFlags_Selected : 0);

    const bool rootOpen = ImGui::TreeNodeEx("##AircraftRoot", rootFlags, "%s", m_aircraft.getName().c_str());
    if (ImGui::IsItemClicked()) {
        m_aircraft.setSelectedComponentId(std::nullopt);
    }

    if (rootOpen) {
        const auto selectedId = m_aircraft.getSelectedComponentId();
        if (m_aircraft.getComponents().empty()) {
            ImGui::TextDisabled("No components yet");
        } else {
            for (const auto& component : m_aircraft.getComponents()) {
                ImGui::PushID(static_cast<int>(component->getId()));

                bool visible = component->isVisible();
                if (ImGui::Checkbox("##vis", &visible)) {
                    component->setVisible(visible);
                }
                ImGui::SameLine();

                const bool isSelected = selectedId.has_value() && *selectedId == component->getId();
                const ImGuiTreeNodeFlags leafFlags =
                    ImGuiTreeNodeFlags_Leaf |
                    ImGuiTreeNodeFlags_NoTreePushOnOpen |
                    ImGuiTreeNodeFlags_SpanAvailWidth |
                    (isSelected ? ImGuiTreeNodeFlags_Selected : 0);

                ImGui::TreeNodeEx(
                    component->getName().c_str(),
                    leafFlags,
                    "%s [%s]",
                    component->getName().c_str(),
                    component->getTypeName().c_str()
                );

                if (ImGui::IsItemClicked()) {
                    m_aircraft.setSelectedComponentId(component->getId());
                }

                ImGui::PopID();
            }
        }
        ImGui::TreePop();
    }

    ImGui::End();
}

} // namespace opads
