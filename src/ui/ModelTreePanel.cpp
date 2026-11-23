#include "ui/ModelTreePanel.hpp"

#include <string>

#include <imgui.h>

#include "core/Fuselage.hpp"
#include "core/Wing.hpp"

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

    const auto selectedId = m_aircraft.getSelectedComponentId();
    if (selectedId.has_value()) {
        ImGui::SameLine();
        if (ImGui::Button("Delete")) {
            m_aircraft.removeComponent(*selectedId);
        }
    }

    if (ImGui::BeginPopup("AddComponentPopup")) {
        ImGui::TextDisabled("Component Types");
        ImGui::Separator();
        if (ImGui::MenuItem("Wing")) {
            const std::uint64_t newId = m_aircraft.allocateComponentId();
            const std::string wingName = "Wing " + std::to_string(newId);
            m_aircraft.addComponent(std::make_unique<Wing>(newId, wingName));
            m_aircraft.setSelectedComponentId(newId);
        }
        if (ImGui::MenuItem("Fuselage")) {
            const std::uint64_t newId = m_aircraft.allocateComponentId();
            const std::string fuseName = "Fuselage " + std::to_string(newId);
            m_aircraft.addComponent(std::make_unique<Fuselage>(newId, fuseName));
            m_aircraft.setSelectedComponentId(newId);
        }
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
        const auto currentSelected = m_aircraft.getSelectedComponentId();
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

                const bool isSelected = currentSelected.has_value() && *currentSelected == component->getId();
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
