#include "ui/ModelTreePanel.hpp"

#include <string>

#include <imgui.h>

#include "core/Fuselage.hpp"
#include "core/ProjectSerializer.hpp"
#include "core/Wing.hpp"

namespace opads {

ModelTreePanel::ModelTreePanel(Aircraft& aircraft, UndoStack& undoStack)
    : Panel("Model Tree")
    , m_aircraft(aircraft)
    , m_undoStack(undoStack) {
}

void ModelTreePanel::onRender() {
    if (!m_open) {
        return;
    }

    if (!ImGui::Begin(m_title.c_str(), &m_open)) {
        ImGui::End();
        return;
    }

    if (ImGui::Button("+ Wing")) {
        const std::string before = ProjectSerializer::saveToString(m_aircraft);
        const std::uint64_t newId = m_aircraft.allocateComponentId();
        const std::string wingName = "Wing " + std::to_string(newId);
        m_aircraft.addComponent(std::make_unique<Wing>(newId, wingName));
        m_aircraft.setSelectedComponentId(newId);
        m_undoStack.pushSnapshot(before, m_aircraft, "Add " + wingName);
    }
    ImGui::SameLine();
    if (ImGui::Button("+ Fuselage")) {
        const std::string before = ProjectSerializer::saveToString(m_aircraft);
        const std::uint64_t newId = m_aircraft.allocateComponentId();
        const std::string fuseName = "Fuselage " + std::to_string(newId);
        m_aircraft.addComponent(std::make_unique<Fuselage>(newId, fuseName));
        m_aircraft.setSelectedComponentId(newId);
        m_undoStack.pushSnapshot(before, m_aircraft, "Add " + fuseName);
    }

    const auto selectedId = m_aircraft.getSelectedComponentId();
    if (selectedId.has_value()) {
        ImGui::SameLine();
        if (ImGui::Button("Delete")) {
            const std::string before = ProjectSerializer::saveToString(m_aircraft);
            const Component* target = m_aircraft.findComponent(*selectedId);
            const std::string compName = target ? target->getName() : "Component";
            if (m_aircraft.removeComponent(*selectedId)) {
                m_undoStack.pushSnapshot(before, m_aircraft, "Delete " + compName);
            }
        }
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
            ImGui::TextDisabled("Empty aircraft. Click + Wing or + Fuselage.");
        } else {
            std::optional<std::uint64_t> pendingDeleteId;
            for (const auto& component : m_aircraft.getComponents()) {
                ImGui::PushID(static_cast<int>(component->getId()));

                bool visible = component->isVisible();
                if (ImGui::Checkbox("##vis", &visible)) {
                    const std::string before = ProjectSerializer::saveToString(m_aircraft);
                    component->setVisible(visible);
                    m_undoStack.pushSnapshot(before, m_aircraft, "Toggle Visibility");
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
                    "%s  [%s]",
                    component->getName().c_str(),
                    component->getTypeName().c_str()
                );

                if (ImGui::IsItemClicked()) {
                    m_aircraft.setSelectedComponentId(component->getId());
                }

                if (ImGui::BeginPopupContextItem()) {
                    if (ImGui::MenuItem("Select")) {
                        m_aircraft.setSelectedComponentId(component->getId());
                    }
                    if (ImGui::MenuItem("Delete")) {
                        pendingDeleteId = component->getId();
                    }
                    ImGui::EndPopup();
                }

                ImGui::PopID();
            }

            if (pendingDeleteId.has_value()) {
                const std::string before = ProjectSerializer::saveToString(m_aircraft);
                if (m_aircraft.removeComponent(*pendingDeleteId)) {
                    m_undoStack.pushSnapshot(before, m_aircraft, "Delete Component");
                }
            }
        }
        ImGui::TreePop();
    }

    ImGui::End();
}

} // namespace opads
