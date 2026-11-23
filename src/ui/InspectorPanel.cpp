#include "ui/InspectorPanel.hpp"

#include <algorithm>
#include <array>
#include <cstring>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

namespace opads {

namespace {
bool editStringField(const char* label, std::string& value) {
    std::array<char, 128> buffer{};
    const std::size_t copyLen = std::min(value.size(), buffer.size() - 1);
    std::memcpy(buffer.data(), value.data(), copyLen);
    buffer[copyLen] = '\0';
    if (ImGui::InputText(label, buffer.data(), buffer.size())) {
        value = buffer.data();
        return true;
    }
    return false;
}
} // namespace

InspectorPanel::InspectorPanel(Aircraft& aircraft)
    : Panel("Inspector")
    , m_aircraft(aircraft) {
}

void InspectorPanel::onRender() {
    if (!m_open) {
        return;
    }

    if (!ImGui::Begin(m_title.c_str(), &m_open)) {
        ImGui::End();
        return;
    }

    const auto selectedId = m_aircraft.getSelectedComponentId();
    Component* selected = selectedId.has_value() ? m_aircraft.findComponent(*selectedId) : nullptr;
    if (selected != nullptr) {
        renderComponentProperties(*selected);
    } else {
        renderAircraftProperties();
    }

    ImGui::End();
}

void InspectorPanel::renderAircraftProperties() {
    ImGui::TextUnformatted("Aircraft Properties");
    ImGui::Separator();

    std::string name = m_aircraft.getName();
    if (editStringField("Name", name)) {
        m_aircraft.setName(std::move(name));
    }

    ImGui::Spacing();
    ImGui::Text("Components: %zu", m_aircraft.getComponents().size());
    ImGui::TextDisabled("Coordinate System: Aerospace Z-Up (X aft, Y stbd, Z up)");
}

void InspectorPanel::renderComponentProperties(Component& component) {
    ImGui::Text("%s (%s)", component.getName().c_str(), component.getTypeName().c_str());
    ImGui::Separator();

    std::string name = component.getName();
    if (editStringField("Name", name)) {
        component.setName(std::move(name));
    }

    bool visible = component.isVisible();
    if (ImGui::Checkbox("Visible", &visible)) {
        component.setVisible(visible);
    }

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
        Transform& transform = component.getTransform();

        glm::vec3 position = transform.getPosition();
        if (ImGui::DragFloat3("Position (m)", glm::value_ptr(position), 0.05f, -1000.0f, 1000.0f, "%.3f")) {
            transform.setPosition(position);
        }

        glm::vec3 rotation = transform.getRotationDeg();
        if (ImGui::DragFloat3("Rotation (deg)", glm::value_ptr(rotation), 0.5f, -360.0f, 360.0f, "%.2f")) {
            transform.setRotationDeg(rotation);
        }

        glm::vec3 scale = transform.getScale();
        if (ImGui::DragFloat3("Scale", glm::value_ptr(scale), 0.02f, 0.01f, 100.0f, "%.3f")) {
            transform.setScale(scale);
        }
    }

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Mesh Diagnostics", ImGuiTreeNodeFlags_DefaultOpen)) {
        const Mesh& mesh = component.getMesh();
        ImGui::Text("Vertices:  %zu", mesh.getVertexCount());
        ImGui::Text("Triangles: %zu", mesh.getTriangleCount());
    }
}

} // namespace opads
