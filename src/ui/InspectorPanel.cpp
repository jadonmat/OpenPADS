#include "ui/InspectorPanel.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <string>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include "core/Airfoil.hpp"
#include "core/Parameter.hpp"
#include "core/Wing.hpp"

namespace opads {

namespace {

bool editStringField(const char* label, std::string& value, std::size_t maxLen = 128) {
    std::array<char, 256> buffer{};
    const std::size_t limit = std::min(maxLen, buffer.size()) - 1;
    const std::size_t copyLen = std::min(value.size(), limit);
    std::memcpy(buffer.data(), value.data(), copyLen);
    buffer[copyLen] = '\0';
    if (ImGui::InputText(label, buffer.data(), limit + 1)) {
        value = buffer.data();
        return true;
    }
    return false;
}

std::filesystem::path locateAirfoilLibraryDir() {
    std::error_code ec;
    const std::array<std::filesystem::path, 5> candidates = {
        std::filesystem::path("data/airfoils"),
        std::filesystem::path("../data/airfoils"),
        std::filesystem::path("../../data/airfoils"),
        std::filesystem::path("../../../data/airfoils"),
        std::filesystem::path(OPENPADS_DATA_DIR) / "airfoils"
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate, ec) && std::filesystem::is_directory(candidate, ec)) {
            return candidate;
        }
    }
    return {};
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

    if (!m_datLibraryScanned) {
        refreshDatLibrary();
        m_datLibraryScanned = true;
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

void InspectorPanel::refreshDatLibrary() {
    const std::filesystem::path dir = locateAirfoilLibraryDir();
    m_datLibrary = Airfoil::discoverDatLibrary(dir);
}

bool InspectorPanel::editParameterDrag(Parameter& param) {
    float val = param.getFloat();
    const std::string label = param.getUnits().empty()
        ? param.getName()
        : (param.getName() + " (" + param.getUnits() + ")");
    if (ImGui::DragFloat(
            label.c_str(),
            &val,
            static_cast<float>(param.getStep()),
            static_cast<float>(param.getMin()),
            static_cast<float>(param.getMax()),
            "%.3f"
        )) {
        return param.setFloat(val);
    }
    return false;
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

    if (auto* wing = dynamic_cast<Wing*>(&component)) {
        ImGui::Spacing();
        renderWingEditor(*wing);
    } else {
        ImGui::Spacing();
        renderTransformSection(component);
    }
}

void InspectorPanel::renderWingEditor(Wing& wing) {
    // Live aero-geometric summary header
    if (ImGui::BeginTable("##WingMetrics", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Area S:  %.3f m^2", wing.getPlanformArea());
        ImGui::Text("Span b:  %.3f m", wing.getSpan());
        ImGui::Text("Aspect Ratio: %.2f", wing.getAspectRatio());
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("MAC:     %.3f m", wing.getMeanAerodynamicChord());
        ImGui::Text("Wetted:  %.3f m^2", wing.getWettedArea());
        ImGui::Text("Bays:    %zu", wing.getSectionCount());
        ImGui::EndTable();
    }

    ImGui::Spacing();
    if (ImGui::BeginTabBar("##WingTabs")) {
        if (ImGui::BeginTabItem("Planform & Bays")) {
            renderWingPlanformTab(wing);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Cross Sections")) {
            renderWingCrossSectionsTab(wing);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Transform")) {
            renderTransformSection(wing);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

void InspectorPanel::renderWingPlanformTab(Wing& wing) {
    int symIndex = (wing.getSymmetry() == SymmetryMode::XZ) ? 1 : 0;
    const char* symItems[] = {"None (Single Side)", "XZ Plane (Left + Right)"};
    if (ImGui::Combo("Symmetry", &symIndex, symItems, 2)) {
        wing.setSymmetry(symIndex == 1 ? SymmetryMode::XZ : SymmetryMode::None);
    }

    bool capTips = wing.getCapTips();
    if (ImGui::Checkbox("Cap Tips", &capTips)) {
        wing.setCapTips(capTips);
    }

    int chordPts = wing.getChordPoints();
    if (ImGui::SliderInt("Chord Pts", &chordPts, 15, 91)) {
        wing.setChordPoints(chordPts);
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Trapezoidal Bays");

    if (ImGui::Button("+ Add Bay")) {
        wing.addSection();
        m_selectedBay = wing.getSectionCount() - 1;
    }
    ImGui::SameLine();
    const bool canRemove = wing.getSectionCount() > 1;
    if (!canRemove) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("- Remove Bay")) {
        wing.removeSection(m_selectedBay);
        m_selectedBay = std::min(m_selectedBay, wing.getSectionCount() - 1);
    }
    if (!canRemove) {
        ImGui::EndDisabled();
    }

    m_selectedBay = std::min(m_selectedBay, wing.getSectionCount() - 1);
    if (ImGui::BeginListBox("##BayList", ImVec2(-1.0f, 82.0f))) {
        for (std::size_t i = 0; i < wing.getSectionCount(); ++i) {
            const WingSection& sec = wing.getSection(i);
            const std::string label =
                "Bay " + std::to_string(i + 1) +
                "  (b=" + std::to_string(sec.span().getFloat()).substr(0, 4) +
                "m, cr=" + std::to_string(sec.rootChord().getFloat()).substr(0, 4) +
                ", ct=" + std::to_string(sec.tipChord().getFloat()).substr(0, 4) + ")";
            const bool selected = (i == m_selectedBay);
            if (ImGui::Selectable(label.c_str(), selected)) {
                m_selectedBay = i;
                wing.setSelectedCrossSection(i + 1);
            }
        }
        ImGui::EndListBox();
    }

    WingSection& bay = wing.getSection(m_selectedBay);
    bool changed = false;
    changed |= editParameterDrag(bay.span());
    changed |= editParameterDrag(bay.rootChord());
    changed |= editParameterDrag(bay.tipChord());
    changed |= editParameterDrag(bay.sweepDeg());
    changed |= editParameterDrag(bay.sweepLocation());
    changed |= editParameterDrag(bay.dihedralDeg());

    int spanSub = bay.getSpanSubdivisions();
    if (ImGui::SliderInt("Span Subdivisions", &spanSub, 1, 32)) {
        bay.setSpanSubdivisions(spanSub);
        changed = true;
    }

    if (changed) {
        wing.syncSectionsFromBay(m_selectedBay);
    }
}

void InspectorPanel::renderWingCrossSectionsTab(Wing& wing) {
    const std::size_t xsecCount = wing.getCrossSectionCount();
    std::size_t selectedXSec = std::min(wing.getSelectedCrossSection(), xsecCount - 1);

    if (ImGui::BeginListBox("##XSecList", ImVec2(-1.0f, 92.0f))) {
        for (std::size_t i = 0; i < xsecCount; ++i) {
            const std::string xsecLabel = (i == 0)
                ? ("XSec 0 (Root) - " + wing.getSection(0).getRootAirfoil().getName())
                : ("XSec " + std::to_string(i) + " - " + wing.getSection(i - 1).getTipAirfoil().getName());
            if (ImGui::Selectable(xsecLabel.c_str(), i == selectedXSec)) {
                selectedXSec = i;
                wing.setSelectedCrossSection(i);
            }
        }
        ImGui::EndListBox();
    }

    Parameter* chordParam = nullptr;
    Parameter* twistParam = nullptr;
    Parameter* twistLocParam = nullptr;
    Airfoil* airfoil = nullptr;

    if (selectedXSec == 0) {
        WingSection& firstBay = wing.getSection(0);
        chordParam = &firstBay.rootChord();
        twistParam = &firstBay.rootTwistDeg();
        twistLocParam = &firstBay.twistLocation();
        airfoil = &firstBay.getRootAirfoil();
    } else {
        WingSection& bay = wing.getSection(selectedXSec - 1);
        chordParam = &bay.tipChord();
        twistParam = &bay.tipTwistDeg();
        twistLocParam = &bay.twistLocation();
        airfoil = &bay.getTipAirfoil();
    }

    bool changed = false;
    changed |= editParameterDrag(*chordParam);
    changed |= editParameterDrag(*twistParam);
    changed |= editParameterDrag(*twistLocParam);

    ImGui::Separator();
    ImGui::Text("Airfoil: %s", airfoil->getName().c_str());
    changed |= renderAirfoilEditor(*airfoil);
    renderAirfoilPreview(*airfoil);

    if (changed) {
        wing.syncSectionsFromCrossSection(selectedXSec);
    }
}

bool InspectorPanel::renderAirfoilEditor(Airfoil& airfoil) {
    bool changed = false;

    int typeIdx = static_cast<int>(airfoil.getType());
    const char* types[] = {"NACA 4-Digit", "NACA 5-Digit", "Database / .dat File"};
    if (ImGui::Combo("Airfoil Type", &typeIdx, types, 3)) {
        if (typeIdx == 0) {
            airfoil.setNaca4(airfoil.getNaca4Code());
            changed = true;
        } else if (typeIdx == 1) {
            airfoil.setNaca5(airfoil.getNaca5Code());
            changed = true;
        } else if (!m_datLibrary.empty()) {
            changed |= airfoil.loadFromDatFile(m_datLibrary.front());
        }
    }

    if (airfoil.getType() == AirfoilType::Naca4) {
        std::string code = airfoil.getNaca4Code();
        if (editStringField("NACA 4 Code", code, 8)) {
            if (code.size() == 4) {
                airfoil.setNaca4(code);
                changed = true;
            }
        }
    } else if (airfoil.getType() == AirfoilType::Naca5) {
        std::string code = airfoil.getNaca5Code();
        if (editStringField("NACA 5 Code", code, 8)) {
            if (code.size() == 5) {
                airfoil.setNaca5(code);
                changed = true;
            }
        }
    } else {
        if (ImGui::Button("Refresh Library")) {
            refreshDatLibrary();
        }
        if (m_datLibrary.empty()) {
            ImGui::TextDisabled("No .dat files found in data/airfoils/");
        } else {
            const std::string currentStem = airfoil.getDatPath().empty()
                ? std::string("Select .dat airfoil...")
                : std::filesystem::path(airfoil.getDatPath()).stem().string();
            if (ImGui::BeginCombo("Airfoil Library", currentStem.c_str())) {
                for (const auto& path : m_datLibrary) {
                    const std::string stem = path.stem().string();
                    const bool isSelected = (path.string() == airfoil.getDatPath());
                    if (ImGui::Selectable(stem.c_str(), isSelected)) {
                        changed |= airfoil.loadFromDatFile(path);
                    }
                }
                ImGui::EndCombo();
            }
        }

        std::string customPath = airfoil.getDatPath();
        if (editStringField("Custom .dat Path", customPath, 256)) {
            if (airfoil.loadFromDatFile(customPath)) {
                changed = true;
            }
        }
    }

    float thickScale = airfoil.getThicknessScale();
    if (ImGui::DragFloat("Thickness Scale", &thickScale, 0.01f, 0.1f, 3.0f, "%.2f")) {
        changed |= airfoil.setThicknessScale(thickScale);
    }

    float camberScale = airfoil.getCamberScale();
    if (ImGui::DragFloat("Camber Scale", &camberScale, 0.02f, -2.0f, 2.5f, "%.2f")) {
        changed |= airfoil.setCamberScale(camberScale);
    }

    float teGap = airfoil.getTeGap();
    if (ImGui::DragFloat("TE Gap", &teGap, 0.0005f, 0.0f, 0.03f, "%.4f")) {
        changed |= airfoil.setTeGap(teGap);
    }

    if (airfoil.getType() != AirfoilType::DatFile) {
        bool sharpTE = airfoil.isSharpTE();
        if (ImGui::Checkbox("Sharp Trailing Edge", &sharpTE)) {
            changed |= airfoil.setSharpTE(sharpTE);
        }
    }

    ImGui::TextDisabled("Max t/c: %.1f%%   Max Camber: %.1f%%",
        airfoil.getMaxThicknessRatio() * 100.0f,
        airfoil.getMaxCamberRatio() * 100.0f);

    return changed;
}

void InspectorPanel::renderAirfoilPreview(const Airfoil& airfoil) {
    const auto& loop = airfoil.getClosedLoop();
    const auto& camber = airfoil.getCamberLine();
    if (loop.size() < 3) {
        return;
    }

    const ImVec2 size(ImGui::GetContentRegionAvail().x, 120.0f);
    if (size.x < 40.0f) {
        return;
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(22, 25, 31, 255), 4.0f);
    drawList->AddRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(60, 68, 82, 255), 4.0f);

    const float padX = 14.0f;
    const float chordPx = size.x - 2.0f * padX;
    const float scale = chordPx;
    const float midY = origin.y + size.y * 0.55f;

    // Chord reference line
    drawList->AddLine(
        ImVec2(origin.x + padX, midY),
        ImVec2(origin.x + padX + chordPx, midY),
        IM_COL32(75, 85, 100, 180),
        1.0f
    );

    const auto toScreen = [&](const glm::vec2& p) -> ImVec2 {
        return ImVec2(origin.x + padX + p.x * scale, midY - p.y * scale);
    };

    for (std::size_t i = 0; i + 1 < camber.size(); ++i) {
        drawList->AddLine(toScreen(camber[i]), toScreen(camber[i + 1]), IM_COL32(235, 175, 65, 180), 1.2f);
    }

    for (std::size_t i = 0; i + 1 < loop.size(); ++i) {
        drawList->AddLine(toScreen(loop[i]), toScreen(loop[i + 1]), IM_COL32(95, 185, 255, 255), 2.0f);
    }

    ImGui::Dummy(size);
}

void InspectorPanel::renderTransformSection(Component& component) {
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
