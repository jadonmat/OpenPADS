#include "ui/InspectorPanel.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <string>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include "core/Airfoil.hpp"
#include "core/Fuselage.hpp"
#include "core/ImportedMesh.hpp"
#include "core/LandingGear.hpp"
#include "core/Parameter.hpp"
#include "core/Pod.hpp"
#include "core/ProjectSerializer.hpp"
#include "core/Propeller.hpp"
#include "core/Wing.hpp"
#include "ui/AirfoilUiHelper.hpp"

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

} // namespace

InspectorPanel::InspectorPanel(Aircraft& aircraft, UndoStack& undoStack, GLFWwindow* window)
    : Panel("Inspector")
    , m_aircraft(aircraft)
    , m_undoStack(undoStack)
    , m_window(window) {
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
        AirfoilUiHelper::refreshLibrary(m_datLibrary);
        m_datLibraryScanned = true;
    }

    if (!m_undoTrackingActive) {
        m_pendingBeforeJson = ProjectSerializer::saveToString(m_aircraft);
    }

    ImGui::PushItemWidth(-125.0f);

    const auto selectedId = m_aircraft.getSelectedComponentId();
    Component* selected = selectedId.has_value() ? m_aircraft.findComponent(*selectedId) : nullptr;
    if (selected != nullptr) {
        renderComponentProperties(*selected);
    } else {
        renderAircraftProperties();
    }

    ImGui::PopItemWidth();

    const bool anyActive = ImGui::IsAnyItemActive();
    if (anyActive) {
        m_undoTrackingActive = true;
    } else {
        m_undoStack.pushSnapshot(m_pendingBeforeJson, m_aircraft, "Edit Properties");
        m_undoTrackingActive = false;
    }

    ImGui::End();
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
    ImGui::TextUnformatted("Aircraft Summary");
    ImGui::Separator();

    std::string name = m_aircraft.getName();
    if (editStringField("Name", name)) {
        m_aircraft.setName(std::move(name));
    }

    ImGui::Spacing();
    ImGui::Text("Components: %zu", m_aircraft.getComponents().size());
    ImGui::TextDisabled("Coordinate System: Aerospace Z-Up (X aft, Y stbd, Z up)");
    ImGui::Spacing();
    ImGui::TextDisabled("Tip: Use Model Tree (+ Wing / + Fuselage) or click parts in 3D.");
}

void InspectorPanel::renderComponentProperties(Component& component) {
    ImGui::Text("%s", component.getName().c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("(%s)", component.getTypeName().c_str());
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
    } else if (auto* fuse = dynamic_cast<Fuselage*>(&component)) {
        ImGui::Spacing();
        renderFuselageEditor(*fuse);
    } else if (auto* pod = dynamic_cast<Pod*>(&component)) {
        ImGui::Spacing();
        renderPodEditor(*pod);
    } else if (auto* prop = dynamic_cast<Propeller*>(&component)) {
        ImGui::Spacing();
        renderPropellerEditor(*prop);
    } else if (auto* gear = dynamic_cast<LandingGear*>(&component)) {
        ImGui::Spacing();
        renderLandingGearEditor(*gear);
    } else if (auto* imp = dynamic_cast<ImportedMesh*>(&component)) {
        ImGui::Spacing();
        renderImportedMeshEditor(*imp);
    } else {
        ImGui::Spacing();
        renderTransformSection(component);
    }
}

void InspectorPanel::renderWingEditor(Wing& wing) {
    if (ImGui::BeginTable("##WingMetrics", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Area S:  %.3f m^2", wing.getPlanformArea());
        ImGui::Text("Span b:  %.3f m", wing.getSpan());
        ImGui::Text("AR:      %.2f", wing.getAspectRatio());
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("MAC:     %.3f m", wing.getMeanAerodynamicChord());
        ImGui::Text("Wetted:  %.3f m^2", wing.getWettedArea());
        ImGui::Text("Bays:    %zu", wing.getSectionCount());
        ImGui::EndTable();
    }

    ImGui::Spacing();
    if (ImGui::BeginTabBar("##WingTabs")) {
        if (ImGui::BeginTabItem("Planform")) {
            renderWingPlanformTab(wing);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Cross Sections")) {
            renderWingCrossSectionsTab(wing);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Controls")) {
            renderWingControlsTab(wing);
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
    const char* symItems[] = {"None (Single Side)", "XZ Mirror (Left + Right)"};
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
    if (ImGui::BeginListBox("##BayList", ImVec2(-1.0f, 78.0f))) {
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
    if (ImGui::SliderInt("Span Subdiv", &spanSub, 1, 32)) {
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

    if (ImGui::Button("< Prev XSec") && selectedXSec > 0) {
        --selectedXSec;
        wing.setSelectedCrossSection(selectedXSec);
    }
    ImGui::SameLine();
    if (ImGui::Button("Next XSec >") && selectedXSec + 1 < xsecCount) {
        ++selectedXSec;
        wing.setSelectedCrossSection(selectedXSec);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(%zu / %zu)", selectedXSec, xsecCount - 1);

    if (ImGui::BeginListBox("##XSecList", ImVec2(-1.0f, 82.0f))) {
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

void InspectorPanel::renderWingControlsTab(Wing& wing) {
    if (ImGui::Button("+ Aileron")) {
        wing.addControlSurface(ControlSurface("Aileron", 0, 0.55, 0.92, 0.25, 0.0, false));
    }
    ImGui::SameLine();
    if (ImGui::Button("+ Flap")) {
        wing.addControlSurface(ControlSurface("Flap", 0, 0.08, 0.50, 0.28, 0.0, true));
    }

    if (wing.getControlSurfaceCount() == 0) {
        ImGui::TextDisabled("No trailing-edge control surfaces yet.");
        return;
    }

    std::optional<std::size_t> removeIdx;
    for (std::size_t i = 0; i < wing.getControlSurfaceCount(); ++i) {
        ControlSurface& cs = wing.getControlSurface(i);
        ImGui::PushID(static_cast<int>(i));
        if (ImGui::CollapsingHeader(cs.getName().c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
            std::string name = cs.getName();
            if (editStringField("Label", name, 48)) {
                cs.setName(std::move(name));
            }
            int bayIdx = static_cast<int>(cs.getBayIndex());
            if (ImGui::SliderInt("Bay Index", &bayIdx, 0, static_cast<int>(wing.getSectionCount() - 1))) {
                cs.setBayIndex(static_cast<std::size_t>(bayIdx));
                wing.markDirty();
            }
            bool changed = false;
            changed |= editParameterDrag(cs.spanStartFrac());
            changed |= editParameterDrag(cs.spanEndFrac());
            changed |= editParameterDrag(cs.chordFrac());
            changed |= editParameterDrag(cs.deflectionDeg());
            bool sym = cs.isSymmetric();
            if (ImGui::Checkbox("Symmetric Deflection", &sym)) {
                cs.setSymmetric(sym);
                changed = true;
            }
            if (changed) {
                wing.markDirty();
            }
            if (ImGui::Button("Remove Surface")) {
                removeIdx = i;
            }
        }
        ImGui::PopID();
    }
    if (removeIdx.has_value()) {
        wing.removeControlSurface(*removeIdx);
    }
}

void InspectorPanel::renderFuselageEditor(Fuselage& fuselage) {
    if (ImGui::BeginTable("##FuseMetrics", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Length:   %.2f m", fuselage.length().getValue());
        ImGui::Text("Max W:    %.2f m", fuselage.getMaxWidth());
        ImGui::Text("Max H:    %.2f m", fuselage.getMaxHeight());
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("Fineness: %.2f", fuselage.getFinenessRatio());
        ImGui::Text("Wetted:   %.2f m^2", fuselage.getWettedArea());
        ImGui::Text("Volume:   %.2f m^3", fuselage.getVolume());
        ImGui::EndTable();
    }

    ImGui::Spacing();
    if (ImGui::BeginTabBar("##FuseTabs")) {
        if (ImGui::BeginTabItem("Loft")) {
            renderFuselageLoftTab(fuselage);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Cross Sections")) {
            renderFuselageSectionsTab(fuselage);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Transform")) {
            renderTransformSection(fuselage);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

void InspectorPanel::renderFuselageLoftTab(Fuselage& fuselage) {
    if (editParameterDrag(fuselage.length())) {
        fuselage.markDirty();
    }

    bool smoothLoft = fuselage.isSmoothLoft();
    if (ImGui::Checkbox("Smooth Loft (PCHIP)", &smoothLoft)) {
        fuselage.setSmoothLoft(smoothLoft);
    }

    bool capEnds = fuselage.getCapEnds();
    if (ImGui::Checkbox("Cap Open Ends", &capEnds)) {
        fuselage.setCapEnds(capEnds);
    }

    int radialPts = fuselage.getRadialPoints();
    if (ImGui::SliderInt("Radial Pts", &radialPts, 12, 72)) {
        fuselage.setRadialPoints(radialPts);
    }

    int axialSub = fuselage.getAxialSubdivisions();
    if (ImGui::SliderInt("Axial Subdiv", &axialSub, 1, 24)) {
        fuselage.setAxialSubdivisions(axialSub);
    }
}

void InspectorPanel::renderFuselageSectionsTab(Fuselage& fuselage) {
    if (fuselage.getSectionCount() == 0) {
        return;
    }

    std::size_t selectedIdx = std::min(fuselage.getSelectedSection(), fuselage.getSectionCount() - 1);
    if (ImGui::Button("+ Insert")) {
        fuselage.addSectionAfter(selectedIdx);
        selectedIdx = fuselage.getSelectedSection();
    }
    ImGui::SameLine();
    const bool canRemove = fuselage.getSectionCount() > 2;
    if (!canRemove) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("- Remove")) {
        fuselage.removeSection(selectedIdx);
        selectedIdx = fuselage.getSelectedSection();
    }
    if (!canRemove) {
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    if (ImGui::Button("<") && selectedIdx > 0) {
        --selectedIdx;
        fuselage.setSelectedSection(selectedIdx);
    }
    ImGui::SameLine();
    if (ImGui::Button(">") && selectedIdx + 1 < fuselage.getSectionCount()) {
        ++selectedIdx;
        fuselage.setSelectedSection(selectedIdx);
    }

    const char* shapeNames[] = {"Point", "Circle", "Ellipse", "SuperEllipse", "RoundedRect"};

    if (ImGui::BeginListBox("##FuseXSecList", ImVec2(-1.0f, 90.0f))) {
        for (std::size_t i = 0; i < fuselage.getSectionCount(); ++i) {
            const FuselageSection& sec = fuselage.getSection(i);
            const int sIdx = static_cast<int>(sec.getShape());
            const std::string label =
                "XSec " + std::to_string(i) +
                " [" + shapeNames[sIdx] + "]  u=" +
                std::to_string(sec.stationFrac().getFloat()).substr(0, 4);
            if (ImGui::Selectable(label.c_str(), i == selectedIdx)) {
                selectedIdx = i;
                fuselage.setSelectedSection(i);
            }
        }
        ImGui::EndListBox();
    }

    FuselageSection& sec = fuselage.getSection(selectedIdx);
    bool changed = false;

    int shapeIdx = static_cast<int>(sec.getShape());
    if (ImGui::Combo("Shape", &shapeIdx, shapeNames, 5)) {
        sec.setShape(static_cast<CrossSectionShape>(shapeIdx));
        changed = true;
    }

    changed |= editParameterDrag(sec.stationFrac());
    if (sec.getShape() != CrossSectionShape::Point) {
        changed |= editParameterDrag(sec.width());
        if (sec.getShape() != CrossSectionShape::Circle) {
            changed |= editParameterDrag(sec.height());
        }
    }
    changed |= editParameterDrag(sec.yOffset());
    changed |= editParameterDrag(sec.zOffset());

    if (sec.getShape() == CrossSectionShape::SuperEllipse) {
        changed |= editParameterDrag(sec.superEllipseN());
    } else if (sec.getShape() == CrossSectionShape::RoundedRect) {
        changed |= editParameterDrag(sec.cornerRadiusFrac());
    }

    if (changed) {
        fuselage.markDirty();
    }

    ImGui::Spacing();
    renderFuselageSectionPreview(sec);
}

void InspectorPanel::renderFuselageSectionPreview(const FuselageSection& section) {
    const auto loop = section.sampleLoopYZ(48);
    if (loop.size() < 3) {
        return;
    }

    const ImVec2 size(ImGui::GetContentRegionAvail().x, 115.0f);
    if (size.x < 40.0f) {
        return;
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(22, 25, 31, 255), 4.0f);
    drawList->AddRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(60, 68, 82, 255), 4.0f);

    float maxExtent = 0.5f;
    for (const glm::vec2& pt : loop) {
        maxExtent = std::max(maxExtent, std::abs(pt.x - section.yOffset().getFloat()));
        maxExtent = std::max(maxExtent, std::abs(pt.y - section.zOffset().getFloat()));
    }
    const float scale = (std::min(size.x, size.y) * 0.38f) / maxExtent;
    const float cx = origin.x + size.x * 0.5f;
    const float cy = origin.y + size.y * 0.5f;

    drawList->AddLine(ImVec2(origin.x + 8.0f, cy), ImVec2(origin.x + size.x - 8.0f, cy), IM_COL32(65, 75, 88, 160), 1.0f);
    drawList->AddLine(ImVec2(cx, origin.y + 8.0f), ImVec2(cx, origin.y + size.y - 8.0f), IM_COL32(65, 75, 88, 160), 1.0f);

    const float yc = section.yOffset().getFloat();
    const float zc = section.zOffset().getFloat();
    const auto toScreen = [&](const glm::vec2& p) -> ImVec2 {
        return ImVec2(cx + (p.x - yc) * scale, cy - (p.y - zc) * scale);
    };

    for (std::size_t i = 0; i + 1 < loop.size(); ++i) {
        drawList->AddLine(toScreen(loop[i]), toScreen(loop[i + 1]), IM_COL32(95, 200, 255, 255), 2.0f);
    }

    ImGui::Dummy(size);
}

void InspectorPanel::renderPodEditor(Pod& pod) {
    ImGui::Text("Fineness: %.2f  |  Wetted: %.2f m^2  |  Vol: %.2f m^3",
        pod.getFinenessRatio(), pod.getWettedArea(), pod.getVolume());

    if (ImGui::BeginTabBar("##PodTabs")) {
        if (ImGui::BeginTabItem("Geometry")) {
            int symIdx = (pod.getSymmetry() == SymmetryMode::XZ) ? 1 : 0;
            const char* symItems[] = {"None", "XZ Mirror"};
            if (ImGui::Combo("Symmetry", &symIdx, symItems, 2)) {
                pod.setSymmetry(symIdx == 1 ? SymmetryMode::XZ : SymmetryMode::None);
            }
            bool changed = false;
            changed |= editParameterDrag(pod.length());
            changed |= editParameterDrag(pod.width());
            changed |= editParameterDrag(pod.height());
            changed |= editParameterDrag(pod.noseFrac());
            changed |= editParameterDrag(pod.tailFrac());
            changed |= editParameterDrag(pod.nosePower());
            changed |= editParameterDrag(pod.tailPower());
            changed |= editParameterDrag(pod.superEllipseN());
            if (changed) {
                pod.markDirty();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Transform")) {
            renderTransformSection(pod);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

void InspectorPanel::renderPropellerEditor(Propeller& prop) {
    ImGui::Text("Disk Area: %.2f m^2  |  Solidity: %.3f", prop.getDiskArea(), prop.getSolidity());

    if (ImGui::BeginTabBar("##PropTabs")) {
        if (ImGui::BeginTabItem("Rotor & Blades")) {
            int blades = prop.getBladeCount();
            if (ImGui::SliderInt("Blades", &blades, 2, 8)) {
                prop.setBladeCount(blades);
            }
            bool cw = prop.isClockwise();
            if (ImGui::Checkbox("Clockwise", &cw)) {
                prop.setClockwise(cw);
            }
            ImGui::SameLine();
            bool disk = prop.getShowDiskRing();
            if (ImGui::Checkbox("Disk Ring", &disk)) {
                prop.setShowDiskRing(disk);
            }
            ImGui::SameLine();
            bool spinner = prop.hasSpinner();
            if (ImGui::Checkbox("Spinner", &spinner)) {
                prop.setHasSpinner(spinner);
            }

            bool changed = false;
            changed |= editParameterDrag(prop.diameter());
            changed |= editParameterDrag(prop.hubDiameter());
            changed |= editParameterDrag(prop.spinnerLength());
            if (!prop.getBladeCurve().empty()) {
                ImGui::TextDisabled("Imported radial blade curve (%zu stations)", prop.getBladeCurve().size());
                ImGui::SameLine();
                if (ImGui::SmallButton("Use Linear")) {
                    prop.clearBladeCurve();
                }
            }
            bool linearEdited = false;
            linearEdited |= editParameterDrag(prop.rootChord());
            linearEdited |= editParameterDrag(prop.tipChord());
            linearEdited |= editParameterDrag(prop.collectivePitchDeg());
            linearEdited |= editParameterDrag(prop.washoutDeg());
            linearEdited |= editParameterDrag(prop.sweepDeg());
            if (linearEdited) {
                prop.clearBladeCurve();
                changed = true;
            }
            if (changed) {
                prop.markDirty();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Blade Airfoil")) {
            if (renderAirfoilEditor(prop.getBladeAirfoil())) {
                prop.markDirty();
            }
            renderAirfoilPreview(prop.getBladeAirfoil());
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Transform")) {
            renderTransformSection(prop);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

void InspectorPanel::renderLandingGearEditor(LandingGear& gear) {
    ImGui::Text("Frontal Area: %.3f m^2  |  Wetted: %.2f m^2",
        gear.getFrontalArea(), gear.getWettedArea());

    if (ImGui::BeginTabBar("##GearTabs")) {
        if (ImGui::BeginTabItem("Gear Geometry")) {
            int symIdx = (gear.getSymmetry() == SymmetryMode::XZ) ? 1 : 0;
            const char* symItems[] = {"Single (Nose/Tail)", "XZ Mirror (Main Gear)"};
            if (ImGui::Combo("Symmetry", &symIdx, symItems, 2)) {
                gear.setSymmetry(symIdx == 1 ? SymmetryMode::XZ : SymmetryMode::None);
            }
            int wheels = gear.getWheelsPerStrut();
            if (ImGui::SliderInt("Wheels / Strut", &wheels, 1, 2)) {
                gear.setWheelsPerStrut(wheels);
            }
            bool pant = gear.hasWheelPant();
            if (ImGui::Checkbox("Streamlined Wheel Pant", &pant)) {
                gear.setWheelPant(pant);
            }

            bool changed = false;
            changed |= editParameterDrag(gear.strutLength());
            changed |= editParameterDrag(gear.strutDiameter());
            changed |= editParameterDrag(gear.rakeDeg());
            changed |= editParameterDrag(gear.splayDeg());
            changed |= editParameterDrag(gear.wheelDiameter());
            changed |= editParameterDrag(gear.wheelWidth());
            if (changed) {
                gear.markDirty();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Transform")) {
            renderTransformSection(gear);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

void InspectorPanel::renderImportedMeshEditor(ImportedMesh& meshComp) {
    ImGui::Text("Wetted Area: %.2f m^2", meshComp.getWettedArea());
    if (!meshComp.getSourcePath().empty()) {
        ImGui::TextWrapped("Source: %s", meshComp.getSourcePath().c_str());
    }

    if (ImGui::BeginTabBar("##ImportedMeshTabs")) {
        if (ImGui::BeginTabItem("Conversion")) {
            float unitScale = meshComp.getUnitScale();
            if (ImGui::Button("Meters (1.0)")) meshComp.setUnitScale(1.0f);
            ImGui::SameLine();
            if (ImGui::Button("mm (0.001)")) meshComp.setUnitScale(0.001f);
            ImGui::SameLine();
            if (ImGui::Button("in (0.0254)")) meshComp.setUnitScale(0.0254f);
            ImGui::SameLine();
            if (ImGui::Button("ft (0.3048)")) meshComp.setUnitScale(0.3048f);

            unitScale = meshComp.getUnitScale();
            if (ImGui::DragFloat("Unit Scale", &unitScale, 0.001f, 0.0001f, 100.0f, "%.5f")) {
                meshComp.setUnitScale(unitScale);
            }

            int axisIdx = (meshComp.getAxisConversion() == UpAxisConversion::YUpToZUp) ? 1 : 0;
            const char* axisItems[] = {"Keep Z-Up", "Convert Y-Up -> Z-Up"};
            if (ImGui::Combo("Up Axis", &axisIdx, axisItems, 2)) {
                meshComp.setAxisConversion(axisIdx == 1 ? UpAxisConversion::YUpToZUp : UpAxisConversion::NoneZUp);
            }

            bool centerOrigin = meshComp.isCenterOrigin();
            if (ImGui::Checkbox("Center Origin to Bounds", &centerOrigin)) {
                meshComp.setCenterOrigin(centerOrigin);
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Transform")) {
            renderTransformSection(meshComp);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

bool InspectorPanel::renderAirfoilEditor(Airfoil& airfoil) {
    bool changed = false;

    int typeIdx = static_cast<int>(airfoil.getType());
    const char* types[] = {"NACA 4-Digit", "NACA 5-Digit", "Database / .dat", "CST (Sculpt)"};
    if (ImGui::Combo("Airfoil Type", &typeIdx, types, 4)) {
        if (typeIdx == 0) {
            airfoil.setNaca4(airfoil.getNaca4Code());
            changed = true;
        } else if (typeIdx == 1) {
            airfoil.setNaca5(airfoil.getNaca5Code());
            changed = true;
        } else if (typeIdx == 2 && !m_datLibrary.empty()) {
            changed |= airfoil.loadFromDatFile(m_datLibrary.front());
        } else if (typeIdx == 3) {
            airfoil.convertToCst(4);
            changed = true;
        }
    }

    if (airfoil.getType() == AirfoilType::Naca4) {
        std::string code = airfoil.getNaca4Code();
        if (editStringField("NACA 4 Code", code, 8) && code.size() == 4) {
            airfoil.setNaca4(code);
            changed = true;
        }
    } else if (airfoil.getType() == AirfoilType::Naca5) {
        std::string code = airfoil.getNaca5Code();
        if (editStringField("NACA 5 Code", code, 8) && code.size() == 5) {
            airfoil.setNaca5(code);
            changed = true;
        }
    } else if (airfoil.getType() == AirfoilType::Cst) {
        ImGui::TextDisabled("Drag CST points in the Airfoil Editor tab.");
    } else {
        changed |= AirfoilUiHelper::renderSearchableDatCombo("Library", airfoil, m_datLibrary, m_airfoilSearchFilter);
    }

    if (ImGui::Button("Upload .dat File...")) {
        changed |= AirfoilUiHelper::uploadDatFile(m_window, airfoil, m_datLibrary, m_uploadStatus);
    }
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) {
        AirfoilUiHelper::refreshLibrary(m_datLibrary);
    }
    if (!m_uploadStatus.empty()) {
        ImGui::TextDisabled("%s", m_uploadStatus.c_str());
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

    if (airfoil.getType() != AirfoilType::DatFile && airfoil.getType() != AirfoilType::Cst) {
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

    const ImVec2 size(ImGui::GetContentRegionAvail().x, 110.0f);
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
