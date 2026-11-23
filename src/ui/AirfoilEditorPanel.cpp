#include "ui/AirfoilEditorPanel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include <imgui.h>
#include <implot.h>

#include "core/Airfoil.hpp"
#include "core/ProjectSerializer.hpp"
#include "core/Wing.hpp"
#include "ui/AirfoilUiHelper.hpp"

namespace opads {

namespace {

bool editTextBuffer(const char* label, std::string& value, std::size_t maxLen = 128) {
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

double cstClassValue(double x) noexcept {
    return std::sqrt(std::max(x, 0.0)) * std::max(1.0 - x, 0.0);
}

} // namespace

AirfoilEditorPanel::AirfoilEditorPanel(Aircraft& aircraft, UndoStack& undoStack, GLFWwindow* window)
    : Panel("Airfoil Editor")
    , m_aircraft(aircraft)
    , m_undoStack(undoStack)
    , m_window(window) {
}

void AirfoilEditorPanel::onRender() {
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

    const auto selectedId = m_aircraft.getSelectedComponentId();
    Component* comp = selectedId.has_value() ? m_aircraft.findComponent(*selectedId) : nullptr;
    auto* wing = dynamic_cast<Wing*>(comp);

    if (wing == nullptr || wing->getSectionCount() == 0) {
        ImGui::TextDisabled("Select a Wing component to inspect, upload, or sculpt its cross-section airfoils.");
        ImGui::End();
        return;
    }

    const std::size_t xsecCount = wing->getCrossSectionCount();
    std::size_t xsecIndex = std::min(wing->getSelectedCrossSection(), xsecCount - 1);

    ImGui::Text("Target: %s  |  XSec %zu / %zu", wing->getName().c_str(), xsecIndex, xsecCount - 1);
    ImGui::SameLine();
    if (ImGui::Button("< Prev") && xsecIndex > 0) {
        --xsecIndex;
        wing->setSelectedCrossSection(xsecIndex);
    }
    ImGui::SameLine();
    if (ImGui::Button("Next >") && xsecIndex + 1 < xsecCount) {
        ++xsecIndex;
        wing->setSelectedCrossSection(xsecIndex);
    }

    Airfoil& airfoil = (xsecIndex == 0)
        ? wing->getSection(0).getRootAirfoil()
        : wing->getSection(xsecIndex - 1).getTipAirfoil();

    ImGui::Separator();

    const float controlsWidth = 300.0f;
    ImGui::BeginChild("##AirfoilControls", ImVec2(controlsWidth, 0.0f), true);
    renderControls(*wing, xsecIndex, airfoil);
    ImGui::EndChild();

    ImGui::SameLine();

    ImGui::BeginChild("##AirfoilPlotRegion", ImVec2(0.0f, 0.0f), false);
    renderPlot(*wing, xsecIndex, airfoil);
    ImGui::EndChild();

    const bool anyActive = ImGui::IsAnyItemActive() || ImGui::IsMouseDown(ImGuiMouseButton_Left);
    if (anyActive) {
        m_undoTrackingActive = true;
    } else {
        m_undoStack.pushSnapshot(m_pendingBeforeJson, m_aircraft, "Edit Airfoil");
        m_undoTrackingActive = false;
    }

    ImGui::End();
}

void AirfoilEditorPanel::renderControls(Wing& wing, std::size_t xsecIndex, Airfoil& airfoil) {
    bool changed = false;

    if (ImGui::CollapsingHeader("Airfoil Source", ImGuiTreeNodeFlags_DefaultOpen)) {
        int typeIdx = static_cast<int>(airfoil.getType());
        const char* types[] = {"NACA 4-Digit", "NACA 5-Digit", "Database / .dat", "CST (Sculpt)"};
        if (ImGui::Combo("Type", &typeIdx, types, 4)) {
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
            if (editTextBuffer("NACA 4", code, 8) && code.size() == 4) {
                airfoil.setNaca4(code);
                changed = true;
            }
        } else if (airfoil.getType() == AirfoilType::Naca5) {
            std::string code = airfoil.getNaca5Code();
            if (editTextBuffer("NACA 5", code, 8) && code.size() == 5) {
                airfoil.setNaca5(code);
                changed = true;
            }
        } else if (airfoil.getType() == AirfoilType::DatFile) {
            changed |= AirfoilUiHelper::renderSearchableDatCombo("Library", airfoil, m_datLibrary, m_searchFilter);
        } else {
            ImGui::TextDisabled("Drag points on the 2D plot to sculpt!");
            const auto& upW = airfoil.getCstUpperWeights();
            const auto& loW = airfoil.getCstLowerWeights();
            if (ImGui::TreeNode("CST Weights")) {
                for (std::size_t i = 0; i < upW.size(); ++i) {
                    float w = static_cast<float>(upW[i]);
                    const std::string lbl = "Upper w" + std::to_string(i);
                    if (ImGui::DragFloat(lbl.c_str(), &w, 0.005f, -1.0f, 1.0f, "%.4f")) {
                        airfoil.setCstUpperWeight(i, static_cast<double>(w));
                        changed = true;
                    }
                }
                for (std::size_t i = 0; i < loW.size(); ++i) {
                    float w = static_cast<float>(loW[i]);
                    const std::string lbl = "Lower w" + std::to_string(i);
                    if (ImGui::DragFloat(lbl.c_str(), &w, 0.005f, -1.0f, 1.0f, "%.4f")) {
                        airfoil.setCstLowerWeight(i, static_cast<double>(w));
                        changed = true;
                    }
                }
                ImGui::TreePop();
            }
        }

        if (ImGui::Button("Upload .dat File...")) {
            changed |= AirfoilUiHelper::uploadDatFile(m_window, airfoil, m_datLibrary, m_statusMessage);
        }
        ImGui::SameLine();
        if (ImGui::Button("Refresh")) {
            AirfoilUiHelper::refreshLibrary(m_datLibrary);
        }

        if (airfoil.getType() != AirfoilType::Cst) {
            if (ImGui::Button("Convert to Editable CST", ImVec2(-1.0f, 0.0f))) {
                airfoil.convertToCst(4);
                changed = true;
            }
        }
    }

    if (ImGui::CollapsingHeader("Shape Modifiers", ImGuiTreeNodeFlags_DefaultOpen)) {
        float thick = airfoil.getThicknessScale();
        if (ImGui::DragFloat("Thickness", &thick, 0.01f, 0.1f, 3.0f, "%.2f")) {
            changed |= airfoil.setThicknessScale(thick);
        }

        float camber = airfoil.getCamberScale();
        if (ImGui::DragFloat("Camber", &camber, 0.02f, -2.0f, 2.5f, "%.2f")) {
            changed |= airfoil.setCamberScale(camber);
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
    }

    if (ImGui::CollapsingHeader("Metrics & Export", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Show Camber Line", &m_showCamber);
        ImGui::SameLine();
        ImGui::Checkbox("Chord", &m_showChord);

        ImGui::Text("Name:       %s", airfoil.getName().c_str());
        ImGui::Text("Max t/c:    %.2f%%", airfoil.getMaxThicknessRatio() * 100.0f);
        ImGui::Text("Max Camber: %.2f%%", airfoil.getMaxCamberRatio() * 100.0f);
        ImGui::Text("Loop Pts:   %zu", airfoil.getClosedLoop().size());

        ImGui::Separator();
        editTextBuffer("Export File", m_exportFilename, 64);
        if (ImGui::Button("Save .dat to Library", ImVec2(-1.0f, 0.0f))) {
            const std::filesystem::path dir = AirfoilUiHelper::resolveAirfoilsDir();
            if (!dir.empty() && !m_exportFilename.empty()) {
                std::filesystem::path target = dir / m_exportFilename;
                if (target.extension() != ".dat") {
                    target.replace_extension(".dat");
                }
                if (airfoil.saveToDatFile(target)) {
                    m_statusMessage = "Saved: " + target.filename().string();
                    AirfoilUiHelper::refreshLibrary(m_datLibrary);
                } else {
                    m_statusMessage = "Failed to save .dat file.";
                }
            }
        }
        if (!m_statusMessage.empty()) {
            ImGui::TextWrapped("%s", m_statusMessage.c_str());
        }
    }

    if (changed) {
        wing.syncSectionsFromCrossSection(xsecIndex);
    }
}

void AirfoilEditorPanel::renderPlot(Wing& wing, std::size_t xsecIndex, Airfoil& airfoil) {
    const auto& upper = airfoil.getUpperSurface();
    const auto& lower = airfoil.getLowerSurface();
    const auto& camber = airfoil.getCamberLine();
    if (upper.empty() || lower.empty()) {
        return;
    }

    std::vector<double> upX(upper.size());
    std::vector<double> upY(upper.size());
    for (std::size_t i = 0; i < upper.size(); ++i) {
        upX[i] = static_cast<double>(upper[i].x);
        upY[i] = static_cast<double>(upper[i].y);
    }

    std::vector<double> loX(lower.size());
    std::vector<double> loY(lower.size());
    for (std::size_t i = 0; i < lower.size(); ++i) {
        loX[i] = static_cast<double>(lower[i].x);
        loY[i] = static_cast<double>(lower[i].y);
    }

    std::vector<double> camX(camber.size());
    std::vector<double> camY(camber.size());
    for (std::size_t i = 0; i < camber.size(); ++i) {
        camX[i] = static_cast<double>(camber[i].x);
        camY[i] = static_cast<double>(camber[i].y);
    }

    if (ImPlot::BeginPlot("##Airfoil2DPlot", ImVec2(-1.0f, -1.0f), ImPlotFlags_Equal)) {
        ImPlot::SetupAxes("x / c", "y / c");
        ImPlot::SetupAxisLimits(ImAxis_X1, -0.05, 1.05, ImPlotCond_Once);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -0.25, 0.25, ImPlotCond_Once);

        if (m_showChord) {
            const double chordX[2] = {0.0, 1.0};
            const double chordY[2] = {0.0, 0.0};
            ImPlot::SetNextLineStyle(ImVec4(0.45f, 0.50f, 0.58f, 0.8f), 1.0f);
            ImPlot::PlotLine("Chord", chordX, chordY, 2);
        }

        if (m_showCamber) {
            ImPlot::SetNextLineStyle(ImVec4(0.95f, 0.72f, 0.24f, 0.9f), 1.5f);
            ImPlot::PlotLine("Camber", camX.data(), camY.data(), static_cast<int>(camX.size()));
        }

        ImPlot::SetNextLineStyle(ImVec4(0.32f, 0.74f, 1.00f, 1.0f), 2.4f);
        ImPlot::PlotLine("Upper", upX.data(), upY.data(), static_cast<int>(upX.size()));

        ImPlot::SetNextLineStyle(ImVec4(0.42f, 0.88f, 0.58f, 1.0f), 2.4f);
        ImPlot::PlotLine("Lower", loX.data(), loY.data(), static_cast<int>(loX.size()));

        if (airfoil.getType() == AirfoilType::Cst) {
            bool cstChanged = false;
            const auto& upW = airfoil.getCstUpperWeights();
            const auto& loW = airfoil.getCstLowerWeights();
            const std::size_t n = upW.size();

            for (std::size_t i = 0; i < n; ++i) {
                double hx = (static_cast<double>(i) + 0.5) / static_cast<double>(n);
                const double classVal = std::max(cstClassValue(hx), 0.05);
                double hy = upW[i] * classVal;
                if (ImPlot::DragPoint(static_cast<int>(i), &hx, &hy, ImVec4(0.35f, 0.82f, 1.0f, 1.0f), 5.5f)) {
                    airfoil.setCstUpperWeight(i, hy / classVal);
                    cstChanged = true;
                }
            }

            for (std::size_t i = 0; i < loW.size(); ++i) {
                double hx = (static_cast<double>(i) + 0.5) / static_cast<double>(loW.size());
                const double classVal = std::max(cstClassValue(hx), 0.05);
                double hy = loW[i] * classVal;
                if (ImPlot::DragPoint(static_cast<int>(100 + i), &hx, &hy, ImVec4(1.0f, 0.55f, 0.30f, 1.0f), 5.5f)) {
                    airfoil.setCstLowerWeight(i, hy / classVal);
                    cstChanged = true;
                }
            }

            if (cstChanged) {
                wing.syncSectionsFromCrossSection(xsecIndex);
            }
        }

        ImPlot::EndPlot();
    }
}

} // namespace opads
