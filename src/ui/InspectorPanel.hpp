#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include "core/Aircraft.hpp"
#include "ui/Panel.hpp"

namespace opads {

class Airfoil;
class Parameter;
class Wing;

/// Unified property editor for Aircraft, Transform, Wing bays, Cross Sections, and Airfoils.
class InspectorPanel : public Panel {
public:
    explicit InspectorPanel(Aircraft& aircraft);

    void onRender() override;

private:
    void renderAircraftProperties();
    void renderComponentProperties(Component& component);
    void renderWingEditor(Wing& wing);
    void renderWingPlanformTab(Wing& wing);
    void renderWingCrossSectionsTab(Wing& wing);
    void renderTransformSection(Component& component);
    bool renderAirfoilEditor(Airfoil& airfoil);
    void renderAirfoilPreview(const Airfoil& airfoil);
    void refreshDatLibrary();

    static bool editParameterDrag(Parameter& param);

    Aircraft& m_aircraft;
    std::size_t m_selectedBay{0};
    std::vector<std::filesystem::path> m_datLibrary;
    bool m_datLibraryScanned{false};
};

} // namespace opads
