#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "core/Aircraft.hpp"
#include "core/UndoStack.hpp"
#include "ui/Panel.hpp"

struct GLFWwindow;

namespace opads {

class Airfoil;
class Fuselage;
class FuselageSection;
class ImportedMesh;
class LandingGear;
class Parameter;
class Pod;
class Propeller;
class Wing;

/// Unified property editor for all Aircraft components, Cross Sections, and Airfoils.
class InspectorPanel : public Panel {
public:
    InspectorPanel(Aircraft& aircraft, UndoStack& undoStack, GLFWwindow* window);

    void onRender() override;

private:
    void renderAircraftProperties();
    void renderComponentProperties(Component& component);
    void renderWingEditor(Wing& wing);
    void renderWingPlanformTab(Wing& wing);
    void renderWingCrossSectionsTab(Wing& wing);
    void renderWingControlsTab(Wing& wing);
    void renderFuselageEditor(Fuselage& fuselage);
    void renderFuselageLoftTab(Fuselage& fuselage);
    void renderFuselageSectionsTab(Fuselage& fuselage);
    void renderFuselageSectionPreview(const FuselageSection& section);
    void renderPodEditor(Pod& pod);
    void renderPropellerEditor(Propeller& prop);
    void renderLandingGearEditor(LandingGear& gear);
    void renderImportedMeshEditor(ImportedMesh& meshComp);
    void renderTransformSection(Component& component);
    bool renderAirfoilEditor(Airfoil& airfoil);
    void renderAirfoilPreview(const Airfoil& airfoil);

    static bool editParameterDrag(Parameter& param);

    Aircraft& m_aircraft;
    UndoStack& m_undoStack;
    GLFWwindow* m_window{nullptr};

    std::size_t m_selectedBay{0};
    std::vector<std::filesystem::path> m_datLibrary;
    bool m_datLibraryScanned{false};
    std::string m_airfoilSearchFilter;
    std::string m_uploadStatus;

    bool m_undoTrackingActive{false};
    std::string m_pendingBeforeJson;
};

} // namespace opads
