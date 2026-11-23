#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "core/Aircraft.hpp"
#include "ui/Panel.hpp"

namespace opads {

class Airfoil;
class Wing;

/// Dedicated dockable 2D airfoil plot, database browser, and interactive CST sculptor.
class AirfoilEditorPanel : public Panel {
public:
    explicit AirfoilEditorPanel(Aircraft& aircraft);

    void onRender() override;

private:
    void renderControls(Wing& wing, std::size_t xsecIndex, Airfoil& airfoil);
    void renderPlot(Wing& wing, std::size_t xsecIndex, Airfoil& airfoil);
    void refreshDatLibrary();

    Aircraft& m_aircraft;
    std::vector<std::filesystem::path> m_datLibrary;
    bool m_datLibraryScanned{false};
    bool m_showCamber{true};
    bool m_showChord{true};
    std::string m_exportFilename{"custom_airfoil.dat"};
    std::string m_statusMessage;
};

} // namespace opads
