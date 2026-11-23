#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "core/Aircraft.hpp"
#include "core/UndoStack.hpp"
#include "ui/Panel.hpp"

struct GLFWwindow;

namespace opads {

class Airfoil;
class Wing;

/// Dedicated dockable 2D airfoil plot, database browser, file uploader, and CST sculptor.
class AirfoilEditorPanel : public Panel {
public:
    AirfoilEditorPanel(Aircraft& aircraft, UndoStack& undoStack, GLFWwindow* window);

    void onRender() override;

private:
    void renderControls(Wing& wing, std::size_t xsecIndex, Airfoil& airfoil);
    void renderPlot(Wing& wing, std::size_t xsecIndex, Airfoil& airfoil);

    Aircraft& m_aircraft;
    UndoStack& m_undoStack;
    GLFWwindow* m_window{nullptr};

    std::vector<std::filesystem::path> m_datLibrary;
    bool m_datLibraryScanned{false};
    std::string m_searchFilter;
    bool m_showCamber{true};
    bool m_showChord{true};
    std::string m_exportFilename{"custom_airfoil.dat"};
    std::string m_statusMessage;

    bool m_undoTrackingActive{false};
    std::string m_pendingBeforeJson;
};

} // namespace opads
