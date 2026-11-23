#pragma once

#include <glm/vec2.hpp>

#include "core/Aircraft.hpp"
#include "core/UndoStack.hpp"
#include "render/Viewport3D.hpp"
#include "ui/Panel.hpp"

namespace opads {

/// Dockable 3D CAD viewport panel hosting Viewport3D, picking, gizmos, and camera controls.
class ViewportPanel : public Panel {
public:
    ViewportPanel(Aircraft& aircraft, Viewport3D& viewport, UndoStack& undoStack);

    void onRender() override;

private:
    void renderToolbar();
    void renderOverlayHud(const glm::vec2& imageMin, const glm::vec2& imageSize);
    void handleViewportInput(bool imageHovered, const glm::vec2& imageMin, const glm::vec2& imageSize);

    Aircraft& m_aircraft;
    Viewport3D& m_viewport;
    UndoStack& m_undoStack;
};

} // namespace opads
