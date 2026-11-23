#pragma once

#include "core/Aircraft.hpp"
#include "core/UndoStack.hpp"
#include "ui/Panel.hpp"

namespace opads {

/// Hierarchy outliner for adding, duplicating, removing, and selecting components.
class ModelTreePanel : public Panel {
public:
    ModelTreePanel(Aircraft& aircraft, UndoStack& undoStack);

    void onRender() override;

private:
    Aircraft& m_aircraft;
    UndoStack& m_undoStack;
};

} // namespace opads
