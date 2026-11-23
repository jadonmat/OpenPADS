#pragma once

#include "core/Aircraft.hpp"
#include "ui/Panel.hpp"

namespace opads {

/// Hierarchy outliner showing the Aircraft root and its parametric components.
class ModelTreePanel : public Panel {
public:
    explicit ModelTreePanel(Aircraft& aircraft);

    void onRender() override;

private:
    Aircraft& m_aircraft;
};

} // namespace opads
