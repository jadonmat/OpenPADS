#pragma once

#include "core/Aircraft.hpp"
#include "ui/Panel.hpp"

namespace opads {

/// Property editor for the selected Aircraft or Component.
class InspectorPanel : public Panel {
public:
    explicit InspectorPanel(Aircraft& aircraft);

    void onRender() override;

private:
    void renderAircraftProperties();
    void renderComponentProperties(Component& component);

    Aircraft& m_aircraft;
};

} // namespace opads
