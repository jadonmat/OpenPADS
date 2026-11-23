#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/Component.hpp"
#include "core/FuselageSection.hpp"
#include "core/Parameter.hpp"

namespace opads {

/// Parametric multi-station lofted fuselage component along the X axis.
class Fuselage : public Component {
public:
    Fuselage(std::uint64_t id, std::string name = "Fuselage");

    std::string getTypeName() const override;
    std::vector<std::vector<glm::vec3>> getHighlightLoops() const override;

    Parameter& length() noexcept;
    const Parameter& length() const noexcept;

    bool isSmoothLoft() const noexcept;
    void setSmoothLoft(bool smooth);

    bool getCapEnds() const noexcept;
    void setCapEnds(bool cap);

    int getRadialPoints() const noexcept;
    void setRadialPoints(int count);

    int getAxialSubdivisions() const noexcept;
    void setAxialSubdivisions(int count);

    std::size_t getSectionCount() const noexcept;
    FuselageSection& getSection(std::size_t index);
    const FuselageSection& getSection(std::size_t index) const;
    const std::vector<FuselageSection>& getSections() const noexcept;

    /// Inserts a new interpolated cross-section after index (or appends if at end).
    void addSectionAfter(std::size_t index);
    bool removeSection(std::size_t index);
    void clearSections();
    void appendSection(FuselageSection section);

    /// Sorts sections by stationFrac and marks dirty.
    void sortAndRebuild();

    std::size_t getSelectedSection() const noexcept;
    void setSelectedSection(std::size_t index) noexcept;

    glm::vec3 getSectionCenterLocal(std::size_t index) const noexcept;
    void setSectionCenterLocal(std::size_t index, const glm::vec3& localCenter);
    void moveSectionFromHandle(std::size_t index, const glm::vec3& deltaLocal);

    double getMaxWidth() const noexcept;
    double getMaxHeight() const noexcept;
    double getFinenessRatio() const noexcept;
    double getWettedArea() const noexcept;
    double getVolume() const noexcept;

protected:
    void rebuildMesh() override;

private:
    Parameter m_length;
    bool m_smoothLoft{true};
    bool m_capEnds{true};
    int m_radialPoints{36};
    int m_axialSubdivisions{8};
    std::size_t m_selectedSection{1};

    std::vector<FuselageSection> m_sections;
    std::vector<std::vector<glm::vec3>> m_stationRings;
    double m_wettedArea{0.0};
    double m_volume{0.0};
};

} // namespace opads
