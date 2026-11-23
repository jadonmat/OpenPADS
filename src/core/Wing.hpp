#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/Component.hpp"
#include "core/WingSection.hpp"

namespace opads {

enum class SymmetryMode {
    None,
    XZ
};

/// Parametric multi-section lofted wing component.
class Wing : public Component {
public:
    Wing(std::uint64_t id, std::string name = "Main Wing");

    std::string getTypeName() const override;
    std::vector<std::vector<glm::vec3>> getHighlightLoops() const override;

    SymmetryMode getSymmetry() const noexcept;
    void setSymmetry(SymmetryMode mode);

    bool getCapTips() const noexcept;
    void setCapTips(bool cap);

    int getChordPoints() const noexcept;
    void setChordPoints(int pointsPerSurface);

    /// Number of spanwise bays (cross-section count = getSectionCount() + 1).
    std::size_t getSectionCount() const noexcept;
    std::size_t getCrossSectionCount() const noexcept;

    WingSection& getSection(std::size_t index);
    const WingSection& getSection(std::size_t index) const;
    const std::vector<WingSection>& getSections() const noexcept;

    /// Appends or splits a bay and marks the wing dirty.
    void addSection();
    bool removeSection(std::size_t index);

    /// Synchronizes chained root/tip chord, twist, and airfoil after editing a bay or cross-section.
    void syncSectionsFromBay(std::size_t bayIndex);
    void syncSectionsFromCrossSection(std::size_t xsecIndex);

    /// Accesses cross-section index (0 = root, i+1 = tip of bay i).
    std::size_t getSelectedCrossSection() const noexcept;
    void setSelectedCrossSection(std::size_t xsecIndex) noexcept;
    glm::vec3 getCrossSectionLeadingEdge(std::size_t xsecIndex) const noexcept;
    /// Updates span, sweep, and dihedral of bay (xsecIndex - 1) from a new local tip LE position.
    void moveCrossSectionLeadingEdge(std::size_t xsecIndex, const glm::vec3& newLocalTipLE);

    /// Geometric & aerodynamic reference metrics (including symmetry when XZ is active).
    double getPlanformArea() const noexcept;
    double getSpan() const noexcept;
    double getAspectRatio() const noexcept;
    double getMeanAerodynamicChord() const noexcept;
    double getWettedArea() const noexcept;

protected:
    void rebuildMesh() override;

private:
    std::vector<WingSection> m_sections;
    SymmetryMode m_symmetry{SymmetryMode::XZ};
    bool m_capTips{true};
    int m_chordPoints{41};
    std::size_t m_selectedCrossSection{0};

    std::vector<std::vector<glm::vec3>> m_crossSectionRings;
    double m_wettedArea{0.0};
};

} // namespace opads
