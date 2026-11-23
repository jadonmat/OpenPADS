#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/Component.hpp"

namespace opads {

enum class UpAxisConversion {
    NoneZUp,
    YUpToZUp
};

/// Imported triangle mesh component supporting automatic unit scaling, axis conversion, and re-centering.
class ImportedMesh : public Component {
public:
    ImportedMesh(
        std::uint64_t id,
        std::string name,
        std::vector<glm::vec3> rawPositions,
        std::vector<std::uint32_t> rawIndices,
        std::string sourcePath = ""
    );

    std::string getTypeName() const override;

    const std::string& getSourcePath() const noexcept;

    float getUnitScale() const noexcept;
    void setUnitScale(float scale);

    UpAxisConversion getAxisConversion() const noexcept;
    void setAxisConversion(UpAxisConversion mode);

    bool isCenterOrigin() const noexcept;
    void setCenterOrigin(bool center);

    const std::vector<glm::vec3>& getRawPositions() const noexcept;
    const std::vector<std::uint32_t>& getRawIndices() const noexcept;

    double getWettedArea() const noexcept;

protected:
    void rebuildMesh() override;

private:
    std::vector<glm::vec3> m_rawPositions;
    std::vector<std::uint32_t> m_rawIndices;
    std::string m_sourcePath;
    float m_unitScale{1.0f};
    UpAxisConversion m_axisConversion{UpAxisConversion::NoneZUp};
    bool m_centerOrigin{false};
    double m_wettedArea{0.0};
};

} // namespace opads
