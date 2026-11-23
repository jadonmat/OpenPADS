#include "core/Pod.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "core/MeshPrimitives.hpp"

namespace opads {

namespace {

float signedPow(float value, float exponent) noexcept {
    if (std::abs(value) < 1e-7f) {
        return 0.0f;
    }
    const float mag = std::pow(std::abs(value), exponent);
    return (value >= 0.0f) ? mag : -mag;
}

} // namespace

Pod::Pod(std::uint64_t id, std::string name)
    : Component(id, std::move(name))
    , m_length("Length", "m", 3.5, 0.1, 60.0, 0.1)
    , m_width("Width", "m", 0.85, 0.02, 20.0, 0.02)
    , m_height("Height", "m", 0.85, 0.02, 20.0, 0.02)
    , m_noseFrac("Nose Frac", "L", 0.28, 0.05, 0.80, 0.02)
    , m_tailFrac("Tail Frac", "L", 0.32, 0.05, 0.80, 0.02)
    , m_nosePower("Nose Sharpness", "", 0.55, 0.2, 1.5, 0.05)
    , m_tailPower("Tail Sharpness", "", 0.65, 0.2, 1.5, 0.05)
    , m_superEllipseN("Cross Power N", "", 2.0, 0.5, 8.0, 0.1) {
    ensureBuilt();
}

std::string Pod::getTypeName() const {
    return "Pod";
}

std::vector<glm::vec3> Pod::getSnapSamplePointsLocal() const {
    const float L = m_length.getFloat();
    const float ry = 0.5f * m_width.getFloat();
    const float rz = 0.5f * m_height.getFloat();
    std::vector<glm::vec3> pts = {
        {0.0f, 0.0f, 0.0f},
        {L, 0.0f, 0.0f},
        {0.5f * L, 0.0f, 0.0f},
        {0.5f * L, ry, 0.0f},
        {0.5f * L, -ry, 0.0f},
        {0.5f * L, 0.0f, rz},
        {0.5f * L, 0.0f, -rz}
    };
    return pts;
}

Parameter& Pod::length() noexcept { return m_length; }
const Parameter& Pod::length() const noexcept { return m_length; }

Parameter& Pod::width() noexcept { return m_width; }
const Parameter& Pod::width() const noexcept { return m_width; }

Parameter& Pod::height() noexcept { return m_height; }
const Parameter& Pod::height() const noexcept { return m_height; }

Parameter& Pod::noseFrac() noexcept { return m_noseFrac; }
const Parameter& Pod::noseFrac() const noexcept { return m_noseFrac; }

Parameter& Pod::tailFrac() noexcept { return m_tailFrac; }
const Parameter& Pod::tailFrac() const noexcept { return m_tailFrac; }

Parameter& Pod::nosePower() noexcept { return m_nosePower; }
const Parameter& Pod::nosePower() const noexcept { return m_nosePower; }

Parameter& Pod::tailPower() noexcept { return m_tailPower; }
const Parameter& Pod::tailPower() const noexcept { return m_tailPower; }

Parameter& Pod::superEllipseN() noexcept { return m_superEllipseN; }
const Parameter& Pod::superEllipseN() const noexcept { return m_superEllipseN; }

SymmetryMode Pod::getSymmetry() const noexcept { return m_symmetry; }
void Pod::setSymmetry(SymmetryMode mode) {
    if (m_symmetry != mode) {
        m_symmetry = mode;
        markDirty();
    }
}

int Pod::getRadialPoints() const noexcept { return m_radialPoints; }
void Pod::setRadialPoints(int count) {
    const int clamped = std::clamp(count, 8, 72);
    if (m_radialPoints != clamped) {
        m_radialPoints = clamped;
        markDirty();
    }
}

int Pod::getAxialStations() const noexcept { return m_axialStations; }
void Pod::setAxialStations(int count) {
    const int clamped = std::clamp(count, 8, 80);
    if (m_axialStations != clamped) {
        m_axialStations = clamped;
        markDirty();
    }
}

double Pod::getFinenessRatio() const noexcept {
    const double dEq = std::sqrt(std::max(m_width.getValue() * m_height.getValue(), 0.0));
    return (dEq > 1e-6) ? (m_length.getValue() / dEq) : 0.0;
}

double Pod::getWettedArea() const noexcept { return m_wettedArea; }
double Pod::getVolume() const noexcept { return m_volume; }

void Pod::rebuildMesh() {
    m_wettedArea = 0.0;
    m_volume = 0.0;

    const float L = m_length.getFloat();
    const float ryMax = 0.5f * m_width.getFloat();
    const float rzMax = 0.5f * m_height.getFloat();
    const float noseF = std::min(m_noseFrac.getFloat(), 0.90f);
    const float tailF = std::min(m_tailFrac.getFloat(), 0.95f - noseF);
    const float noseP = m_nosePower.getFloat();
    const float tailP = m_tailPower.getFloat();
    const float crossExp = 2.0f / m_superEllipseN.getFloat();

    std::vector<std::vector<glm::vec3>> rings;
    rings.reserve(static_cast<std::size_t>(m_axialStations) + 1);

    for (int s = 0; s <= m_axialStations; ++s) {
        const float u = static_cast<float>(s) / static_cast<float>(m_axialStations);
        const float x = u * L;

        float radiusScale = 1.0f;
        if (u < noseF && noseF > 1e-4f) {
            radiusScale = std::pow(u / noseF, noseP);
        } else if (u > (1.0f - tailF) && tailF > 1e-4f) {
            radiusScale = std::pow((1.0f - u) / tailF, tailP);
        }

        std::vector<glm::vec3> ring;
        ring.reserve(static_cast<std::size_t>(m_radialPoints) + 1);
        for (int p = 0; p <= m_radialPoints; ++p) {
            const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(p) / static_cast<float>(m_radialPoints);
            const float y = (ryMax * radiusScale) * signedPow(std::cos(angle), crossExp);
            const float z = (rzMax * radiusScale) * signedPow(std::sin(angle), crossExp);
            ring.emplace_back(x, y, z);
        }
        rings.push_back(std::move(ring));
    }

    MeshPrimitives::appendLoftedRings(m_mesh, rings, true, true, true);
    if (m_symmetry == SymmetryMode::XZ) {
        MeshPrimitives::mirrorSubmeshXZ(m_mesh, 0, 0);
    }

    m_wettedArea = MeshPrimitives::computeSurfaceArea(m_mesh);
    m_volume = MeshPrimitives::computeVolume(m_mesh);
}

} // namespace opads
