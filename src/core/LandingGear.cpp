#include "core/LandingGear.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include <glm/trigonometric.hpp>

#include "core/MeshPrimitives.hpp"

namespace opads {

namespace {

void appendWheel(Mesh& mesh, const glm::vec3& center, float diameter, float width) {
    const float rTire = 0.5f * diameter;
    const float halfW = 0.5f * width;
    constexpr int kProfilePts = 9;
    constexpr int kRadial = 24;

    // Revolve a beveled tire cross-section around the Y axle
    std::vector<std::vector<glm::vec3>> rings;
    rings.reserve(kProfilePts);

    for (int s = 0; s < kProfilePts; ++s) {
        const float t = static_cast<float>(s) / static_cast<float>(kProfilePts - 1);
        const float yLocal = (-1.0f + 2.0f * t) * halfW;
        // Rounded sidewall taper near +/- halfW
        const float edgeFrac = std::abs(t - 0.5f) * 2.0f;
        const float rRing = rTire * (0.72f + 0.28f * std::sqrt(std::max(0.0f, 1.0f - edgeFrac * edgeFrac)));

        std::vector<glm::vec3> ring;
        ring.reserve(kRadial + 1);
        for (int p = 0; p <= kRadial; ++p) {
            const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(p) / static_cast<float>(kRadial);
            ring.emplace_back(
                center.x + rRing * std::cos(a),
                center.y + yLocal,
                center.z + rRing * std::sin(a)
            );
        }
        rings.push_back(std::move(ring));
    }

    MeshPrimitives::appendLoftedRings(mesh, rings, true, true, true);
}

void appendWheelPant(Mesh& mesh, const glm::vec3& center, float wheelDiam, float wheelWidth) {
    const float len = wheelDiam * 1.95f;
    const float halfW = wheelWidth * 0.78f;
    const float halfH = wheelDiam * 0.58f;
    constexpr int kStations = 18;
    constexpr int kRadial = 20;

    std::vector<std::vector<glm::vec3>> rings;
    rings.reserve(kStations + 1);
    for (int s = 0; s <= kStations; ++s) {
        const float u = static_cast<float>(s) / static_cast<float>(kStations);
        const float x = center.x - 0.42f * len + u * len;
        float profile = 1.0f;
        if (u < 0.35f) {
            profile = std::pow(u / 0.35f, 0.52f);
        } else {
            profile = std::pow((1.0f - u) / 0.65f, 0.75f);
        }

        std::vector<glm::vec3> ring;
        ring.reserve(kRadial + 1);
        for (int p = 0; p <= kRadial; ++p) {
            const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(p) / static_cast<float>(kRadial);
            ring.emplace_back(
                x,
                center.y + halfW * profile * std::cos(a),
                center.z + 0.04f * wheelDiam + halfH * profile * std::sin(a)
            );
        }
        rings.push_back(std::move(ring));
    }

    MeshPrimitives::appendLoftedRings(mesh, rings, true, true, true);
}

} // namespace

LandingGear::LandingGear(std::uint64_t id, std::string name)
    : Component(id, std::move(name))
    , m_strutLength("Strut Length", "m", 1.15, 0.1, 15.0, 0.05)
    , m_strutDiameter("Strut Diam", "m", 0.09, 0.01, 2.0, 0.01)
    , m_rakeDeg("Rake", "deg", 6.0, -45.0, 45.0, 0.5)
    , m_splayDeg("Splay", "deg", 10.0, -45.0, 45.0, 0.5)
    , m_wheelDiameter("Wheel Diam", "m", 0.48, 0.05, 5.0, 0.02)
    , m_wheelWidth("Wheel Width", "m", 0.18, 0.02, 2.0, 0.01) {
    ensureBuilt();
}

std::string LandingGear::getTypeName() const {
    return "LandingGear";
}

std::vector<glm::vec3> LandingGear::getSnapSamplePointsLocal() const {
    const float L = m_strutLength.getFloat();
    const float rake = glm::radians(m_rakeDeg.getFloat());
    const float splay = glm::radians(m_splayDeg.getFloat());
    const glm::vec3 axle(L * std::sin(rake), L * std::sin(splay), -L * std::cos(rake) * std::cos(splay));
    const float rW = 0.5f * m_wheelDiameter.getFloat();
    return {
        {0.0f, 0.0f, 0.0f},
        axle,
        axle - glm::vec3(0.0f, 0.0f, rW)
    };
}

Parameter& LandingGear::strutLength() noexcept { return m_strutLength; }
const Parameter& LandingGear::strutLength() const noexcept { return m_strutLength; }

Parameter& LandingGear::strutDiameter() noexcept { return m_strutDiameter; }
const Parameter& LandingGear::strutDiameter() const noexcept { return m_strutDiameter; }

Parameter& LandingGear::rakeDeg() noexcept { return m_rakeDeg; }
const Parameter& LandingGear::rakeDeg() const noexcept { return m_rakeDeg; }

Parameter& LandingGear::splayDeg() noexcept { return m_splayDeg; }
const Parameter& LandingGear::splayDeg() const noexcept { return m_splayDeg; }

Parameter& LandingGear::wheelDiameter() noexcept { return m_wheelDiameter; }
const Parameter& LandingGear::wheelDiameter() const noexcept { return m_wheelDiameter; }

Parameter& LandingGear::wheelWidth() noexcept { return m_wheelWidth; }
const Parameter& LandingGear::wheelWidth() const noexcept { return m_wheelWidth; }

int LandingGear::getWheelsPerStrut() const noexcept { return m_wheelsPerStrut; }
void LandingGear::setWheelsPerStrut(int count) {
    const int clamped = std::clamp(count, 1, 2);
    if (m_wheelsPerStrut != clamped) {
        m_wheelsPerStrut = clamped;
        markDirty();
    }
}

bool LandingGear::hasWheelPant() const noexcept { return m_wheelPant; }
void LandingGear::setWheelPant(bool enabled) {
    if (m_wheelPant != enabled) {
        m_wheelPant = enabled;
        markDirty();
    }
}

SymmetryMode LandingGear::getSymmetry() const noexcept { return m_symmetry; }
void LandingGear::setSymmetry(SymmetryMode mode) {
    if (m_symmetry != mode) {
        m_symmetry = mode;
        markDirty();
    }
}

double LandingGear::getFrontalArea() const noexcept {
    const double strutArea = m_strutLength.getValue() * m_strutDiameter.getValue();
    const double wheelArea = static_cast<double>(m_wheelsPerStrut) * m_wheelDiameter.getValue() * m_wheelWidth.getValue();
    const double singleSide = strutArea + wheelArea;
    return (m_symmetry == SymmetryMode::XZ) ? (2.0 * singleSide) : singleSide;
}

double LandingGear::getWettedArea() const noexcept {
    return m_wettedArea;
}

void LandingGear::rebuildMesh() {
    m_wettedArea = 0.0;

    const float L = m_strutLength.getFloat();
    const float rStrut = 0.5f * m_strutDiameter.getFloat();
    const float rake = glm::radians(m_rakeDeg.getFloat());
    const float splay = glm::radians(m_splayDeg.getFloat());

    const glm::vec3 topPos(0.0f, 0.0f, 0.0f);
    const glm::vec3 axleCenter(
        L * std::sin(rake),
        L * std::sin(splay),
        -L * std::cos(rake) * std::cos(splay)
    );
    const glm::vec3 oleoJoint = topPos + (axleCenter - topPos) * 0.68f;

    // Upper main strut + lower narrower oleo piston
    MeshPrimitives::appendCylinder(m_mesh, topPos, oleoJoint, rStrut, rStrut * 0.92f, 18, true);
    MeshPrimitives::appendCylinder(m_mesh, oleoJoint, axleCenter, rStrut * 0.68f, rStrut * 0.65f, 18, true);

    const float wDiam = m_wheelDiameter.getFloat();
    const float wWidth = m_wheelWidth.getFloat();

    if (m_wheelsPerStrut == 1) {
        const glm::vec3 axleOut = axleCenter + glm::vec3(0.0f, wWidth * 0.4f, 0.0f);
        const glm::vec3 axleIn = axleCenter - glm::vec3(0.0f, wWidth * 0.25f, 0.0f);
        MeshPrimitives::appendCylinder(m_mesh, axleIn, axleOut, rStrut * 0.48f, rStrut * 0.48f, 14, true);

        const glm::vec3 wheelCenter = axleCenter + glm::vec3(0.0f, wWidth * 0.15f, 0.0f);
        appendWheel(m_mesh, wheelCenter, wDiam, wWidth);
        if (m_wheelPant) {
            appendWheelPant(m_mesh, wheelCenter, wDiam, wWidth);
        }
    } else {
        const float offset = wWidth * 0.72f + rStrut * 0.6f;
        const glm::vec3 leftWheel = axleCenter - glm::vec3(0.0f, offset, 0.0f);
        const glm::vec3 rightWheel = axleCenter + glm::vec3(0.0f, offset, 0.0f);
        MeshPrimitives::appendCylinder(m_mesh, leftWheel, rightWheel, rStrut * 0.45f, rStrut * 0.45f, 14, true);
        appendWheel(m_mesh, leftWheel, wDiam, wWidth);
        appendWheel(m_mesh, rightWheel, wDiam, wWidth);
        if (m_wheelPant) {
            appendWheelPant(m_mesh, leftWheel, wDiam, wWidth);
            appendWheelPant(m_mesh, rightWheel, wDiam, wWidth);
        }
    }

    if (m_symmetry == SymmetryMode::XZ) {
        MeshPrimitives::mirrorSubmeshXZ(m_mesh, 0, 0);
    }

    m_wettedArea = MeshPrimitives::computeSurfaceArea(m_mesh);
}

} // namespace opads
