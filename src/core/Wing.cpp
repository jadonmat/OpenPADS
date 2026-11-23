#include "core/Wing.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

namespace opads {

namespace {

std::vector<glm::vec3> buildCrossSectionRing3D(
    const Airfoil& airfoil,
    const glm::vec3& leadingEdgePos,
    float chord,
    float twistDeg,
    float twistLocation,
    float dihedralDeg
) {
    const auto& loop = airfoil.getClosedLoop();
    std::vector<glm::vec3> ring;
    ring.reserve(loop.size());

    const float twistRad = glm::radians(twistDeg);
    const float cosT = std::cos(twistRad);
    const float sinT = std::sin(twistRad);

    const float dihedralRad = glm::radians(dihedralDeg);
    const float cosG = std::cos(dihedralRad);
    const float sinG = std::sin(dihedralRad);

    const float pivotX = twistLocation * chord;

    for (const glm::vec2& pt : loop) {
        const float localX = pt.x * chord - pivotX;
        const float localZ = pt.y * chord;

        // Nose-up twist: positive rotation moves LE up (+Z) and TE down (-Z).
        const float twistedX = pivotX + localX * cosT + localZ * sinT;
        const float twistedZ = -localX * sinT + localZ * cosT;

        // Orient section normal perpendicular to the dihedral plane so vertical tails (90 deg) don't collapse.
        const glm::vec3 worldPt(
            leadingEdgePos.x + twistedX,
            leadingEdgePos.y - twistedZ * sinG,
            leadingEdgePos.z + twistedZ * cosG
        );
        ring.push_back(worldPt);
    }

    return ring;
}

std::vector<glm::vec3> interpolateRings(const std::vector<glm::vec3>& a, const std::vector<glm::vec3>& b, float t) {
    const std::size_t n = std::min(a.size(), b.size());
    std::vector<glm::vec3> out(n);
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = a[i] + t * (b[i] - a[i]);
    }
    return out;
}

void appendAirfoilCap(
    Mesh& mesh,
    const std::vector<glm::vec3>& ring,
    const glm::vec3& outwardNormal,
    bool reverseWinding
) {
    const std::size_t total = ring.size();
    if (total < 3) {
        return;
    }

    // Ring is ordered Upper TE (0) -> LE (half) -> Lower TE (total - 1).
    const std::size_t half = total / 2;
    const auto base = static_cast<std::uint32_t>(mesh.getVertexCount());
    for (const glm::vec3& pos : ring) {
        mesh.addVertex(pos, outwardNormal);
    }

    for (std::size_t k = 0; k < half; ++k) {
        const auto u0 = base + static_cast<std::uint32_t>(k);
        const auto u1 = base + static_cast<std::uint32_t>(k + 1);
        const auto l0 = base + static_cast<std::uint32_t>(total - 1 - k);
        const auto l1 = base + static_cast<std::uint32_t>(total - 2 - k);

        if (!reverseWinding) {
            if (k + 1 < half) {
                mesh.addTriangle(u0, l0, l1);
                mesh.addTriangle(u0, l1, u1);
            } else {
                mesh.addTriangle(u0, l0, u1);
            }
        } else {
            if (k + 1 < half) {
                mesh.addTriangle(u0, l1, l0);
                mesh.addTriangle(u0, u1, l1);
            } else {
                mesh.addTriangle(u0, u1, l0);
            }
        }
    }
}

void appendWingHalf(
    Mesh& mesh,
    const std::vector<std::vector<glm::vec3>>& rings,
    bool capRoot,
    bool capTip
) {
    if (rings.size() < 2) {
        return;
    }
    const std::size_t ringCount = rings.size();
    const std::size_t ptCount = rings.front().size();
    if (ptCount < 3) {
        return;
    }

    std::vector<glm::vec3> normals(ringCount * ptCount, glm::vec3(0.0f));
    const auto idxAt = [ptCount](std::size_t r, std::size_t p) {
        return r * ptCount + p;
    };

    // Accumulate area-weighted face normals across skin quads.
    for (std::size_t r = 0; r + 1 < ringCount; ++r) {
        for (std::size_t p = 0; p + 1 < ptCount; ++p) {
            const glm::vec3& v00 = rings[r][p];
            const glm::vec3& v10 = rings[r + 1][p];
            const glm::vec3& v11 = rings[r + 1][p + 1];
            const glm::vec3& v01 = rings[r][p + 1];

            const glm::vec3 n1 = glm::cross(v10 - v00, v11 - v00);
            const glm::vec3 n2 = glm::cross(v11 - v00, v01 - v00);

            normals[idxAt(r, p)] += n1 + n2;
            normals[idxAt(r + 1, p)] += n1;
            normals[idxAt(r + 1, p + 1)] += n1 + n2;
            normals[idxAt(r, p + 1)] += n2;
        }
    }

    const auto base = static_cast<std::uint32_t>(mesh.getVertexCount());
    for (std::size_t r = 0; r < ringCount; ++r) {
        for (std::size_t p = 0; p < ptCount; ++p) {
            glm::vec3 n = normals[idxAt(r, p)];
            const float len = glm::length(n);
            n = (len > 1e-7f) ? (n / len) : glm::vec3(0.0f, 0.0f, 1.0f);
            mesh.addVertex(rings[r][p], n);
        }
    }

    for (std::size_t r = 0; r + 1 < ringCount; ++r) {
        for (std::size_t p = 0; p + 1 < ptCount; ++p) {
            const auto i00 = base + static_cast<std::uint32_t>(idxAt(r, p));
            const auto i10 = base + static_cast<std::uint32_t>(idxAt(r + 1, p));
            const auto i11 = base + static_cast<std::uint32_t>(idxAt(r + 1, p + 1));
            const auto i01 = base + static_cast<std::uint32_t>(idxAt(r, p + 1));

            mesh.addTriangle(i00, i10, i11);
            mesh.addTriangle(i00, i11, i01);
        }
    }

    if (capRoot) {
        glm::vec3 spanDir = rings[1][0] - rings[0][0];
        const float len = glm::length(spanDir);
        const glm::vec3 rootNormal = (len > 1e-6f) ? -(spanDir / len) : glm::vec3(0.0f, -1.0f, 0.0f);
        appendAirfoilCap(mesh, rings.front(), rootNormal, true);
    }

    if (capTip) {
        glm::vec3 spanDir = rings[ringCount - 1][0] - rings[ringCount - 2][0];
        const float len = glm::length(spanDir);
        const glm::vec3 tipNormal = (len > 1e-6f) ? (spanDir / len) : glm::vec3(0.0f, 1.0f, 0.0f);
        appendAirfoilCap(mesh, rings.back(), tipNormal, false);
    }
}

double computeMeshSurfaceArea(const Mesh& mesh) noexcept {
    const auto& verts = mesh.getVertices();
    const auto& indices = mesh.getIndices();
    double area = 0.0;
    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        const glm::vec3& a = verts[indices[i]].position;
        const glm::vec3& b = verts[indices[i + 1]].position;
        const glm::vec3& c = verts[indices[i + 2]].position;
        area += 0.5 * static_cast<double>(glm::length(glm::cross(b - a, c - a)));
    }
    return area;
}

} // namespace

Wing::Wing(std::uint64_t id, std::string name)
    : Component(id, std::move(name)) {
    m_sections.emplace_back();
    ensureBuilt();
}

std::string Wing::getTypeName() const {
    return "Wing";
}

std::vector<std::vector<glm::vec3>> Wing::getHighlightLoops() const {
    if (m_crossSectionRings.empty()) {
        return {};
    }
    const std::size_t idx = std::min(m_selectedCrossSection, m_crossSectionRings.size() - 1);
    std::vector<std::vector<glm::vec3>> result;
    result.push_back(m_crossSectionRings[idx]);

    if (m_symmetry == SymmetryMode::XZ) {
        std::vector<glm::vec3> mirrored = m_crossSectionRings[idx];
        for (glm::vec3& pt : mirrored) {
            pt.y = -pt.y;
        }
        result.push_back(std::move(mirrored));
    }
    return result;
}

SymmetryMode Wing::getSymmetry() const noexcept {
    return m_symmetry;
}

void Wing::setSymmetry(SymmetryMode mode) {
    if (m_symmetry != mode) {
        m_symmetry = mode;
        markDirty();
    }
}

bool Wing::getCapTips() const noexcept {
    return m_capTips;
}

void Wing::setCapTips(bool cap) {
    if (m_capTips != cap) {
        m_capTips = cap;
        markDirty();
    }
}

int Wing::getChordPoints() const noexcept {
    return m_chordPoints;
}

void Wing::setChordPoints(int pointsPerSurface) {
    const int clamped = std::clamp(pointsPerSurface, 9, 121);
    if (clamped != m_chordPoints) {
        m_chordPoints = clamped;
        markDirty();
    }
}

std::size_t Wing::getSectionCount() const noexcept {
    return m_sections.size();
}

std::size_t Wing::getCrossSectionCount() const noexcept {
    return m_sections.size() + 1;
}

WingSection& Wing::getSection(std::size_t index) {
    return m_sections.at(index);
}

const WingSection& Wing::getSection(std::size_t index) const {
    return m_sections.at(index);
}

const std::vector<WingSection>& Wing::getSections() const noexcept {
    return m_sections;
}

void Wing::addSection() {
    WingSection nextBay;
    if (!m_sections.empty()) {
        const WingSection& prev = m_sections.back();
        nextBay.rootChord().setValue(prev.tipChord().getValue());
        nextBay.tipChord().setValue(std::max(0.2, prev.tipChord().getValue() * 0.7));
        nextBay.span().setValue(std::max(0.5, prev.span().getValue() * 0.5));
        nextBay.sweepDeg().setValue(prev.sweepDeg().getValue());
        nextBay.sweepLocation().setValue(prev.sweepLocation().getValue());
        nextBay.dihedralDeg().setValue(prev.dihedralDeg().getValue());
        nextBay.rootTwistDeg().setValue(prev.tipTwistDeg().getValue());
        nextBay.tipTwistDeg().setValue(prev.tipTwistDeg().getValue());
        nextBay.twistLocation().setValue(prev.twistLocation().getValue());
        nextBay.getRootAirfoil() = prev.getTipAirfoil();
        nextBay.getTipAirfoil() = prev.getTipAirfoil();
    }
    m_sections.push_back(std::move(nextBay));
    m_selectedCrossSection = m_sections.size();
    markDirty();
}

bool Wing::removeSection(std::size_t index) {
    if (m_sections.size() <= 1 || index >= m_sections.size()) {
        return false;
    }
    m_sections.erase(m_sections.begin() + static_cast<std::ptrdiff_t>(index));
    if (index > 0 && index < m_sections.size()) {
        syncSectionsFromBay(index - 1);
    }
    m_selectedCrossSection = std::min(m_selectedCrossSection, m_sections.size());
    markDirty();
    return true;
}

void Wing::syncSectionsFromBay(std::size_t bayIndex) {
    if (bayIndex >= m_sections.size()) {
        return;
    }
    if (bayIndex > 0) {
        WingSection& prev = m_sections[bayIndex - 1];
        const WingSection& curr = m_sections[bayIndex];
        prev.tipChord().setValue(curr.rootChord().getValue());
        prev.tipTwistDeg().setValue(curr.rootTwistDeg().getValue());
        prev.getTipAirfoil() = curr.getRootAirfoil();
    }
    if (bayIndex + 1 < m_sections.size()) {
        const WingSection& curr = m_sections[bayIndex];
        WingSection& next = m_sections[bayIndex + 1];
        next.rootChord().setValue(curr.tipChord().getValue());
        next.rootTwistDeg().setValue(curr.tipTwistDeg().getValue());
        next.getRootAirfoil() = curr.getTipAirfoil();
    }
    markDirty();
}

void Wing::syncSectionsFromCrossSection(std::size_t xsecIndex) {
    if (xsecIndex == 0 && !m_sections.empty()) {
        markDirty();
        return;
    }
    if (xsecIndex > 0 && xsecIndex <= m_sections.size()) {
        const std::size_t prevBay = xsecIndex - 1;
        if (xsecIndex < m_sections.size()) {
            const WingSection& left = m_sections[prevBay];
            WingSection& right = m_sections[xsecIndex];
            right.rootChord().setValue(left.tipChord().getValue());
            right.rootTwistDeg().setValue(left.tipTwistDeg().getValue());
            right.getRootAirfoil() = left.getTipAirfoil();
        }
        markDirty();
    }
}

std::size_t Wing::getSelectedCrossSection() const noexcept {
    return m_selectedCrossSection;
}

void Wing::setSelectedCrossSection(std::size_t xsecIndex) noexcept {
    m_selectedCrossSection = std::min(xsecIndex, m_sections.size());
}

glm::vec3 Wing::getCrossSectionLeadingEdge(std::size_t xsecIndex) const noexcept {
    glm::vec3 le(0.0f, 0.0f, 0.0f);
    const std::size_t clamped = std::min(xsecIndex, m_sections.size());
    for (std::size_t i = 0; i < clamped; ++i) {
        le = m_sections[i].computeTipLeadingEdge(le);
    }
    return le;
}

void Wing::moveCrossSectionLeadingEdge(std::size_t xsecIndex, const glm::vec3& newLocalTipLE) {
    if (xsecIndex == 0 || xsecIndex > m_sections.size()) {
        return;
    }
    const std::size_t bayIndex = xsecIndex - 1;
    const glm::vec3 rootLE = getCrossSectionLeadingEdge(bayIndex);
    const glm::vec3 delta = newLocalTipLE - rootLE;

    const float newSpan = std::max(0.05f, std::sqrt(delta.y * delta.y + delta.z * delta.z));
    const float newDihedralDeg = std::clamp(glm::degrees(std::atan2(delta.z, std::max(delta.y, 1e-4f))), -89.0f, 90.0f);

    WingSection& bay = m_sections[bayIndex];
    const float f = bay.sweepLocation().getFloat();
    const float cr = bay.rootChord().getFloat();
    const float ct = bay.tipChord().getFloat();
    const float sweepNumerator = delta.x - f * (cr - ct);
    const float newSweepDeg = std::clamp(glm::degrees(std::atan2(sweepNumerator, newSpan)), -75.0f, 75.0f);

    bay.span().setFloat(newSpan);
    bay.dihedralDeg().setFloat(newDihedralDeg);
    bay.sweepDeg().setFloat(newSweepDeg);
    markDirty();
}

float Wing::getCrossSectionChord(std::size_t xsecIndex) const noexcept {
    if (m_sections.empty()) {
        return 1.0f;
    }
    if (xsecIndex == 0) {
        return m_sections.front().rootChord().getFloat();
    }
    const std::size_t bayIdx = std::min(xsecIndex - 1, m_sections.size() - 1);
    return m_sections[bayIdx].tipChord().getFloat();
}

void Wing::setCrossSectionChord(std::size_t xsecIndex, float chord) {
    if (m_sections.empty()) {
        return;
    }
    const std::size_t clamped = std::min(xsecIndex, m_sections.size());
    if (clamped == 0) {
        m_sections.front().rootChord().setFloat(chord);
    } else {
        m_sections[clamped - 1].tipChord().setFloat(chord);
    }
    syncSectionsFromCrossSection(clamped);
}

float Wing::getCrossSectionTwistDeg(std::size_t xsecIndex) const noexcept {
    if (m_sections.empty()) {
        return 0.0f;
    }
    if (xsecIndex == 0) {
        return m_sections.front().rootTwistDeg().getFloat();
    }
    const std::size_t bayIdx = std::min(xsecIndex - 1, m_sections.size() - 1);
    return m_sections[bayIdx].tipTwistDeg().getFloat();
}

void Wing::setCrossSectionTwistDeg(std::size_t xsecIndex, float twistDeg) {
    if (m_sections.empty()) {
        return;
    }
    const std::size_t clamped = std::min(xsecIndex, m_sections.size());
    if (clamped == 0) {
        m_sections.front().rootTwistDeg().setFloat(twistDeg);
    } else {
        m_sections[clamped - 1].tipTwistDeg().setFloat(twistDeg);
    }
    syncSectionsFromCrossSection(clamped);
}

Airfoil* Wing::getCrossSectionAirfoil(std::size_t xsecIndex) noexcept {
    if (m_sections.empty()) {
        return nullptr;
    }
    if (xsecIndex == 0) {
        return &m_sections.front().getRootAirfoil();
    }
    const std::size_t bayIdx = std::min(xsecIndex - 1, m_sections.size() - 1);
    return &m_sections[bayIdx].getTipAirfoil();
}

double Wing::getPlanformArea() const noexcept {
    double area = 0.0;
    for (const WingSection& bay : m_sections) {
        area += bay.getArea();
    }
    if (m_symmetry == SymmetryMode::XZ) {
        area *= 2.0;
    }
    return area;
}

double Wing::getSpan() const noexcept {
    double span = 0.0;
    for (const WingSection& bay : m_sections) {
        span += bay.getProjectedYSpan();
    }
    if (m_symmetry == SymmetryMode::XZ) {
        span *= 2.0;
    }
    return span;
}

double Wing::getAspectRatio() const noexcept {
    const double s = getPlanformArea();
    const double b = getSpan();
    return (s > 1e-9) ? (b * b / s) : 0.0;
}

double Wing::getMeanAerodynamicChord() const noexcept {
    double weightedSum = 0.0;
    double areaSum = 0.0;
    for (const WingSection& bay : m_sections) {
        const double bayArea = bay.getArea();
        weightedSum += bayArea * bay.getMeanAerodynamicChord();
        areaSum += bayArea;
    }
    return (areaSum > 1e-9) ? (weightedSum / areaSum) : 0.0;
}

double Wing::getWettedArea() const noexcept {
    return m_wettedArea;
}

void Wing::rebuildMesh() {
    m_crossSectionRings.clear();
    m_wettedArea = 0.0;

    if (m_sections.empty()) {
        return;
    }

    for (WingSection& bay : m_sections) {
        bay.getRootAirfoil().setPointsPerSurface(m_chordPoints);
        bay.getTipAirfoil().setPointsPerSurface(m_chordPoints);
    }

    std::vector<std::vector<glm::vec3>> loftRings;
    glm::vec3 currentLE(0.0f, 0.0f, 0.0f);

    for (std::size_t b = 0; b < m_sections.size(); ++b) {
        const WingSection& bay = m_sections[b];
        const glm::vec3 tipLE = bay.computeTipLeadingEdge(currentLE);

        const std::vector<glm::vec3> rootRing = buildCrossSectionRing3D(
            bay.getRootAirfoil(),
            currentLE,
            bay.rootChord().getFloat(),
            bay.rootTwistDeg().getFloat(),
            bay.twistLocation().getFloat(),
            bay.dihedralDeg().getFloat()
        );
        const std::vector<glm::vec3> tipRing = buildCrossSectionRing3D(
            bay.getTipAirfoil(),
            tipLE,
            bay.tipChord().getFloat(),
            bay.tipTwistDeg().getFloat(),
            bay.twistLocation().getFloat(),
            bay.dihedralDeg().getFloat()
        );

        if (b == 0) {
            m_crossSectionRings.push_back(rootRing);
            loftRings.push_back(rootRing);
        }

        const int sub = bay.getSpanSubdivisions();
        for (int s = 1; s < sub; ++s) {
            const float t = static_cast<float>(s) / static_cast<float>(sub);
            loftRings.push_back(interpolateRings(rootRing, tipRing, t));
        }

        m_crossSectionRings.push_back(tipRing);
        loftRings.push_back(tipRing);

        currentLE = tipLE;
    }

    const bool capRoot = m_capTips && (m_symmetry == SymmetryMode::None);
    appendWingHalf(m_mesh, loftRings, capRoot, m_capTips);

    if (m_symmetry == SymmetryMode::XZ) {
        const std::size_t halfVertCount = m_mesh.getVertexCount();
        const std::size_t halfIdxCount = m_mesh.getIndices().size();

        for (std::size_t i = 0; i < halfVertCount; ++i) {
            const Vertex& v = m_mesh.getVertices()[i];
            m_mesh.addVertex(
                glm::vec3(v.position.x, -v.position.y, v.position.z),
                glm::vec3(v.normal.x, -v.normal.y, v.normal.z)
            );
        }

        const auto vertOffset = static_cast<std::uint32_t>(halfVertCount);
        for (std::size_t i = 0; i + 2 < halfIdxCount; i += 3) {
            const auto i0 = m_mesh.getIndices()[i];
            const auto i1 = m_mesh.getIndices()[i + 1];
            const auto i2 = m_mesh.getIndices()[i + 2];
            // Reverse winding for negative-Y reflection so outward normals stay valid.
            m_mesh.addTriangle(vertOffset + i0, vertOffset + i2, vertOffset + i1);
        }
    }

    m_wettedArea = computeMeshSurfaceArea(m_mesh);
}

} // namespace opads
