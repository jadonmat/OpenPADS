#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

#include "core/Aircraft.hpp"
#include "core/Airfoil.hpp"
#include "core/Component.hpp"
#include "core/Parameter.hpp"
#include "core/Transform.hpp"
#include "core/Wing.hpp"
#include "render/Gizmo.hpp"
#include "render/OrbitCamera.hpp"
#include "render/RayPicker.hpp"

namespace {

class MockComponent final : public opads::Component {
public:
    MockComponent(std::uint64_t id, std::string name)
        : Component(id, std::move(name)) {
    }

    std::string getTypeName() const override {
        return "Mock";
    }

protected:
    void rebuildMesh() override {
        m_mesh.addVertex({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
        m_mesh.addVertex({1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
        m_mesh.addVertex({0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
        m_mesh.addTriangle(0, 1, 2);
        ++buildCount;
    }

public:
    int buildCount{0};
};

} // namespace

TEST_CASE("Transform builds identity and translation matrices", "[core][transform]") {
    opads::Transform t;
    const glm::mat4 identity = t.getMatrix();
    REQUIRE(identity[3][0] == Catch::Approx(0.0f));
    REQUIRE(identity[3][1] == Catch::Approx(0.0f));
    REQUIRE(identity[3][2] == Catch::Approx(0.0f));

    t.setPosition({3.5f, -2.0f, 1.25f});
    const glm::mat4 translated = t.getMatrix();
    REQUIRE(translated[3][0] == Catch::Approx(3.5f));
    REQUIRE(translated[3][1] == Catch::Approx(-2.0f));
    REQUIRE(translated[3][2] == Catch::Approx(1.25f));
}

TEST_CASE("Parameter clamps range and detects changes", "[core][parameter]") {
    opads::Parameter p("Span", "m", 5.0, 0.5, 20.0, 0.1);
    REQUIRE(p.getValue() == Catch::Approx(5.0));
    REQUIRE_FALSE(p.setValue(5.0));
    REQUIRE(p.setValue(30.0));
    REQUIRE(p.getValue() == Catch::Approx(20.0));
    REQUIRE(p.setValue(-2.0));
    REQUIRE(p.getValue() == Catch::Approx(0.5));
}

TEST_CASE("Aircraft manages component ownership, dirty rebuilds, and selection", "[core][aircraft]") {
    opads::Aircraft aircraft("Test Jet");
    REQUIRE(aircraft.getName() == "Test Jet");
    REQUIRE(aircraft.getComponents().empty());

    const auto id = aircraft.allocateComponentId();
    auto comp = std::make_unique<MockComponent>(id, "Main Wing");
    auto* rawPtr = static_cast<MockComponent*>(aircraft.addComponent(std::move(comp)));

    REQUIRE(rawPtr != nullptr);
    REQUIRE(rawPtr->isDirty());
    aircraft.rebuildDirty();
    REQUIRE_FALSE(rawPtr->isDirty());
    REQUIRE(rawPtr->buildCount == 1);
    REQUIRE(rawPtr->getMesh().getVertexCount() == 3);
    REQUIRE(rawPtr->getMesh().getTriangleCount() == 1);

    aircraft.rebuildDirty();
    REQUIRE(rawPtr->buildCount == 1);

    aircraft.setSelectedComponentId(id);
    REQUIRE(aircraft.getSelectedComponentId() == id);
    REQUIRE(aircraft.removeComponent(id));
    REQUIRE_FALSE(aircraft.getSelectedComponentId().has_value());
    REQUIRE(aircraft.getComponents().empty());
}

TEST_CASE("Airfoil generates NACA 4-digit and 5-digit sections", "[core][airfoil]") {
    const auto naca0012 = opads::Airfoil::createNaca4("0012", 41);
    REQUIRE(naca0012.getUpperSurface().size() == 41);
    REQUIRE(naca0012.getLowerSurface().size() == 41);
    REQUIRE(naca0012.getClosedLoop().size() == 81);
    REQUIRE(naca0012.getMaxThicknessRatio() == Catch::Approx(0.12f).margin(0.002f));
    REQUIRE(naca0012.getMaxCamberRatio() == Catch::Approx(0.0f).margin(1e-5f));

    for (std::size_t i = 0; i < naca0012.getUpperSurface().size(); ++i) {
        REQUIRE(naca0012.getUpperSurface()[i].y == Catch::Approx(-naca0012.getLowerSurface()[i].y).margin(1e-5f));
    }

    const auto naca2412 = opads::Airfoil::createNaca4("2412", 41);
    REQUIRE(naca2412.getMaxCamberRatio() == Catch::Approx(0.02f).margin(0.001f));
    REQUIRE(naca2412.getMaxThicknessRatio() == Catch::Approx(0.12f).margin(0.003f));

    const auto naca23012 = opads::Airfoil::createNaca5("23012", 41);
    REQUIRE(naca23012.getClosedLoop().size() == 81);
    REQUIRE(naca23012.getMaxThicknessRatio() == Catch::Approx(0.12f).margin(0.005f));
    REQUIRE(naca23012.getMaxCamberRatio() > 0.01f);
}

TEST_CASE("Airfoil loads and resamples bundled Selig and Lednicer .dat library", "[core][airfoil]") {
    const std::filesystem::path airfoilsDir = std::filesystem::path(OPENPADS_DATA_DIR) / "airfoils";
    const auto files = opads::Airfoil::discoverDatLibrary(airfoilsDir);
    REQUIRE(files.size() >= 10);

    for (const auto& path : files) {
        opads::Airfoil foil;
        INFO("Loading airfoil file: " << path.string());
        REQUIRE(foil.loadFromDatFile(path));
        REQUIRE(foil.getClosedLoop().size() == 81);
        REQUIRE(foil.getMaxThicknessRatio() > 0.04f);
        REQUIRE(foil.getMaxThicknessRatio() < 0.30f);
    }
}

TEST_CASE("Wing computes exact trapezoidal aero metrics and valid symmetric meshes", "[core][wing]") {
    opads::Wing wing(1, "Test Wing");
    auto& bay = wing.getSection(0);
    bay.span().setValue(5.0);
    bay.rootChord().setValue(2.0);
    bay.tipChord().setValue(1.0);
    bay.dihedralDeg().setValue(0.0);
    bay.sweepDeg().setValue(15.0);
    wing.setSymmetry(opads::SymmetryMode::XZ);
    wing.syncSectionsFromBay(0);
    wing.ensureBuilt();

    // S = 2 * 0.5 * 5 * (2 + 1) = 15.0 m^2; b = 10.0 m; AR = 100 / 15 = 6.6667
    REQUIRE(wing.getSpan() == Catch::Approx(10.0).margin(1e-6));
    REQUIRE(wing.getPlanformArea() == Catch::Approx(15.0).margin(1e-6));
    REQUIRE(wing.getAspectRatio() == Catch::Approx(100.0 / 15.0).margin(1e-5));

    // MAC = (2/3) * 2.0 * (1 + 0.5 + 0.25) / (1 + 0.5) = 14 / 9 = 1.55556
    REQUIRE(wing.getMeanAerodynamicChord() == Catch::Approx(14.0 / 9.0).margin(1e-5));
    REQUIRE(wing.getWettedArea() > 28.0);

    const auto& mesh = wing.getMesh();
    REQUIRE_FALSE(mesh.isEmpty());
    const auto& verts = mesh.getVertices();
    const auto& indices = mesh.getIndices();
    REQUIRE(indices.size() % 3 == 0);
    for (const auto idx : indices) {
        REQUIRE(idx < verts.size());
    }
    for (const auto& v : verts) {
        REQUIRE(std::isfinite(v.position.x));
        REQUIRE(std::isfinite(v.position.y));
        REQUIRE(std::isfinite(v.position.z));
        REQUIRE(glm::length(v.normal) == Catch::Approx(1.0f).margin(1e-3f));
    }

    // Multi-bay section chaining check
    wing.addSection();
    REQUIRE(wing.getSectionCount() == 2);
    REQUIRE(wing.getCrossSectionCount() == 3);
    REQUIRE(wing.getSection(1).rootChord().getValue() == Catch::Approx(1.0));
    wing.ensureBuilt();
    REQUIRE(wing.getHighlightLoops().size() == 2);
}

TEST_CASE("Airfoil CST conversion fits NACA 2412 and round-trips .dat export", "[core][airfoil][cst]") {
    auto foil = opads::Airfoil::createNaca4("2412", 41);
    const float origThick = foil.getMaxThicknessRatio();
    const float origCamber = foil.getMaxCamberRatio();

    foil.convertToCst(4);
    REQUIRE(foil.getType() == opads::AirfoilType::Cst);
    REQUIRE(foil.getCstUpperWeights().size() == 5);
    REQUIRE(foil.getMaxThicknessRatio() == Catch::Approx(origThick).margin(0.004f));
    REQUIRE(foil.getMaxCamberRatio() == Catch::Approx(origCamber).margin(0.003f));

    const std::filesystem::path tmpDat = std::filesystem::temp_directory_path() / "openpads_cst_test.dat";
    REQUIRE(foil.saveToDatFile(tmpDat));

    opads::Airfoil reloaded;
    REQUIRE(reloaded.loadFromDatFile(tmpDat));
    REQUIRE(reloaded.getMaxThicknessRatio() == Catch::Approx(origThick).margin(0.005f));
    std::filesystem::remove(tmpDat);
}

TEST_CASE("Wing moveCrossSectionLeadingEdge updates span, sweep, and dihedral", "[core][wing]") {
    opads::Wing wing(1, "Handle Test Wing");
    wing.moveCrossSectionLeadingEdge(1, glm::vec3(1.5f, 4.0f, 1.0f));
    const auto& bay = wing.getSection(0);
    const float expectedSpan = std::sqrt(4.0f * 4.0f + 1.0f * 1.0f);
    REQUIRE(bay.span().getFloat() == Catch::Approx(expectedSpan).margin(1e-4f));
    REQUIRE(bay.dihedralDeg().getFloat() == Catch::Approx(glm::degrees(std::atan2(1.0f, 4.0f))).margin(1e-3f));
}

TEST_CASE("RayPicker hits aircraft wing and selects closest cross-section", "[render][picking]") {
    opads::Aircraft aircraft("Pick Test");
    const auto id = aircraft.allocateComponentId();
    auto wingPtr = std::make_unique<opads::Wing>(id, "Pick Wing");
    wingPtr->getSection(0).span().setValue(6.0);
    wingPtr->getSection(0).dihedralDeg().setValue(0.0);
    wingPtr->getSection(0).sweepDeg().setValue(0.0);
    wingPtr->syncSectionsFromBay(0);
    wingPtr->ensureBuilt();
    aircraft.addComponent(std::move(wingPtr));

    // Ray aimed downward near the right wing tip (y = 5.5, x = 1.0, z = 10.0 -> -Z)
    const opads::Ray tipRay{glm::vec3(1.0f, 5.5f, 10.0f), glm::vec3(0.0f, 0.0f, -1.0f)};
    const opads::PickResult pick = opads::RayPicker::pickAircraft(tipRay, aircraft);
    REQUIRE(pick.hit);
    REQUIRE(pick.componentId == id);
    REQUIRE(pick.crossSectionIndex.has_value());
    REQUIRE(*pick.crossSectionIndex == 1);

    // Axis projection test
    const opads::Ray axisRay{glm::vec3(3.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, -1.0f)};
    REQUIRE(opads::Gizmo::projectRayOntoAxis(axisRay, glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f)) == Catch::Approx(3.0f).margin(1e-4f));
}
