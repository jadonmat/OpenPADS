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
#include "core/Fuselage.hpp"
#include "core/ImportedMesh.hpp"
#include "core/LandingGear.hpp"
#include "core/MeshExporter.hpp"
#include "core/MeshImporter.hpp"
#include "core/OpenVspImporter.hpp"
#include "core/Parameter.hpp"
#include "core/Pod.hpp"
#include "core/ProjectSerializer.hpp"
#include "core/Propeller.hpp"
#include "core/Transform.hpp"
#include "core/UndoStack.hpp"
#include "core/Wing.hpp"
#include "render/GeometrySnap.hpp"
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

TEST_CASE("Fuselage builds valid lofted mesh and matches cylinder volume", "[core][fuselage]") {
    opads::Fuselage fuse(1, "Cylinder Fuse");
    fuse.clearSections();
    fuse.length().setValue(10.0);
    fuse.setSmoothLoft(false);
    fuse.setCapEnds(true);
    fuse.setRadialPoints(64);
    fuse.setAxialSubdivisions(2);
    fuse.appendSection(opads::FuselageSection(opads::CrossSectionShape::Circle, 0.0, 2.0, 2.0));
    fuse.appendSection(opads::FuselageSection(opads::CrossSectionShape::Circle, 1.0, 2.0, 2.0));
    fuse.ensureBuilt();

    // Cylinder R=1.0, L=10.0 -> Volume = pi * 1^2 * 10 = 31.4159, Side+Caps Area = 2*pi*1*10 + 2*pi*1^2 = 69.115
    REQUIRE(fuse.getVolume() == Catch::Approx(31.4159).margin(0.3));
    REQUIRE(fuse.getWettedArea() == Catch::Approx(69.115).margin(0.4));
    REQUIRE(fuse.getFinenessRatio() == Catch::Approx(5.0).margin(1e-4));

    const auto& mesh = fuse.getMesh();
    REQUIRE_FALSE(mesh.isEmpty());
    for (const auto& v : mesh.getVertices()) {
        REQUIRE(glm::length(v.normal) == Catch::Approx(1.0f).margin(1e-3f));
    }
}

TEST_CASE("ProjectSerializer round-trips Aircraft and MeshExporter writes STL and OBJ", "[core][io]") {
    opads::Aircraft original("OpenPADS Jet");
    const auto wId = original.allocateComponentId();
    auto wing = std::make_unique<opads::Wing>(wId, "Main Wing");
    wing->getSection(0).span().setValue(7.5);
    wing->getSection(0).getTipAirfoil().setNaca5("23012");
    wing->syncSectionsFromBay(0);
    wing->getTransform().setPosition({4.0f, 0.0f, 0.2f});
    wing->ensureBuilt();
    original.addComponent(std::move(wing));

    const auto fId = original.allocateComponentId();
    auto fuse = std::make_unique<opads::Fuselage>(fId, "Main Fuselage");
    fuse->length().setValue(15.0);
    fuse->markDirty();
    fuse->ensureBuilt();
    original.addComponent(std::move(fuse));

    const std::string jsonText = opads::ProjectSerializer::saveToString(original);
    opads::Aircraft reloaded;
    REQUIRE(opads::ProjectSerializer::loadFromString(reloaded, jsonText));
    REQUIRE(reloaded.getName() == "OpenPADS Jet");
    REQUIRE(reloaded.getComponents().size() == 2);

    const auto* loadedWing = dynamic_cast<const opads::Wing*>(reloaded.getComponents()[0].get());
    REQUIRE(loadedWing != nullptr);
    REQUIRE(loadedWing->getSection(0).span().getValue() == Catch::Approx(7.5));
    REQUIRE(loadedWing->getSection(0).getTipAirfoil().getName() == "NACA 23012");
    REQUIRE(loadedWing->getTransform().getPosition().x == Catch::Approx(4.0f));

    const auto* loadedFuse = dynamic_cast<const opads::Fuselage*>(reloaded.getComponents()[1].get());
    REQUIRE(loadedFuse != nullptr);
    REQUIRE(loadedFuse->length().getValue() == Catch::Approx(15.0));

    // Export binary STL and verify exact byte count = 84 + 50 * totalTriangles
    std::uint32_t expectedTris = 0;
    for (const auto& c : reloaded.getComponents()) {
        expectedTris += static_cast<std::uint32_t>(c->getMesh().getTriangleCount());
    }
    const std::filesystem::path tmpStl = std::filesystem::temp_directory_path() / "openpads_export_test.stl";
    const std::filesystem::path tmpObj = std::filesystem::temp_directory_path() / "openpads_export_test.obj";
    REQUIRE(opads::MeshExporter::exportBinaryStl(reloaded, tmpStl));
    REQUIRE(std::filesystem::file_size(tmpStl) == 84 + static_cast<std::uintmax_t>(expectedTris) * 50);

    REQUIRE(opads::MeshExporter::exportObj(reloaded, tmpObj));
    REQUIRE(std::filesystem::file_size(tmpObj) > 100);
    std::filesystem::remove(tmpStl);
    std::filesystem::remove(tmpObj);
}

TEST_CASE("RayPicker exclusion and Gizmo CrossSection targeting work for surface snapping", "[render][gizmo]") {
    opads::Aircraft aircraft("Snap Test");
    const auto fuseId = aircraft.allocateComponentId();
    auto fuse = std::make_unique<opads::Fuselage>(fuseId, "Fuse");
    fuse->clearSections();
    fuse->length().setValue(10.0);
    fuse->setSmoothLoft(false);
    fuse->appendSection(opads::FuselageSection(opads::CrossSectionShape::Circle, 0.0, 2.0, 2.0));
    fuse->appendSection(opads::FuselageSection(opads::CrossSectionShape::Circle, 1.0, 2.0, 2.0));
    fuse->ensureBuilt();
    aircraft.addComponent(std::move(fuse));

    const auto wingId = aircraft.allocateComponentId();
    auto wing = std::make_unique<opads::Wing>(wingId, "Wing");
    wing->setCrossSectionChord(1, 0.8f);
    wing->setCrossSectionTwistDeg(1, -3.5f);
    wing->setSelectedCrossSection(1);
    wing->ensureBuilt();
    REQUIRE(wing->getCrossSectionChord(1) == Catch::Approx(0.8f));
    REQUIRE(wing->getCrossSectionTwistDeg(1) == Catch::Approx(-3.5f));
    const glm::vec3 tipLE = wing->getCrossSectionLeadingEdge(1);
    aircraft.addComponent(std::move(wing));
    aircraft.setSelectedComponentId(wingId);

    // Excluding wingId allows snapping directly onto the cylinder fuselage surface at y=1.0
    const opads::Ray sideRay{glm::vec3(5.0f, 5.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)};
    const opads::PickResult snapHit = opads::RayPicker::pickAircraft(sideRay, aircraft, wingId);
    REQUIRE(snapHit.hit);
    REQUIRE(snapHit.componentId == fuseId);
    REQUIRE(snapHit.worldPos.y == Catch::Approx(1.0f).margin(0.02f));

    // Gizmo origin switches cleanly between Part and CrossSection
    opads::Gizmo gizmo;
    glm::vec3 origin(0.0f);
    gizmo.setTarget(opads::GizmoTarget::Part);
    REQUIRE(gizmo.getOrigin(aircraft, origin));
    REQUIRE(glm::length(origin) == Catch::Approx(0.0f).margin(1e-5f));

    gizmo.setTarget(opads::GizmoTarget::CrossSection);
    REQUIRE(gizmo.getOrigin(aircraft, origin));
    REQUIRE(origin.x == Catch::Approx(tipLE.x).margin(1e-4f));
    REQUIRE(origin.y == Catch::Approx(tipLE.y).margin(1e-4f));
    REQUIRE(origin.z == Catch::Approx(tipLE.z).margin(1e-4f));
}

TEST_CASE("GeometrySnap snaps moving wing flush to fuselage surface and UndoStack reverses edits", "[render][snap][undo]") {
    opads::Aircraft aircraft("Snap & Undo");
    opads::UndoStack undo;

    const auto fuseId = aircraft.allocateComponentId();
    auto fuse = std::make_unique<opads::Fuselage>(fuseId, "Fuse");
    fuse->clearSections();
    fuse->length().setValue(10.0);
    fuse->setSmoothLoft(false);
    fuse->appendSection(opads::FuselageSection(opads::CrossSectionShape::Circle, 0.0, 2.0, 2.0));
    fuse->appendSection(opads::FuselageSection(opads::CrossSectionShape::Circle, 1.0, 2.0, 2.0));
    fuse->ensureBuilt();
    aircraft.addComponent(std::move(fuse));

    const auto wingId = aircraft.allocateComponentId();
    auto wing = std::make_unique<opads::Wing>(wingId, "Winglet");
    wing->setSymmetry(opads::SymmetryMode::None);
    wing->getSection(0).span().setValue(2.0);
    wing->getSection(0).dihedralDeg().setValue(0.0);
    wing->getSection(0).sweepDeg().setValue(0.0);
    wing->syncSectionsFromBay(0);
    wing->ensureBuilt();
    opads::Component* wingPtr = aircraft.addComponent(std::move(wing));
    aircraft.setSelectedComponentId(wingId);

    // Move wing near y=1.18 (just 0.18m outside cylinder surface y=1.0), GeometrySnap along Y should snap flush to y=1.0
    const opads::SnapResult snap = opads::GeometrySnap::snapPartAlongAxis(
        *wingPtr,
        glm::vec3(4.0f, 1.18f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        aircraft,
        0.5f
    );
    REQUIRE(snap.snapped);
    REQUIRE(snap.snappedPosition.y == Catch::Approx(1.0f).margin(0.03f));

    // Undo / Redo verification
    undo.beginTransaction(aircraft, "Move Wing");
    wingPtr->getTransform().setPosition(snap.snappedPosition);
    REQUIRE(undo.commitTransaction(aircraft));
    REQUIRE(undo.canUndo());

    REQUIRE(undo.undo(aircraft));
    REQUIRE(aircraft.findComponent(wingId)->getTransform().getPosition().y == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(undo.canRedo());

    REQUIRE(undo.redo(aircraft));
    REQUIRE(aircraft.findComponent(wingId)->getTransform().getPosition().y == Catch::Approx(snap.snappedPosition.y).margin(1e-4f));
}

TEST_CASE("Phase 2 components build valid meshes and support ControlSurfaces", "[core][phase2]") {
    opads::Wing wing(1, "Flapped Wing");
    wing.addControlSurface(opads::ControlSurface("Aileron", 0, 0.55, 0.90, 0.25, 18.0, false));
    wing.addControlSurface(opads::ControlSurface("Flap", 0, 0.10, 0.45, 0.28, 25.0, true));
    wing.ensureBuilt();
    REQUIRE(wing.getControlSurfaceCount() == 2);
    REQUIRE_FALSE(wing.getMesh().isEmpty());
    // 1 selected XSec * 2 (XZ) + 2 control surfaces * 2 (XZ) = 6 highlight loops
    REQUIRE(wing.getHighlightLoops().size() == 6);

    opads::Pod pod(2, "Turbine Pod");
    pod.setSymmetry(opads::SymmetryMode::XZ);
    pod.ensureBuilt();
    REQUIRE(pod.getWettedArea() > 1.0);
    REQUIRE(pod.getVolume() > 0.2);
    for (const auto& v : pod.getMesh().getVertices()) {
        REQUIRE(glm::length(v.normal) == Catch::Approx(1.0f).margin(1e-3f));
    }

    opads::Propeller prop(3, "3-Blade Prop");
    prop.setBladeCount(4);
    prop.ensureBuilt();
    REQUIRE(prop.getBladeCount() == 4);
    REQUIRE(prop.getDiskArea() == Catch::Approx(3.14159 * 1.05 * 1.05).margin(1e-2));
    REQUIRE(prop.getSolidity() > 0.05);
    REQUIRE_FALSE(prop.getMesh().isEmpty());

    opads::LandingGear gear(4, "Main Gear");
    gear.setWheelsPerStrut(2);
    gear.setWheelPant(true);
    gear.ensureBuilt();
    REQUIRE(gear.getFrontalArea() > 0.1);
    REQUIRE(gear.getWettedArea() > 1.0);
    REQUIRE_FALSE(gear.getMesh().isEmpty());
}

TEST_CASE("OpenVspImporter parses .vsp3 XML and MeshImporter converts STL/OBJ with unit scaling", "[core][import]") {
    constexpr const char* kSampleVsp3 = R"(
<Vsp_Geometry>
  <Vehicle>
    <Geom>
      <ParmContainer>
        <Name Value="MainWing"/>
        <XForm>
          <X_Location Value="3.5"/>
          <Y_Location Value="0.0"/>
          <Z_Location Value="0.4"/>
        </XForm>
        <Sym>
          <Sym_Planar_Flag Value="2"/>
        </Sym>
      </ParmContainer>
      <GeomBase><TypeName>Wing</TypeName></GeomBase>
      <WingGeom>
        <XSecSurf>
          <XSec>
            <XSecCurve><ParmContainer><ThickChord Value="0.12"/><Camber Value="0.02"/><CamberLoc Value="0.4"/></ParmContainer></XSecCurve>
          </XSec>
          <XSec>
            <ParmContainer><Span Value="6.5"/><Root_Chord Value="2.0"/><Tip_Chord Value="1.0"/><Sweep Value="14.0"/><Dihedral Value="5.0"/></ParmContainer>
            <XSecCurve><ParmContainer><ThickChord Value="0.10"/><Camber Value="0.02"/><CamberLoc Value="0.4"/></ParmContainer></XSecCurve>
          </XSec>
        </XSecSurf>
      </WingGeom>
    </Geom>
    <Geom>
      <ParmContainer>
        <Name Value="Fuse"/>
        <Design><Length Value="11.0"/></Design>
      </ParmContainer>
      <GeomBase><TypeName>Fuselage</TypeName></GeomBase>
      <FuselageGeom>
        <XSecSurf>
          <XSec><ParmContainer><XLocPercent Value="0.0"/></ParmContainer><XSecCurve Type="0"/></XSec>
          <XSec><ParmContainer><XLocPercent Value="0.5"/></ParmContainer><XSecCurve Type="2"><ParmContainer><Width Value="1.6"/><Height Value="1.8"/></ParmContainer></XSecCurve></XSec>
          <XSec><ParmContainer><XLocPercent Value="1.0"/></ParmContainer><XSecCurve Type="0"/></XSec>
        </XSecSurf>
      </FuselageGeom>
    </Geom>
  </Vehicle>
</Vsp_Geometry>
)";

    opads::Aircraft vspAircraft;
    REQUIRE(opads::OpenVspImporter::importFromString(vspAircraft, kSampleVsp3, "VspTest"));
    REQUIRE(vspAircraft.getComponents().size() == 2);

    const auto* vspWing = dynamic_cast<const opads::Wing*>(vspAircraft.getComponents()[0].get());
    REQUIRE(vspWing != nullptr);
    REQUIRE(vspWing->getTransform().getPosition().x == Catch::Approx(3.5f));
    REQUIRE(vspWing->getSection(0).span().getValue() == Catch::Approx(6.5));
    REQUIRE(vspWing->getSection(0).getRootAirfoil().getName() == "NACA 2412");

    // Export STL & OBJ, re-import with MeshImporter + mm->m scaling + duplicate + JSON round-trip
    const std::filesystem::path tmpStl = std::filesystem::temp_directory_path() / "openpads_phase2_import.stl";
    const std::filesystem::path tmpObj = std::filesystem::temp_directory_path() / "openpads_phase2_import.obj";
    REQUIRE(opads::MeshExporter::exportBinaryStl(vspAircraft, tmpStl));
    REQUIRE(opads::MeshExporter::exportObj(vspAircraft, tmpObj));

    opads::Aircraft meshAircraft("Imported Test");
    REQUIRE(opads::MeshImporter::importAuto(meshAircraft, tmpStl, 0.001f, opads::UpAxisConversion::YUpToZUp));
    REQUIRE(opads::MeshImporter::importAuto(meshAircraft, tmpObj, 1.0f, opads::UpAxisConversion::NoneZUp));
    REQUIRE(meshAircraft.getComponents().size() >= 2);

    auto* impMesh = dynamic_cast<opads::ImportedMesh*>(meshAircraft.getComponents().front().get());
    REQUIRE(impMesh != nullptr);
    REQUIRE(impMesh->getUnitScale() == Catch::Approx(0.001f));
    REQUIRE(impMesh->getAxisConversion() == opads::UpAxisConversion::YUpToZUp);

    // Duplicate & ProjectSerializer round-trip across all component types
    REQUIRE(opads::ProjectSerializer::duplicateComponent(vspAircraft, vspWing->getId()) != nullptr);
    vspAircraft.addComponent(std::make_unique<opads::Pod>(vspAircraft.allocateComponentId(), "Pod1"));
    vspAircraft.addComponent(std::make_unique<opads::Propeller>(vspAircraft.allocateComponentId(), "Prop1"));
    vspAircraft.addComponent(std::make_unique<opads::LandingGear>(vspAircraft.allocateComponentId(), "Gear1"));

    const std::string saved = opads::ProjectSerializer::saveToString(vspAircraft);
    opads::Aircraft reloaded;
    REQUIRE(opads::ProjectSerializer::loadFromString(reloaded, saved));
    REQUIRE(reloaded.getComponents().size() == 6);

    std::filesystem::remove(tmpStl);
    std::filesystem::remove(tmpObj);
}
