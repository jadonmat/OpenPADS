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

TEST_CASE("OpenVspImporter accurately imports nested Fuselage, Pod, Propeller PCurves, Gear, and LenUnit", "[core][import][vsp3]") {
    constexpr const char* kNestedVsp3 = R"(
<Vsp_Geometry>
  <Vehicle>
    <ParmContainer>
      <STEPSettings><LenUnit Value="4.0"/></STEPSettings>
    </ParmContainer>
    <Geom>
      <ParmContainer>
        <Name Value="Fuse"/>
        <Design><Length Value="87.0"/></Design>
      </ParmContainer>
      <GeomBase><TypeName>Fuselage</TypeName></GeomBase>
      <FuselageGeom>
        <XSecSurf>
          <XSec>
            <ParmContainer><XSec><RefLength Value="87.0"/><XLocPercent Value="0.0"/><ZLocPercent Value="-0.03"/></XSec></ParmContainer>
            <XSec><XSecCurve><XSecCurve><Type>0</Type></XSecCurve><ParmContainer><XSecCurve/></ParmContainer></XSecCurve></XSec>
          </XSec>
          <XSec>
            <ParmContainer><XSec><RefLength Value="87.0"/><XLocPercent Value="0.5"/><ZLocPercent Value="0.0"/></XSec></ParmContainer>
            <XSec><XSecCurve><XSecCurve><Type>5</Type></XSecCurve><ParmContainer><XSecCurve><Width Value="8.75"/><Height Value="9.84"/><TopStr Value="1.50"/><BotStr Value="0.83"/><MaxWidthLoc Value="-0.30"/></XSecCurve></ParmContainer></XSecCurve></XSec>
          </XSec>
          <XSec>
            <ParmContainer><XSec><RefLength Value="87.0"/><XLocPercent Value="1.0"/><ZLocPercent Value="0.05"/></XSec></ParmContainer>
            <XSec><XSecCurve><XSecCurve><Type>1</Type></XSecCurve><ParmContainer><XSecCurve><Circle_Diameter Value="2.0"/></XSecCurve></ParmContainer></XSecCurve></XSec>
          </XSec>
        </XSecSurf>
      </FuselageGeom>
    </Geom>
    <Geom>
      <ParmContainer>
        <Name Value="EnginePod"/>
        <Design><Length Value="10.0"/><FineRatio Value="10.0"/></Design>
      </ParmContainer>
      <GeomBase><TypeName>Pod</TypeName></GeomBase>
      <PodGeom/>
    </Geom>
    <Geom>
      <ParmContainer>
        <Name Value="Prop"/>
        <XForm><X_Location Value="30.0"/><Y_Location Value="10.0"/><Z_Location Value="2.0"/></XForm>
        <Design><Diameter Value="10.0"/><NumBlade Value="6.0"/><Feather Value="-10.0"/><ReverseFlag Value="0.0"/></Design>
      </ParmContainer>
      <GeomBase><TypeName>Propeller</TypeName></GeomBase>
      <PropellerGeom>
        <Chord><ParmContainer><Chord><r_0 Value="0.2"/><crd_0 Value="0.10"/><r_1 Value="0.6"/><crd_1 Value="0.22"/><r_2 Value="1.0"/><crd_2 Value="0.12"/></Chord></ParmContainer></Chord>
        <Twist><ParmContainer><Twist><r_0 Value="0.2"/><tw_0 Value="45.0"/><r_1 Value="0.75"/><tw_1 Value="30.0"/><r_2 Value="1.0"/><tw_2 Value="22.0"/></Twist></ParmContainer></Twist>
        <XSecSurf>
          <XSec><ParmContainer><XSec><RadiusFrac Value="0.20"/></XSec></ParmContainer><XSec><XSecCurve><XSecCurve><Type>2</Type></XSecCurve></XSecCurve></XSec></XSec>
          <XSec><ParmContainer><XSec><RadiusFrac Value="0.50"/></XSec></ParmContainer><XSec><XSecCurve><XSecCurve><Type>18</Type></XSecCurve><ParmContainer><XSecCurve><ThickChord Value="0.14"/><IdealCl Value="0.6"/></XSecCurve></ParmContainer></XSecCurve></XSec></XSec>
        </XSecSurf>
      </PropellerGeom>
    </Geom>
    <Geom>
      <ParmContainer>
        <Name Value="GearGeom"/>
        <XForm><Z_Location Value="-7.0"/></XForm>
        <Gear><m_ModelLenUnits Value="4.0"/></Gear>
      </ParmContainer>
      <GeomBase><TypeName>Gear</TypeName></GeomBase>
      <Gear>
        <Bogie_0>
          <ParmContainer>
            <Bogie><NumAcross Value="2.0"/><XContactPt Value="10.0"/><YContactPt Value="0.0"/><ZAboveGround Value="0.0"/><TravelZ Value="1.0"/><Symmetrical Value="0.0"/></Bogie>
            <Tire><DiameterModel Value="2.0"/><WidthModel Value="0.6"/></Tire>
          </ParmContainer>
        </Bogie_0>
      </Gear>
    </Geom>
  </Vehicle>
</Vsp_Geometry>
)";

    opads::Aircraft ac;
    REQUIRE(opads::OpenVspImporter::importFromString(ac, kNestedVsp3, "Nested"));
    REQUIRE(ac.getComponents().size() == 4);

    // 1. Fuselage in feet converted to meters: 87 ft * 0.3048 = 26.5176 m
    const auto* fuse = dynamic_cast<const opads::Fuselage*>(ac.getComponents()[0].get());
    REQUIRE(fuse != nullptr);
    REQUIRE(fuse->length().getValue() == Catch::Approx(87.0 * 0.3048).margin(1e-3));
    REQUIRE(fuse->getSectionCount() == 3);
    REQUIRE(fuse->getSection(1).width().getValue() == Catch::Approx(8.75 * 0.3048).margin(1e-3));
    REQUIRE(fuse->getSection(1).height().getValue() == Catch::Approx(9.84 * 0.3048).margin(1e-3));
    REQUIRE(fuse->getSection(1).getShape() == opads::CrossSectionShape::GeneralFuse);
    REQUIRE(fuse->getSection(1).getGeneralFuse().botStr < fuse->getSection(1).getGeneralFuse().topStr);
    REQUIRE(fuse->getSection(2).getShape() == opads::CrossSectionShape::Circle);
    REQUIRE(fuse->getSection(2).width().getValue() == Catch::Approx(2.0 * 0.3048).margin(1e-3));

    // 2. PodGeom: diameter = 2 * length / FineRatio (10 ft / 10 * 2 = 2 ft = 0.6096 m)
    const auto* pod = dynamic_cast<const opads::Pod*>(ac.getComponents()[1].get());
    REQUIRE(pod != nullptr);
    REQUIRE(pod->length().getValue() == Catch::Approx(10.0 * 0.3048).margin(1e-3));
    REQUIRE(pod->width().getValue() == Catch::Approx(2.0 * 0.3048).margin(1e-3));

    // 3. Propeller: Diameter = 10 ft = 3.048 m, R = 1.524 m, 6 blades, curve preserved, no huge nose spinner
    const auto* prop = dynamic_cast<const opads::Propeller*>(ac.getComponents()[2].get());
    REQUIRE(prop != nullptr);
    REQUIRE(prop->diameter().getValue() == Catch::Approx(10.0 * 0.3048).margin(1e-3));
    REQUIRE(prop->getBladeCount() == 6);
    REQUIRE_FALSE(prop->hasSpinner());
    REQUIRE(prop->getBladeCurve().size() >= 10);
    // Max chord 0.22 * R = 0.22 * 1.524 = 0.33528 m, pitch@0.75 = 30 - 10 = 20 deg
    REQUIRE(prop->collectivePitchDeg().getValue() == Catch::Approx(20.0).margin(0.5));
    REQUIRE(prop->hubDiameter().getValue() == Catch::Approx(0.20 * 10.0 * 0.3048).margin(1e-3));

    // 4. LandingGear: imported from Bogie_0 with twin wheels, Diameter = 2 ft = 0.6096 m
    const auto* gear = dynamic_cast<const opads::LandingGear*>(ac.getComponents()[3].get());
    REQUIRE(gear != nullptr);
    REQUIRE(gear->getWheelsPerStrut() == 2);
    REQUIRE(gear->wheelDiameter().getValue() == Catch::Approx(2.0 * 0.3048).margin(1e-3));

    // Round-trip save/load preserves asymmetric GeneralFuse section and Propeller bladeCurve
    const std::string jsonSaved = opads::ProjectSerializer::saveToString(ac);
    opads::Aircraft acReloaded;
    REQUIRE(opads::ProjectSerializer::loadFromString(acReloaded, jsonSaved));
    const auto* propReloaded = dynamic_cast<const opads::Propeller*>(acReloaded.getComponents()[2].get());
    REQUIRE(propReloaded != nullptr);
    REQUIRE(propReloaded->getBladeCurve().size() == prop->getBladeCurve().size());
    REQUIRE_FALSE(propReloaded->hasSpinner());

    // If the reference .vsp3 file exists on disk, verify every component imports cleanly at true scale
    const std::filesystem::path refVsp =
        R"(C:\PROJECTS\EVERGGREENLABS\Flight Sim Assets\Aircraft and related assets Models\Aircraft-Blender\OpenVSP\FinalConfiguration (2) (1).vsp3)";
    if (std::filesystem::exists(refVsp)) {
        opads::Aircraft realAc;
        REQUIRE(opads::OpenVspImporter::importFromFile(realAc, refVsp));
        REQUIRE(realAc.getComponents().size() >= 12);

        auto findByName = [&](std::string_view name) -> const opads::Component* {
            for (const auto& c : realAc.getComponents()) {
                if (c->getName() == name) {
                    return c.get();
                }
            }
            return nullptr;
        };

        const opads::Component* mainFuse = findByName("Fuselage");
        REQUIRE(mainFuse != nullptr);
        const auto* mainFuseTyped = dynamic_cast<const opads::Fuselage*>(mainFuse);
        REQUIRE(mainFuseTyped != nullptr);
        REQUIRE(mainFuseTyped->length().getValue() == Catch::Approx(87.0 * 0.3048).margin(0.05));
        REQUIRE(mainFuseTyped->getMaxWidth() == Catch::Approx(8.75 * 0.3048).margin(0.05));
        REQUIRE(mainFuseTyped->getSection(0).getSkinning().enabled);
        REQUIRE(mainFuseTyped->getSection(0).getSkinning().topRAngleDeg == Catch::Approx(45.0).margin(0.1));
        REQUIRE(mainFuseTyped->getSection(1).getShape() == opads::CrossSectionShape::GeneralFuse);
        REQUIRE(mainFuseTyped->getSection(1).getMaxWidthLocFrac() == Catch::Approx(-0.3048).margin(1e-3));
        REQUIRE(mainFuseTyped->getTailCapType() == opads::EndCapType::Round);

        // Verify imported OpenVSP control surfaces (SS_Control) and tip caps
        const auto* mainWing = dynamic_cast<const opads::Wing*>(findByName("MainWingBody"));
        REQUIRE(mainWing != nullptr);
        REQUIRE(mainWing->getControlSurfaceCount() == 1);
        REQUIRE(mainWing->getControlSurface(0).spanStartFrac().getValue() == Catch::Approx(0.725).margin(1e-3));
        REQUIRE(mainWing->getControlSurface(0).spanEndFrac().getValue() == Catch::Approx(0.95).margin(1e-3));

        const auto* vTail = dynamic_cast<const opads::Wing*>(findByName("VerticalStabilizer"));
        REQUIRE(vTail != nullptr);
        REQUIRE(vTail->getTipCapType() == opads::EndCapType::Round);
        REQUIRE(vTail->getControlSurfaceCount() == 1);

        const auto* hTail = dynamic_cast<const opads::Wing*>(findByName("HorizStab"));
        REQUIRE(hTail != nullptr);
        REQUIRE(hTail->getControlSurfaceCount() == 1);

        // Tail RoundedRect cross section should close seamlessly without a seam jump
        const auto tailLoop = mainFuseTyped->getSection(5).sampleLoopYZ(32);
        REQUIRE(glm::length(tailLoop.front() - tailLoop.back()) < 1e-4f);

        const auto* leftEngine = dynamic_cast<const opads::Fuselage*>(findByName("LeftEngine"));
        REQUIRE(leftEngine != nullptr);
        REQUIRE(leftEngine->getMaxHeight() == Catch::Approx(3.75 * 0.3048).margin(0.05));

        const auto* leftProp = dynamic_cast<const opads::Propeller*>(findByName("LeftProp"));
        REQUIRE(leftProp != nullptr);
        REQUIRE(leftProp->getBladeCount() == 8);
        REQUIRE(leftProp->diameter().getValue() == Catch::Approx(13.48425 * 0.3048).margin(0.05));
        REQUIRE(leftProp->getSolidity() > 0.35);
        REQUIRE(leftProp->getBladeCurve().front().thicknessRatio > 0.35);
        REQUIRE(leftProp->getBladeCurve().back().thicknessRatio < 0.06);
        REQUIRE(std::abs(leftProp->getBladeCurve().back().sweepOffset) > 0.1);

        // Verify all imported components produce valid finite unit normals and triangles
        for (const auto& comp : realAc.getComponents()) {
            const auto& m = comp->getMesh();
            REQUIRE(m.getTriangleCount() > 0);
            for (const opads::Vertex& v : m.getVertices()) {
                REQUIRE(std::isfinite(v.position.x));
                REQUIRE(std::isfinite(v.position.y));
                REQUIRE(std::isfinite(v.position.z));
                REQUIRE(glm::length(v.normal) == Catch::Approx(1.0f).margin(1e-2f));
            }
        }
    }
}

TEST_CASE("Fuselage Hermite skinning and RoundedRect seam closure produce valid geometry", "[core][fuselage][propeller]") {
    // RoundedRect taller than wide closes identically at front and back
    opads::FuselageSection rr(opads::CrossSectionShape::RoundedRect, 1.0, 0.25, 0.72);
    rr.cornerRadiusFrac().setValue(0.5);
    const auto rrLoop = rr.sampleLoopYZ(32);
    REQUIRE(rrLoop.size() == 33);
    REQUIRE(glm::length(rrLoop.front() - rrLoop.back()) < 1e-5f);

    // 45-deg Hermite nose should be significantly fuller (larger volume) than collapsed unskinned nose
    auto buildNoseTest = [](std::uint64_t compId, bool enableSkin) {
        auto f = std::make_unique<opads::Fuselage>(compId, "NoseTest");
        f->clearSections();
        f->length().setValue(10.0);
        f->setAxialSubdivisions(8);
        f->setRadialPoints(24);
        opads::FuselageSection s0(opads::CrossSectionShape::Point, 0.0, 0.0, 0.0, 0.0);
        opads::FuselageSection s1(opads::CrossSectionShape::Circle, 0.25, 2.0, 2.0, 0.0);
        opads::FuselageSection s2(opads::CrossSectionShape::Circle, 1.0, 2.0, 2.0, 0.0);
        if (enableSkin) {
            s0.skinning().enabled = true;
            s0.skinning().topRAngleDeg = s0.skinning().bottomRAngleDeg =
                s0.skinning().leftRAngleDeg = s0.skinning().rightRAngleDeg = 55.0;
            s1.skinning().enabled = true;
        }
        f->appendSection(s0);
        f->appendSection(s1);
        f->appendSection(s2);
        f->ensureBuilt();
        return f;
    };

    const auto unskinned = buildNoseTest(1, false);
    auto skinned = buildNoseTest(2, true);
    REQUIRE(skinned->getVolume() > unskinned->getVolume() + 0.5);

    // Skinning round-trip serialization
    opads::Aircraft ac("SkinRoundTrip");
    ac.addComponent(buildNoseTest(ac.allocateComponentId(), true));
    const std::string saved = opads::ProjectSerializer::saveToString(ac);
    opads::Aircraft reloaded;
    REQUIRE(opads::ProjectSerializer::loadFromString(reloaded, saved));
    const auto* fReloaded = dynamic_cast<const opads::Fuselage*>(reloaded.getComponents()[0].get());
    REQUIRE(fReloaded != nullptr);
    REQUIRE(fReloaded->getSection(0).getSkinning().enabled);
    REQUIRE(fReloaded->getSection(0).getSkinning().topRAngleDeg == Catch::Approx(55.0));

    // Propeller blades must have outward-facing triangle winding for both CW and CCW rotation
    for (bool cw : {true, false}) {
        opads::Propeller prop(2, "OutwardProp");
        prop.setClockwise(cw);
        prop.setBladeCount(2);
        prop.ensureBuilt();
        const auto& verts = prop.getMesh().getVertices();
        const auto& idx = prop.getMesh().getIndices();
        int nonDegen = 0;
        int outwardScore = 0;
        for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
            const glm::vec3& a = verts[idx[t]].position;
            const glm::vec3& b = verts[idx[t + 1]].position;
            const glm::vec3& c = verts[idx[t + 2]].position;
            const glm::vec3 fn = glm::cross(b - a, c - a);
            if (glm::length(fn) < 1e-8f) {
                continue;
            }
            ++nonDegen;
            const glm::vec3 avgVertN = verts[idx[t]].normal + verts[idx[t + 1]].normal + verts[idx[t + 2]].normal;
            if (glm::dot(fn, avgVertN) > 0.0f) {
                ++outwardScore;
            }
        }
        REQUIRE(nonDegen > 100);
        REQUIRE(outwardScore == nonDegen);
    }
}
