#include "core/ProjectSerializer.hpp"

#include <fstream>
#include <sstream>
#include <vector>

#include <nlohmann/json.hpp>

#include "core/Airfoil.hpp"
#include "core/Fuselage.hpp"
#include "core/ImportedMesh.hpp"
#include "core/LandingGear.hpp"
#include "core/Pod.hpp"
#include "core/Propeller.hpp"
#include "core/Wing.hpp"

namespace opads {

namespace {

using json = nlohmann::json;

json vec3ToJson(const glm::vec3& v) {
    return json::array({v.x, v.y, v.z});
}

glm::vec3 jsonToVec3(const json& j, const glm::vec3& fallback) {
    if (!j.is_array() || j.size() != 3) {
        return fallback;
    }
    return glm::vec3(j[0].get<float>(), j[1].get<float>(), j[2].get<float>());
}

std::string airfoilTypeToString(AirfoilType t) {
    switch (t) {
    case AirfoilType::Naca4: return "naca4";
    case AirfoilType::Naca5: return "naca5";
    case AirfoilType::DatFile: return "dat";
    case AirfoilType::Cst: return "cst";
    }
    return "naca4";
}

std::string shapeToString(CrossSectionShape shape) {
    switch (shape) {
    case CrossSectionShape::Point: return "point";
    case CrossSectionShape::Circle: return "circle";
    case CrossSectionShape::Ellipse: return "ellipse";
    case CrossSectionShape::SuperEllipse: return "super_ellipse";
    case CrossSectionShape::RoundedRect: return "rounded_rect";
    }
    return "ellipse";
}

CrossSectionShape stringToShape(const std::string& s) {
    if (s == "point") return CrossSectionShape::Point;
    if (s == "circle") return CrossSectionShape::Circle;
    if (s == "super_ellipse") return CrossSectionShape::SuperEllipse;
    if (s == "rounded_rect") return CrossSectionShape::RoundedRect;
    return CrossSectionShape::Ellipse;
}

json serializeAirfoil(const Airfoil& foil) {
    json j;
    j["type"] = airfoilTypeToString(foil.getType());
    j["name"] = foil.getName();
    j["naca4"] = foil.getNaca4Code();
    j["naca5"] = foil.getNaca5Code();
    j["datPath"] = foil.getDatPath();
    j["thicknessScale"] = foil.getThicknessScale();
    j["camberScale"] = foil.getCamberScale();
    j["teGap"] = foil.getTeGap();
    j["sharpTE"] = foil.isSharpTE();
    j["cstUpper"] = foil.getCstUpperWeights();
    j["cstLower"] = foil.getCstLowerWeights();
    if (foil.getType() == AirfoilType::DatFile) {
        j["datFallback"] = foil.toDatString();
    }
    return j;
}

void deserializeAirfoil(const json& j, Airfoil& foil) {
    const std::string type = j.value("type", "naca4");
    const std::string name = j.value("name", "Airfoil");

    if (type == "naca5") {
        foil.setNaca5(j.value("naca5", "23012"));
    } else if (type == "cst" && j.contains("cstUpper") && j.contains("cstLower")) {
        foil.setCst(
            j["cstUpper"].get<std::vector<double>>(),
            j["cstLower"].get<std::vector<double>>(),
            name
        );
    } else if (type == "dat") {
        const std::string datPath = j.value("datPath", "");
        bool loaded = !datPath.empty() && foil.loadFromDatFile(datPath);
        if (!loaded && j.contains("datFallback")) {
            loaded = foil.loadFromDatString(j["datFallback"].get<std::string>(), name);
        }
        if (!loaded) {
            foil.setNaca4("2412");
        }
    } else {
        foil.setNaca4(j.value("naca4", "2412"));
    }

    foil.setSharpTE(j.value("sharpTE", true));
    foil.setThicknessScale(j.value("thicknessScale", 1.0f));
    foil.setCamberScale(j.value("camberScale", 1.0f));
    foil.setTeGap(j.value("teGap", 0.0f));
}

json serializeTransform(const Transform& t) {
    return json{
        {"position", vec3ToJson(t.getPosition())},
        {"rotationDeg", vec3ToJson(t.getRotationDeg())},
        {"scale", vec3ToJson(t.getScale())}
    };
}

void deserializeTransform(const json& j, Transform& t) {
    if (!j.is_object()) {
        return;
    }
    if (j.contains("position")) {
        t.setPosition(jsonToVec3(j["position"], glm::vec3(0.0f)));
    }
    if (j.contains("rotationDeg")) {
        t.setRotationDeg(jsonToVec3(j["rotationDeg"], glm::vec3(0.0f)));
    }
    if (j.contains("scale")) {
        t.setScale(jsonToVec3(j["scale"], glm::vec3(1.0f)));
    }
}

json serializeWing(const Wing& wing) {
    json bays = json::array();
    for (const WingSection& sec : wing.getSections()) {
        bays.push_back(json{
            {"span", sec.span().getValue()},
            {"rootChord", sec.rootChord().getValue()},
            {"tipChord", sec.tipChord().getValue()},
            {"sweepDeg", sec.sweepDeg().getValue()},
            {"sweepLocation", sec.sweepLocation().getValue()},
            {"dihedralDeg", sec.dihedralDeg().getValue()},
            {"rootTwistDeg", sec.rootTwistDeg().getValue()},
            {"tipTwistDeg", sec.tipTwistDeg().getValue()},
            {"twistLocation", sec.twistLocation().getValue()},
            {"spanSubdivisions", sec.getSpanSubdivisions()},
            {"rootAirfoil", serializeAirfoil(sec.getRootAirfoil())},
            {"tipAirfoil", serializeAirfoil(sec.getTipAirfoil())}
        });
    }

    json controls = json::array();
    for (const ControlSurface& cs : wing.getControlSurfaces()) {
        controls.push_back(json{
            {"name", cs.getName()},
            {"bayIndex", cs.getBayIndex()},
            {"spanStartFrac", cs.spanStartFrac().getValue()},
            {"spanEndFrac", cs.spanEndFrac().getValue()},
            {"chordFrac", cs.chordFrac().getValue()},
            {"deflectionDeg", cs.deflectionDeg().getValue()},
            {"symmetric", cs.isSymmetric()}
        });
    }

    return json{
        {"type", "Wing"},
        {"name", wing.getName()},
        {"visible", wing.isVisible()},
        {"symmetry", wing.getSymmetry() == SymmetryMode::XZ ? "XZ" : "None"},
        {"capTips", wing.getCapTips()},
        {"chordPoints", wing.getChordPoints()},
        {"transform", serializeTransform(wing.getTransform())},
        {"sections", bays},
        {"controlSurfaces", controls}
    };
}

json serializeFuselage(const Fuselage& fuse) {
    json sections = json::array();
    for (const FuselageSection& sec : fuse.getSections()) {
        sections.push_back(json{
            {"shape", shapeToString(sec.getShape())},
            {"stationFrac", sec.stationFrac().getValue()},
            {"width", sec.width().getValue()},
            {"height", sec.height().getValue()},
            {"yOffset", sec.yOffset().getValue()},
            {"zOffset", sec.zOffset().getValue()},
            {"superEllipseN", sec.superEllipseN().getValue()},
            {"cornerRadiusFrac", sec.cornerRadiusFrac().getValue()}
        });
    }

    return json{
        {"type", "Fuselage"},
        {"name", fuse.getName()},
        {"visible", fuse.isVisible()},
        {"length", fuse.length().getValue()},
        {"smoothLoft", fuse.isSmoothLoft()},
        {"capEnds", fuse.getCapEnds()},
        {"radialPoints", fuse.getRadialPoints()},
        {"axialSubdivisions", fuse.getAxialSubdivisions()},
        {"transform", serializeTransform(fuse.getTransform())},
        {"sections", sections}
    };
}

json serializePod(const Pod& pod) {
    return json{
        {"type", "Pod"},
        {"name", pod.getName()},
        {"visible", pod.isVisible()},
        {"length", pod.length().getValue()},
        {"width", pod.width().getValue()},
        {"height", pod.height().getValue()},
        {"noseFrac", pod.noseFrac().getValue()},
        {"tailFrac", pod.tailFrac().getValue()},
        {"nosePower", pod.nosePower().getValue()},
        {"tailPower", pod.tailPower().getValue()},
        {"superEllipseN", pod.superEllipseN().getValue()},
        {"symmetry", pod.getSymmetry() == SymmetryMode::XZ ? "XZ" : "None"},
        {"radialPoints", pod.getRadialPoints()},
        {"axialStations", pod.getAxialStations()},
        {"transform", serializeTransform(pod.getTransform())}
    };
}

json serializePropeller(const Propeller& prop) {
    return json{
        {"type", "Propeller"},
        {"name", prop.getName()},
        {"visible", prop.isVisible()},
        {"diameter", prop.diameter().getValue()},
        {"hubDiameter", prop.hubDiameter().getValue()},
        {"spinnerLength", prop.spinnerLength().getValue()},
        {"rootChord", prop.rootChord().getValue()},
        {"tipChord", prop.tipChord().getValue()},
        {"collectivePitchDeg", prop.collectivePitchDeg().getValue()},
        {"washoutDeg", prop.washoutDeg().getValue()},
        {"sweepDeg", prop.sweepDeg().getValue()},
        {"bladeCount", prop.getBladeCount()},
        {"clockwise", prop.isClockwise()},
        {"showDiskRing", prop.getShowDiskRing()},
        {"airfoil", serializeAirfoil(prop.getBladeAirfoil())},
        {"transform", serializeTransform(prop.getTransform())}
    };
}

json serializeLandingGear(const LandingGear& gear) {
    return json{
        {"type", "LandingGear"},
        {"name", gear.getName()},
        {"visible", gear.isVisible()},
        {"strutLength", gear.strutLength().getValue()},
        {"strutDiameter", gear.strutDiameter().getValue()},
        {"rakeDeg", gear.rakeDeg().getValue()},
        {"splayDeg", gear.splayDeg().getValue()},
        {"wheelDiameter", gear.wheelDiameter().getValue()},
        {"wheelWidth", gear.wheelWidth().getValue()},
        {"wheelsPerStrut", gear.getWheelsPerStrut()},
        {"wheelPant", gear.hasWheelPant()},
        {"symmetry", gear.getSymmetry() == SymmetryMode::XZ ? "XZ" : "None"},
        {"transform", serializeTransform(gear.getTransform())}
    };
}

json serializeImportedMesh(const ImportedMesh& imp) {
    json posArr = json::array();
    for (const glm::vec3& p : imp.getRawPositions()) {
        posArr.push_back(p.x);
        posArr.push_back(p.y);
        posArr.push_back(p.z);
    }
    return json{
        {"type", "ImportedMesh"},
        {"name", imp.getName()},
        {"visible", imp.isVisible()},
        {"sourcePath", imp.getSourcePath()},
        {"unitScale", imp.getUnitScale()},
        {"axisConversion", imp.getAxisConversion() == UpAxisConversion::YUpToZUp ? "YUpToZUp" : "ZUp"},
        {"centerOrigin", imp.isCenterOrigin()},
        {"positions", std::move(posArr)},
        {"indices", imp.getRawIndices()},
        {"transform", serializeTransform(imp.getTransform())}
    };
}

json serializeComponent(const Component& comp) {
    if (const auto* wing = dynamic_cast<const Wing*>(&comp)) {
        return serializeWing(*wing);
    }
    if (const auto* fuse = dynamic_cast<const Fuselage*>(&comp)) {
        return serializeFuselage(*fuse);
    }
    if (const auto* pod = dynamic_cast<const Pod*>(&comp)) {
        return serializePod(*pod);
    }
    if (const auto* prop = dynamic_cast<const Propeller*>(&comp)) {
        return serializePropeller(*prop);
    }
    if (const auto* gear = dynamic_cast<const LandingGear*>(&comp)) {
        return serializeLandingGear(*gear);
    }
    if (const auto* imp = dynamic_cast<const ImportedMesh*>(&comp)) {
        return serializeImportedMesh(*imp);
    }
    return json{};
}

std::unique_ptr<Component> deserializeComponent(const json& cj, std::uint64_t id) {
    const std::string type = cj.value("type", "");
    const std::string name = cj.value("name", type);

    if (type == "Wing") {
        auto wing = std::make_unique<Wing>(id, name);
        wing->setVisible(cj.value("visible", true));
        wing->setSymmetry(cj.value("symmetry", "XZ") == "XZ" ? SymmetryMode::XZ : SymmetryMode::None);
        wing->setCapTips(cj.value("capTips", true));
        wing->setChordPoints(cj.value("chordPoints", 41));
        if (cj.contains("transform")) {
            deserializeTransform(cj["transform"], wing->getTransform());
        }

        if (cj.contains("sections") && cj["sections"].is_array() && !cj["sections"].empty()) {
            const auto& bays = cj["sections"];
            while (wing->getSectionCount() < bays.size()) {
                wing->addSection();
            }
            while (wing->getSectionCount() > bays.size() && wing->getSectionCount() > 1) {
                wing->removeSection(wing->getSectionCount() - 1);
            }
            for (std::size_t i = 0; i < bays.size(); ++i) {
                const json& bj = bays[i];
                WingSection& sec = wing->getSection(i);
                sec.span().setValue(bj.value("span", 6.0));
                sec.rootChord().setValue(bj.value("rootChord", 2.0));
                sec.tipChord().setValue(bj.value("tipChord", 1.0));
                sec.sweepDeg().setValue(bj.value("sweepDeg", 0.0));
                sec.sweepLocation().setValue(bj.value("sweepLocation", 0.25));
                sec.dihedralDeg().setValue(bj.value("dihedralDeg", 0.0));
                sec.rootTwistDeg().setValue(bj.value("rootTwistDeg", 0.0));
                sec.tipTwistDeg().setValue(bj.value("tipTwistDeg", 0.0));
                sec.twistLocation().setValue(bj.value("twistLocation", 0.25));
                sec.setSpanSubdivisions(bj.value("spanSubdivisions", 8));
                if (bj.contains("rootAirfoil")) {
                    deserializeAirfoil(bj["rootAirfoil"], sec.getRootAirfoil());
                }
                if (bj.contains("tipAirfoil")) {
                    deserializeAirfoil(bj["tipAirfoil"], sec.getTipAirfoil());
                }
            }
        }
        if (cj.contains("controlSurfaces") && cj["controlSurfaces"].is_array()) {
            for (const json& csj : cj["controlSurfaces"]) {
                wing->addControlSurface(ControlSurface(
                    csj.value("name", "Control"),
                    csj.value("bayIndex", std::size_t{0}),
                    csj.value("spanStartFrac", 0.5),
                    csj.value("spanEndFrac", 0.9),
                    csj.value("chordFrac", 0.25),
                    csj.value("deflectionDeg", 0.0),
                    csj.value("symmetric", false)
                ));
            }
        }
        wing->markDirty();
        wing->ensureBuilt();
        return wing;
    }

    if (type == "Fuselage") {
        auto fuse = std::make_unique<Fuselage>(id, name);
        fuse->setVisible(cj.value("visible", true));
        fuse->length().setValue(cj.value("length", 12.0));
        fuse->setSmoothLoft(cj.value("smoothLoft", true));
        fuse->setCapEnds(cj.value("capEnds", true));
        fuse->setRadialPoints(cj.value("radialPoints", 36));
        fuse->setAxialSubdivisions(cj.value("axialSubdivisions", 8));
        if (cj.contains("transform")) {
            deserializeTransform(cj["transform"], fuse->getTransform());
        }

        if (cj.contains("sections") && cj["sections"].is_array() && cj["sections"].size() >= 2) {
            fuse->clearSections();
            for (const json& sj : cj["sections"]) {
                FuselageSection sec(
                    stringToShape(sj.value("shape", "ellipse")),
                    sj.value("stationFrac", 0.5),
                    sj.value("width", 1.5),
                    sj.value("height", 1.5),
                    sj.value("zOffset", 0.0),
                    sj.value("superEllipseN", 2.5)
                );
                sec.yOffset().setValue(sj.value("yOffset", 0.0));
                sec.cornerRadiusFrac().setValue(sj.value("cornerRadiusFrac", 0.25));
                fuse->appendSection(std::move(sec));
            }
        }
        fuse->markDirty();
        fuse->ensureBuilt();
        return fuse;
    }

    if (type == "Pod") {
        auto pod = std::make_unique<Pod>(id, name);
        pod->setVisible(cj.value("visible", true));
        pod->length().setValue(cj.value("length", 3.5));
        pod->width().setValue(cj.value("width", 0.85));
        pod->height().setValue(cj.value("height", 0.85));
        pod->noseFrac().setValue(cj.value("noseFrac", 0.28));
        pod->tailFrac().setValue(cj.value("tailFrac", 0.32));
        pod->nosePower().setValue(cj.value("nosePower", 0.55));
        pod->tailPower().setValue(cj.value("tailPower", 0.65));
        pod->superEllipseN().setValue(cj.value("superEllipseN", 2.0));
        pod->setSymmetry(cj.value("symmetry", "None") == "XZ" ? SymmetryMode::XZ : SymmetryMode::None);
        pod->setRadialPoints(cj.value("radialPoints", 28));
        pod->setAxialStations(cj.value("axialStations", 24));
        if (cj.contains("transform")) {
            deserializeTransform(cj["transform"], pod->getTransform());
        }
        pod->markDirty();
        pod->ensureBuilt();
        return pod;
    }

    if (type == "Propeller") {
        auto prop = std::make_unique<Propeller>(id, name);
        prop->setVisible(cj.value("visible", true));
        prop->diameter().setValue(cj.value("diameter", 2.1));
        prop->hubDiameter().setValue(cj.value("hubDiameter", 0.28));
        prop->spinnerLength().setValue(cj.value("spinnerLength", 0.42));
        prop->rootChord().setValue(cj.value("rootChord", 0.19));
        prop->tipChord().setValue(cj.value("tipChord", 0.09));
        prop->collectivePitchDeg().setValue(cj.value("collectivePitchDeg", 24.0));
        prop->washoutDeg().setValue(cj.value("washoutDeg", 22.0));
        prop->sweepDeg().setValue(cj.value("sweepDeg", 4.0));
        prop->setBladeCount(cj.value("bladeCount", 3));
        prop->setClockwise(cj.value("clockwise", true));
        prop->setShowDiskRing(cj.value("showDiskRing", true));
        if (cj.contains("airfoil")) {
            deserializeAirfoil(cj["airfoil"], prop->getBladeAirfoil());
        }
        if (cj.contains("transform")) {
            deserializeTransform(cj["transform"], prop->getTransform());
        }
        prop->markDirty();
        prop->ensureBuilt();
        return prop;
    }

    if (type == "LandingGear") {
        auto gear = std::make_unique<LandingGear>(id, name);
        gear->setVisible(cj.value("visible", true));
        gear->strutLength().setValue(cj.value("strutLength", 1.15));
        gear->strutDiameter().setValue(cj.value("strutDiameter", 0.09));
        gear->rakeDeg().setValue(cj.value("rakeDeg", 6.0));
        gear->splayDeg().setValue(cj.value("splayDeg", 10.0));
        gear->wheelDiameter().setValue(cj.value("wheelDiameter", 0.48));
        gear->wheelWidth().setValue(cj.value("wheelWidth", 0.18));
        gear->setWheelsPerStrut(cj.value("wheelsPerStrut", 1));
        gear->setWheelPant(cj.value("wheelPant", false));
        gear->setSymmetry(cj.value("symmetry", "XZ") == "XZ" ? SymmetryMode::XZ : SymmetryMode::None);
        if (cj.contains("transform")) {
            deserializeTransform(cj["transform"], gear->getTransform());
        }
        gear->markDirty();
        gear->ensureBuilt();
        return gear;
    }

    if (type == "ImportedMesh") {
        std::vector<glm::vec3> positions;
        if (cj.contains("positions") && cj["positions"].is_array()) {
            const auto& arr = cj["positions"];
            positions.reserve(arr.size() / 3);
            for (std::size_t i = 0; i + 2 < arr.size(); i += 3) {
                positions.emplace_back(arr[i].get<float>(), arr[i + 1].get<float>(), arr[i + 2].get<float>());
            }
        }
        std::vector<std::uint32_t> indices;
        if (cj.contains("indices") && cj["indices"].is_array()) {
            indices = cj["indices"].get<std::vector<std::uint32_t>>();
        }
        auto imp = std::make_unique<ImportedMesh>(
            id,
            name,
            std::move(positions),
            std::move(indices),
            cj.value("sourcePath", "")
        );
        imp->setVisible(cj.value("visible", true));
        imp->setUnitScale(cj.value("unitScale", 1.0f));
        imp->setAxisConversion(
            cj.value("axisConversion", "ZUp") == "YUpToZUp" ? UpAxisConversion::YUpToZUp : UpAxisConversion::NoneZUp
        );
        imp->setCenterOrigin(cj.value("centerOrigin", false));
        if (cj.contains("transform")) {
            deserializeTransform(cj["transform"], imp->getTransform());
        }
        imp->markDirty();
        imp->ensureBuilt();
        return imp;
    }

    return nullptr;
}

} // namespace

std::string ProjectSerializer::saveToString(const Aircraft& aircraft) {
    json root;
    root["format"] = "OpenPADS";
    root["version"] = 2;
    root["aircraftName"] = aircraft.getName();

    json components = json::array();
    for (const auto& comp : aircraft.getComponents()) {
        json cj = serializeComponent(*comp);
        if (!cj.is_null() && !cj.empty()) {
            components.push_back(std::move(cj));
        }
    }
    root["components"] = std::move(components);
    return root.dump(2);
}

bool ProjectSerializer::loadFromString(Aircraft& aircraft, std::string_view jsonText) {
    json root;
    try {
        root = json::parse(jsonText);
    } catch (const json::exception&) {
        return false;
    }

    if (!root.is_object() || root.value("format", "") != "OpenPADS") {
        return false;
    }

    Aircraft loaded(root.value("aircraftName", "Untitled Aircraft"));
    if (root.contains("components") && root["components"].is_array()) {
        for (const json& cj : root["components"]) {
            const std::uint64_t id = loaded.allocateComponentId();
            if (auto comp = deserializeComponent(cj, id)) {
                loaded.addComponent(std::move(comp));
            }
        }
    }

    if (!loaded.getComponents().empty()) {
        loaded.setSelectedComponentId(loaded.getComponents().front()->getId());
    }
    aircraft = std::move(loaded);
    return true;
}

bool ProjectSerializer::saveToFile(const Aircraft& aircraft, const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }
    out << saveToString(aircraft);
    return out.good();
}

bool ProjectSerializer::loadFromFile(Aircraft& aircraft, const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return loadFromString(aircraft, ss.str());
}

Component* ProjectSerializer::duplicateComponent(Aircraft& aircraft, std::uint64_t sourceId) {
    const Component* src = aircraft.findComponent(sourceId);
    if (src == nullptr) {
        return nullptr;
    }
    json cj = serializeComponent(*src);
    if (cj.is_null() || cj.empty()) {
        return nullptr;
    }
    cj["name"] = src->getName() + " Copy";
    const std::uint64_t newId = aircraft.allocateComponentId();
    auto clone = deserializeComponent(cj, newId);
    if (!clone) {
        return nullptr;
    }
    Component* ptr = aircraft.addComponent(std::move(clone));
    aircraft.setSelectedComponentId(newId);
    return ptr;
}

} // namespace opads
