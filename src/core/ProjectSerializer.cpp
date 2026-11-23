#include "core/ProjectSerializer.hpp"

#include <fstream>
#include <sstream>
#include <vector>

#include <nlohmann/json.hpp>

#include "core/Airfoil.hpp"
#include "core/Fuselage.hpp"
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

    return json{
        {"type", "Wing"},
        {"name", wing.getName()},
        {"visible", wing.isVisible()},
        {"symmetry", wing.getSymmetry() == SymmetryMode::XZ ? "XZ" : "None"},
        {"capTips", wing.getCapTips()},
        {"chordPoints", wing.getChordPoints()},
        {"transform", serializeTransform(wing.getTransform())},
        {"sections", bays}
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

} // namespace

std::string ProjectSerializer::saveToString(const Aircraft& aircraft) {
    json root;
    root["format"] = "OpenPADS";
    root["version"] = 1;
    root["aircraftName"] = aircraft.getName();

    json components = json::array();
    for (const auto& comp : aircraft.getComponents()) {
        if (const auto* wing = dynamic_cast<const Wing*>(comp.get())) {
            components.push_back(serializeWing(*wing));
        } else if (const auto* fuse = dynamic_cast<const Fuselage*>(comp.get())) {
            components.push_back(serializeFuselage(*fuse));
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
            const std::string type = cj.value("type", "");
            const std::string name = cj.value("name", type);
            const std::uint64_t id = loaded.allocateComponentId();

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
                wing->markDirty();
                wing->ensureBuilt();
                loaded.addComponent(std::move(wing));
            } else if (type == "Fuselage") {
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
                loaded.addComponent(std::move(fuse));
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

} // namespace opads
