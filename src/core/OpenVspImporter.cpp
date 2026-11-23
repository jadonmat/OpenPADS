#include "core/OpenVspImporter.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <pugixml.hpp>

#include "core/Fuselage.hpp"
#include "core/Pod.hpp"
#include "core/Propeller.hpp"
#include "core/Wing.hpp"

namespace opads {

namespace {

std::string readParmString(const pugi::xml_node& container, const char* name, const std::string& fallback) {
    if (!container) {
        return fallback;
    }
    // Check direct child <Name Value="..."/> or <Name>text</Name>
    if (const pugi::xml_node child = container.child(name)) {
        if (const pugi::xml_attribute valAttr = child.attribute("Value")) {
            return valAttr.as_string(fallback.c_str());
        }
        if (child.text() && !child.text().empty()) {
            return child.text().as_string(fallback.c_str());
        }
    }
    // Check <Parm Name="..." Value="..."/>
    for (const pugi::xml_node parm : container.children("Parm")) {
        if (std::string(parm.attribute("Name").as_string()) == name) {
            return parm.attribute("Value").as_string(fallback.c_str());
        }
    }
    // Search immediate sub-containers
    for (const pugi::xml_node sub : container.children()) {
        if (const pugi::xml_node nested = sub.child(name)) {
            if (const pugi::xml_attribute valAttr = nested.attribute("Value")) {
                return valAttr.as_string(fallback.c_str());
            }
        }
        for (const pugi::xml_node parm : sub.children("Parm")) {
            if (std::string(parm.attribute("Name").as_string()) == name) {
                return parm.attribute("Value").as_string(fallback.c_str());
            }
        }
    }
    return fallback;
}

double readParmDouble(const pugi::xml_node& container, const char* name, double fallback) {
    if (!container) {
        return fallback;
    }
    if (const pugi::xml_node child = container.child(name)) {
        if (const pugi::xml_attribute valAttr = child.attribute("Value")) {
            return valAttr.as_double(fallback);
        }
    }
    for (const pugi::xml_node parm : container.children("Parm")) {
        if (std::string(parm.attribute("Name").as_string()) == name) {
            return parm.attribute("Value").as_double(fallback);
        }
    }
    for (const pugi::xml_node sub : container.children()) {
        if (const pugi::xml_node nested = sub.child(name)) {
            if (const pugi::xml_attribute valAttr = nested.attribute("Value")) {
                return valAttr.as_double(fallback);
            }
        }
        for (const pugi::xml_node parm : sub.children("Parm")) {
            if (std::string(parm.attribute("Name").as_string()) == name) {
                return parm.attribute("Value").as_double(fallback);
            }
        }
    }
    return fallback;
}

Transform readVspTransform(const pugi::xml_node& parmContainer) {
    Transform t;
    const pugi::xml_node xform = parmContainer.child("XForm");
    const pugi::xml_node src = xform ? xform : parmContainer;

    const float x = static_cast<float>(readParmDouble(src, "X_Location", 0.0));
    const float y = static_cast<float>(readParmDouble(src, "Y_Location", 0.0));
    const float z = static_cast<float>(readParmDouble(src, "Z_Location", 0.0));
    const float rx = static_cast<float>(readParmDouble(src, "X_Rotation", 0.0));
    const float ry = static_cast<float>(readParmDouble(src, "Y_Rotation", 0.0));
    const float rz = static_cast<float>(readParmDouble(src, "Z_Rotation", 0.0));

    t.setPosition(glm::vec3(x, y, z));
    t.setRotationDeg(glm::vec3(rx, ry, rz));
    return t;
}

void applyVspAirfoil(const pugi::xml_node& xsecNode, Airfoil& foilOut) {
    const pugi::xml_node curveNode = xsecNode.child("XSecCurve");
    const pugi::xml_node parmNode = curveNode ? curveNode.child("ParmContainer") : xsecNode.child("ParmContainer");
    const pugi::xml_node src = parmNode ? parmNode : curveNode;
    if (!src) {
        return;
    }

    const int type = static_cast<int>(readParmDouble(curveNode, "Type", readParmDouble(src, "Type", -1.0)));
    const double thick = readParmDouble(src, "ThickChord", 0.12);
    const double camber = readParmDouble(src, "Camber", 0.0);
    const double camberLoc = readParmDouble(src, "CamberLoc", 0.4);

    if (type == 4 || camber >= 0.0) {
        const int mDigit = std::clamp(static_cast<int>(std::round(camber * 100.0)), 0, 9);
        const int pDigit = std::clamp(static_cast<int>(std::round(camberLoc * 10.0)), 0, 9);
        const int ttDigits = std::clamp(static_cast<int>(std::round(thick * 100.0)), 1, 40);
        std::ostringstream code;
        code << mDigit << (mDigit == 0 ? 0 : pDigit) << std::setw(2) << std::setfill('0') << ttDigits;
        foilOut.setNaca4(code.str());
    }
}

void importWingGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, const std::string& name) {
    const std::uint64_t id = aircraft.allocateComponentId();
    auto wing = std::make_unique<Wing>(id, name);

    const pugi::xml_node parm = geomNode.child("ParmContainer");
    wing->getTransform() = readVspTransform(parm);

    const int symFlag = static_cast<int>(readParmDouble(parm, "Sym_Planar_Flag", 2.0));
    wing->setSymmetry((symFlag == 0) ? SymmetryMode::None : SymmetryMode::XZ);

    const pugi::xml_node wingGeom = geomNode.child("WingGeom");
    const pugi::xml_node xsecSurf = wingGeom.child("XSecSurf");
    std::vector<pugi::xml_node> xsecs;
    for (const pugi::xml_node xsec : xsecSurf.children("XSec")) {
        xsecs.push_back(xsec);
    }

    if (xsecs.size() >= 2) {
        const std::size_t bayCount = xsecs.size() - 1;
        while (wing->getSectionCount() < bayCount) {
            wing->addSection();
        }
        while (wing->getSectionCount() > bayCount && wing->getSectionCount() > 1) {
            wing->removeSection(wing->getSectionCount() - 1);
        }

        for (std::size_t i = 0; i < bayCount; ++i) {
            const pugi::xml_node bayNode = xsecs[i + 1];
            const pugi::xml_node bayParm = bayNode.child("ParmContainer");
            WingSection& bay = wing->getSection(i);

            bay.span().setValue(readParmDouble(bayParm, "Span", 5.0));
            bay.rootChord().setValue(readParmDouble(bayParm, "Root_Chord", 2.0));
            bay.tipChord().setValue(readParmDouble(bayParm, "Tip_Chord", 1.0));
            bay.sweepDeg().setValue(readParmDouble(bayParm, "Sweep", 0.0));
            bay.sweepLocation().setValue(readParmDouble(bayParm, "Sweep_Location", 0.25));
            bay.dihedralDeg().setValue(readParmDouble(bayParm, "Dihedral", 0.0));
            bay.tipTwistDeg().setValue(readParmDouble(bayParm, "Twist", 0.0));
            bay.twistLocation().setValue(readParmDouble(bayParm, "Twist_Location", 0.25));

            applyVspAirfoil(xsecs[i], bay.getRootAirfoil());
            applyVspAirfoil(bayNode, bay.getTipAirfoil());
        }
    }

    wing->markDirty();
    wing->ensureBuilt();
    aircraft.addComponent(std::move(wing));
}

void importFuselageGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, const std::string& name) {
    const std::uint64_t id = aircraft.allocateComponentId();
    auto fuse = std::make_unique<Fuselage>(id, name);

    const pugi::xml_node parm = geomNode.child("ParmContainer");
    fuse->getTransform() = readVspTransform(parm);
    const double length = readParmDouble(parm, "Length", 12.0);
    fuse->length().setValue(length);

    const pugi::xml_node fuseGeom = geomNode.child("FuselageGeom");
    const pugi::xml_node xsecSurf = fuseGeom.child("XSecSurf");
    std::vector<pugi::xml_node> xsecs;
    for (const pugi::xml_node xsec : xsecSurf.children("XSec")) {
        xsecs.push_back(xsec);
    }

    if (xsecs.size() >= 2) {
        fuse->clearSections();
        for (std::size_t i = 0; i < xsecs.size(); ++i) {
            const pugi::xml_node xsecParm = xsecs[i].child("ParmContainer");
            const pugi::xml_node curve = xsecs[i].child("XSecCurve");
            const pugi::xml_node curveParm = curve ? curve.child("ParmContainer") : pugi::xml_node();

            const double frac = readParmDouble(
                xsecParm,
                "XLocPercent",
                static_cast<double>(i) / static_cast<double>(xsecs.size() - 1)
            );
            const double zPct = readParmDouble(xsecParm, "ZLocPercent", 0.0);
            const double width = readParmDouble(curveParm, "Width", readParmDouble(curveParm, "Diameter", 1.5));
            const double height = readParmDouble(curveParm, "Height", width);
            const int curveType = static_cast<int>(readParmDouble(curve, "Type", 2.0));

            CrossSectionShape shape = CrossSectionShape::Ellipse;
            if (curveType == 0 || (width < 1e-4 && height < 1e-4)) {
                shape = CrossSectionShape::Point;
            } else if (curveType == 1) {
                shape = CrossSectionShape::Circle;
            } else if (curveType == 3) {
                shape = CrossSectionShape::SuperEllipse;
            } else if (curveType == 4) {
                shape = CrossSectionShape::RoundedRect;
            }

            fuse->appendSection(FuselageSection(shape, frac, width, height, zPct * length, 2.5));
        }
    }

    fuse->markDirty();
    fuse->ensureBuilt();
    aircraft.addComponent(std::move(fuse));
}

void importPodGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, const std::string& name) {
    const std::uint64_t id = aircraft.allocateComponentId();
    auto pod = std::make_unique<Pod>(id, name);

    const pugi::xml_node parm = geomNode.child("ParmContainer");
    pod->getTransform() = readVspTransform(parm);
    const double length = readParmDouble(parm, "Length", 3.5);
    const double fineRatio = readParmDouble(parm, "FineRatio", 6.0);
    pod->length().setValue(length);
    const double diam = (fineRatio > 0.1) ? (length / fineRatio) : 0.8;
    pod->width().setValue(diam);
    pod->height().setValue(diam);

    const int symFlag = static_cast<int>(readParmDouble(parm, "Sym_Planar_Flag", 0.0));
    pod->setSymmetry((symFlag == 2) ? SymmetryMode::XZ : SymmetryMode::None);
    pod->markDirty();
    pod->ensureBuilt();
    aircraft.addComponent(std::move(pod));
}

void importPropGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, const std::string& name) {
    const std::uint64_t id = aircraft.allocateComponentId();
    auto prop = std::make_unique<Propeller>(id, name);

    const pugi::xml_node parm = geomNode.child("ParmContainer");
    prop->getTransform() = readVspTransform(parm);
    prop->diameter().setValue(readParmDouble(parm, "Diameter", 2.1));
    prop->setBladeCount(static_cast<int>(readParmDouble(parm, "NumBlade", 3.0)));
    prop->collectivePitchDeg().setValue(readParmDouble(parm, "Beta34", 24.0));
    prop->markDirty();
    prop->ensureBuilt();
    aircraft.addComponent(std::move(prop));
}

} // namespace

bool OpenVspImporter::importFromFile(Aircraft& aircraft, const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return importFromString(aircraft, ss.str(), path.stem().string());
}

bool OpenVspImporter::importFromString(
    Aircraft& aircraft,
    std::string_view xmlContents,
    const std::string& fallbackName
) {
    pugi::xml_document doc;
    const pugi::xml_parse_result parseRes = doc.load_buffer(xmlContents.data(), xmlContents.size());
    if (!parseRes) {
        return false;
    }

    const pugi::xml_node root = doc.child("Vsp_Geometry");
    if (!root) {
        return false;
    }

    const pugi::xml_node vehicle = root.child("Vehicle");
    const pugi::xml_node searchRoot = vehicle ? vehicle : root;

    if (aircraft.getComponents().empty() && aircraft.getName() == "Untitled Aircraft") {
        aircraft.setName(fallbackName);
    }

    std::size_t importedCount = 0;
    for (const pugi::xml_node geom : searchRoot.children("Geom")) {
        const pugi::xml_node parm = geom.child("ParmContainer");
        const std::string geomName = readParmString(parm, "Name", "VSP Geom");
        const pugi::xml_node geomBase = geom.child("GeomBase");
        const std::string typeName = geomBase ? geomBase.child_value("TypeName") : "";

        if (typeName == "Wing" || geom.child("WingGeom")) {
            importWingGeom(geom, aircraft, geomName);
            ++importedCount;
        } else if (typeName == "Fuselage" || geom.child("FuselageGeom")) {
            importFuselageGeom(geom, aircraft, geomName);
            ++importedCount;
        } else if (typeName == "Pod" || geom.child("PodGeom")) {
            importPodGeom(geom, aircraft, geomName);
            ++importedCount;
        } else if (typeName == "Propeller" || geom.child("PropellerGeom")) {
            importPropGeom(geom, aircraft, geomName);
            ++importedCount;
        }
    }

    if (importedCount > 0 && !aircraft.getComponents().empty()) {
        aircraft.setSelectedComponentId(aircraft.getComponents().back()->getId());
    }
    return importedCount > 0;
}

} // namespace opads
