#include "core/OpenVspImporter.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <pugixml.hpp>

#include "core/Fuselage.hpp"
#include "core/LandingGear.hpp"
#include "core/Pod.hpp"
#include "core/Propeller.hpp"
#include "core/Wing.hpp"

namespace opads {

namespace {

struct BodyBoundsXY {
    glm::vec3 minPt{0.0f};
    glm::vec3 maxPt{0.0f};
};

struct ImportContext {
    double unitScale{1.0};
    std::vector<BodyBoundsXY> bodyBounds;
};

double vspLenUnitToMeters(int lenUnit) noexcept {
    switch (lenUnit) {
    case 0: return 0.001;   // LEN_MM
    case 1: return 0.01;    // LEN_CM
    case 2: return 1.0;     // LEN_M
    case 3: return 0.0254;  // LEN_IN
    case 4: return 0.3048;  // LEN_FT
    case 5: return 0.9144;  // LEN_YD
    default: return 1.0;
    }
}

std::string readParmString(const pugi::xml_node& container, const char* name, const std::string& fallback) {
    if (!container) {
        return fallback;
    }
    if (const pugi::xml_node child = container.child(name)) {
        if (const pugi::xml_attribute valAttr = child.attribute("Value")) {
            return valAttr.as_string(fallback.c_str());
        }
        if (child.text() && !child.text().empty()) {
            return child.text().as_string(fallback.c_str());
        }
    }
    for (const pugi::xml_node parm : container.children("Parm")) {
        if (std::string(parm.attribute("Name").as_string()) == name) {
            return parm.attribute("Value").as_string(fallback.c_str());
        }
    }
    for (const pugi::xml_node sub : container.children()) {
        if (const pugi::xml_node nested = sub.child(name)) {
            if (const pugi::xml_attribute valAttr = nested.attribute("Value")) {
                return valAttr.as_string(fallback.c_str());
            }
            if (nested.text() && !nested.text().empty()) {
                return nested.text().as_string(fallback.c_str());
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

bool tryReadParmDouble(const pugi::xml_node& container, const char* name, double& outVal) {
    if (!container) {
        return false;
    }
    if (const pugi::xml_node child = container.child(name)) {
        if (const pugi::xml_attribute valAttr = child.attribute("Value")) {
            outVal = valAttr.as_double(outVal);
            return true;
        }
        if (child.text() && !child.text().empty()) {
            outVal = child.text().as_double(outVal);
            return true;
        }
    }
    for (const pugi::xml_node parm : container.children("Parm")) {
        if (std::string(parm.attribute("Name").as_string()) == name) {
            outVal = parm.attribute("Value").as_double(outVal);
            return true;
        }
    }
    for (const pugi::xml_node sub : container.children()) {
        if (const pugi::xml_node nested = sub.child(name)) {
            if (const pugi::xml_attribute valAttr = nested.attribute("Value")) {
                outVal = valAttr.as_double(outVal);
                return true;
            }
            if (nested.text() && !nested.text().empty()) {
                outVal = nested.text().as_double(outVal);
                return true;
            }
        }
        for (const pugi::xml_node parm : sub.children("Parm")) {
            if (std::string(parm.attribute("Name").as_string()) == name) {
                outVal = parm.attribute("Value").as_double(outVal);
                return true;
            }
        }
    }
    return false;
}

double readParmDouble(const pugi::xml_node& container, const char* name, double fallback) {
    double val = fallback;
    tryReadParmDouble(container, name, val);
    return val;
}

struct VspEndCap {
    EndCapType minType{EndCapType::Flat};
    double minLength{1.0};
    double minStrength{0.5};
    EndCapType maxType{EndCapType::Flat};
    double maxLength{1.0};
    double maxStrength{0.5};
};

EndCapType toEndCapType(int opt) noexcept {
    switch (opt) {
    case 0: return EndCapType::None;
    case 1: return EndCapType::Flat;
    case 2: return EndCapType::Round;
    case 3: return EndCapType::Edge;
    case 4: return EndCapType::Sharp;
    default: return EndCapType::Flat;
    }
}

VspEndCap readVspEndCap(const pugi::xml_node& parmContainer) {
    VspEndCap ec;
    const pugi::xml_node capNode = parmContainer.child("EndCap");
    const pugi::xml_node src = capNode ? capNode : parmContainer;
    ec.minType = toEndCapType(static_cast<int>(std::lround(readParmDouble(src, "CapUMinOption", 1.0))));
    ec.minLength = readParmDouble(src, "CapUMinLength", 1.0);
    ec.minStrength = readParmDouble(src, "CapUMinStrength", 0.5);
    ec.maxType = toEndCapType(static_cast<int>(std::lround(readParmDouble(src, "CapUMaxOption", 1.0))));
    ec.maxLength = readParmDouble(src, "CapUMaxLength", 1.0);
    ec.maxStrength = readParmDouble(src, "CapUMaxStrength", 0.5);
    return ec;
}

struct ResolvedXSecCurve {
    pugi::xml_node curveWrapper;
    pugi::xml_node innerCurve;
    pugi::xml_node curveParm;
    int type{-1};
};

ResolvedXSecCurve resolveXSecCurve(const pugi::xml_node& xsecNode) {
    ResolvedXSecCurve res;
    if (!xsecNode) {
        return res;
    }
    pugi::xml_node wrapper = xsecNode.child("XSecCurve");
    if (!wrapper && xsecNode.child("XSec")) {
        wrapper = xsecNode.child("XSec").child("XSecCurve");
    }
    if (!wrapper) {
        return res;
    }
    res.curveWrapper = wrapper;
    res.innerCurve = wrapper.child("XSecCurve");
    res.curveParm = wrapper.child("ParmContainer");

    double tVal = -1.0;
    if (res.innerCurve && tryReadParmDouble(res.innerCurve, "Type", tVal)) {
        res.type = static_cast<int>(std::lround(tVal));
    } else if (tryReadParmDouble(wrapper, "Type", tVal)) {
        res.type = static_cast<int>(std::lround(tVal));
    } else if (const pugi::xml_attribute tAttr = wrapper.attribute("Type")) {
        res.type = tAttr.as_int(-1);
    } else if (res.curveParm && tryReadParmDouble(res.curveParm, "Type", tVal)) {
        res.type = static_cast<int>(std::lround(tVal));
    }
    return res;
}

Transform readVspTransform(const pugi::xml_node& parmContainer, double unitScale) {
    Transform t;
    const pugi::xml_node xform = parmContainer.child("XForm");
    const pugi::xml_node src = xform ? xform : parmContainer;

    const float x = static_cast<float>(readParmDouble(src, "X_Location", 0.0) * unitScale);
    const float y = static_cast<float>(readParmDouble(src, "Y_Location", 0.0) * unitScale);
    const float z = static_cast<float>(readParmDouble(src, "Z_Location", 0.0) * unitScale);
    const float rx = static_cast<float>(readParmDouble(src, "X_Rotation", 0.0));
    const float ry = static_cast<float>(readParmDouble(src, "Y_Rotation", 0.0));
    const float rz = static_cast<float>(readParmDouble(src, "Z_Rotation", 0.0));

    t.setPosition(glm::vec3(x, y, z));
    t.setRotationDeg(glm::vec3(rx, ry, rz));
    return t;
}

void recordBodyBounds(ImportContext& ctx, const Component& comp) {
    const auto& verts = comp.getMesh().getVertices();
    if (verts.empty()) {
        return;
    }
    const glm::mat4 m = comp.getTransform().getMatrix();
    glm::vec3 wMin(1e9f);
    glm::vec3 wMax(-1e9f);
    for (const Vertex& v : verts) {
        const glm::vec3 wc = glm::vec3(m * glm::vec4(v.position, 1.0f));
        wMin = glm::min(wMin, wc);
        wMax = glm::max(wMax, wc);
    }
    ctx.bodyBounds.push_back(BodyBoundsXY{wMin, wMax});
}

void applyVspAirfoil(const pugi::xml_node& xsecNode, Airfoil& foilOut) {
    const ResolvedXSecCurve curve = resolveXSecCurve(xsecNode);
    const pugi::xml_node src = curve.curveParm ? curve.curveParm : curve.curveWrapper;
    if (!src) {
        return;
    }

    const int type = curve.type;
    const double thick = readParmDouble(src, "ThickChord", 0.12);
    const double camber = readParmDouble(src, "Camber", 0.0);
    const double camberLoc = readParmDouble(src, "CamberLoc", 0.4);
    const double idealCl = readParmDouble(src, "IdealCl", 0.0);
    const int ttDigits = std::clamp(static_cast<int>(std::round(thick * 100.0)), 1, 40);

    // NACA 5-digit (XS_FIVE_DIGIT = 16, XS_FIVE_DIGIT_MOD = 17)
    if (type == 16 || type == 17) {
        const int lDigit = std::clamp(static_cast<int>(std::round(std::max(0.0, idealCl) / 0.15)), 0, 9);
        const int pDigit = std::clamp(static_cast<int>(std::round(camberLoc * 20.0)), 1, 5);
        std::ostringstream code;
        code << lDigit << pDigit << '0' << std::setw(2) << std::setfill('0') << ttDigits;
        foilOut.setNaca5(code.str());
        return;
    }

    // NACA 6-series / 16-series (XS_SIX_SERIES = 8, XS_ONE_SIX_SERIES = 18)
    if (type == 8 || type == 18) {
        const double eqCamber = std::clamp(std::abs(idealCl) * 0.055, 0.0, 0.09);
        const int mDigit = std::clamp(static_cast<int>(std::round(eqCamber * 100.0)), 0, 9);
        std::ostringstream code;
        code << mDigit << (mDigit == 0 ? 0 : 5) << std::setw(2) << std::setfill('0') << ttDigits;
        foilOut.setNaca4(code.str());
        return;
    }

    // Standard NACA 4-series (XS_FOUR_SERIES = 7, XS_FOUR_DIGIT_MOD = 15, or fallback)
    if (type == 7 || type == 15 || camber > 0.0 || thick != 0.12) {
        const int mDigit = std::clamp(static_cast<int>(std::round(std::max(0.0, camber) * 100.0)), 0, 9);
        const int pDigit = std::clamp(static_cast<int>(std::round(camberLoc * 10.0)), 1, 9);
        std::ostringstream code;
        code << mDigit << (mDigit == 0 ? 0 : pDigit) << std::setw(2) << std::setfill('0') << ttDigits;
        foilOut.setNaca4(code.str());
    }
}

void importWingGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, ImportContext& ctx, const std::string& name) {
    const std::uint64_t id = aircraft.allocateComponentId();
    auto wing = std::make_unique<Wing>(id, name);

    const pugi::xml_node parm = geomNode.child("ParmContainer");
    const double geomScale = std::max(1e-4, readParmDouble(parm, "Scale", 1.0));
    const double lenScale = ctx.unitScale * geomScale;
    wing->getTransform() = readVspTransform(parm, ctx.unitScale);

    const int symFlag = static_cast<int>(std::lround(readParmDouble(parm, "Sym_Planar_Flag", 2.0)));
    wing->setSymmetry((symFlag == 2) ? SymmetryMode::XZ : SymmetryMode::None);

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

            bay.span().setValue(readParmDouble(bayParm, "Span", 5.0) * lenScale);
            bay.rootChord().setValue(readParmDouble(bayParm, "Root_Chord", 2.0) * lenScale);
            bay.tipChord().setValue(readParmDouble(bayParm, "Tip_Chord", 1.0) * lenScale);
            bay.sweepDeg().setValue(readParmDouble(bayParm, "Sweep", 0.0));
            bay.sweepLocation().setValue(readParmDouble(bayParm, "Sweep_Location", 0.25));
            bay.dihedralDeg().setValue(readParmDouble(bayParm, "Dihedral", 0.0));
            bay.tipTwistDeg().setValue(readParmDouble(bayParm, "Twist", 0.0));
            bay.twistLocation().setValue(readParmDouble(bayParm, "Twist_Location", 0.25));

            applyVspAirfoil(xsecs[i], bay.getRootAirfoil());
            applyVspAirfoil(bayNode, bay.getTipAirfoil());
        }
    }

    const VspEndCap endCap = readVspEndCap(parm);
    wing->setTipCapType(endCap.maxType);
    wing->setTipCapLength(endCap.maxLength);
    wing->setTipCapStrength(endCap.maxStrength);

    // Import OpenVSP SubSurfaces (SS_Control -> ControlSurface)
    std::string lowerWingName = name;
    for (char& ch : lowerWingName) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    const float rxAbs = std::abs(wing->getTransform().getRotationDeg().x);
    const bool isVerticalTail = (rxAbs > 45.0f && rxAbs < 135.0f) ||
        (lowerWingName.find("vert") != std::string::npos) ||
        (lowerWingName.find("fin") != std::string::npos) ||
        (lowerWingName.find("rudder") != std::string::npos);
    const bool isHorizTail = (lowerWingName.find("horiz") != std::string::npos) ||
        (lowerWingName.find("htail") != std::string::npos) ||
        (lowerWingName.find("elev") != std::string::npos);

    double totalWingSpan = 0.0;
    for (const WingSection& sec : wing->getSections()) {
        totalWingSpan += std::max(1e-4, sec.span().getValue());
    }

    auto parseSubSurfacesNode = [&](const pugi::xml_node& ssParent) {
        if (!ssParent) {
            return;
        }
        for (const pugi::xml_node ss : ssParent.children("SubSurface")) {
            const pugi::xml_node ssParm = ss.child("ParmContainer");
            const pugi::xml_node ctrl = ssParm.child("SS_Control");
            if (!ctrl) {
                continue;
            }
            const double eta0 = std::clamp(readParmDouble(ctrl, "EtaStart", 0.5), 0.0, 1.0);
            const double eta1 = std::clamp(readParmDouble(ctrl, "EtaEnd", 0.9), 0.0, 1.0);
            const double cStart = readParmDouble(ctrl, "Length_C_Start", 0.25);
            const double cEnd = readParmDouble(ctrl, "Length_C_End", cStart);
            const double chordFrac = std::clamp(0.5 * (cStart + cEnd), 0.05, 0.50);
            if (eta1 <= eta0 + 1e-3) {
                continue;
            }

            std::string ctrlName = readParmString(ssParm, "Name", "");
            bool symmetric = true;
            if (ctrlName.empty() || ctrlName == "SS_Control") {
                if (isVerticalTail) {
                    ctrlName = "Rudder";
                    symmetric = true;
                } else if (isHorizTail) {
                    ctrlName = "Elevator";
                    symmetric = true;
                } else if (eta0 >= 0.45) {
                    ctrlName = "Aileron";
                    symmetric = false;
                } else {
                    ctrlName = "Flap";
                    symmetric = true;
                }
            } else {
                std::string lc = ctrlName;
                for (char& ch : lc) {
                    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                }
                symmetric = (lc.find("aileron") == std::string::npos);
            }

            // Map overall normalized span [eta0, eta1] across wing bays
            double accum = 0.0;
            for (std::size_t b = 0; b < wing->getSectionCount(); ++b) {
                const double bSpan = std::max(1e-4, wing->getSection(b).span().getValue());
                const double bFrac0 = accum / totalWingSpan;
                accum += bSpan;
                const double bFrac1 = accum / totalWingSpan;
                const double o0 = std::max(eta0, bFrac0);
                const double o1 = std::min(eta1, bFrac1);
                if (o1 > o0 + 1e-3) {
                    const double u0 = (o0 - bFrac0) / (bFrac1 - bFrac0);
                    const double u1 = (o1 - bFrac0) / (bFrac1 - bFrac0);
                    wing->addControlSurface(ControlSurface(ctrlName, b, u0, u1, chordFrac, 0.0, symmetric));
                }
            }
        }
    };

    parseSubSurfacesNode(geomNode.child("SubSurfaces"));
    if (const pugi::xml_node innerGeom = geomNode.child("Geom")) {
        parseSubSurfacesNode(innerGeom.child("SubSurfaces"));
    }

    wing->markDirty();
    wing->ensureBuilt();
    recordBodyBounds(ctx, *wing);
    aircraft.addComponent(std::move(wing));
}

void importFuselageGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, ImportContext& ctx, const std::string& name) {
    const pugi::xml_node parm = geomNode.child("ParmContainer");
    const double geomScale = std::max(1e-4, readParmDouble(parm, "Scale", 1.0));
    const double lenScale = ctx.unitScale * geomScale;

    pugi::xml_node surfNode = geomNode.child("FuselageGeom").child("XSecSurf");
    if (!surfNode) {
        surfNode = geomNode.child("StackGeom").child("XSecSurf");
    }
    std::vector<pugi::xml_node> xsecs;
    for (const pugi::xml_node xsec : surfNode.children("XSec")) {
        xsecs.push_back(xsec);
    }

    double rawLength = -1.0;
    if (!tryReadParmDouble(parm, "Length", rawLength) || rawLength <= 1e-4) {
        for (const pugi::xml_node xs : xsecs) {
            if (tryReadParmDouble(xs.child("ParmContainer"), "RefLength", rawLength) && rawLength > 1e-4) {
                break;
            }
        }
    }
    if (rawLength <= 1e-4) {
        tryReadParmDouble(parm.child("BBox"), "X_Len", rawLength);
    }
    if (rawLength <= 1e-4) {
        rawLength = 12.0;
    }

    const double totalLengthM = rawLength * lenScale;

    auto buildSingleFuselage = [&](const std::string& compName, bool mirrorY) {
        const std::uint64_t id = aircraft.allocateComponentId();
        auto fuse = std::make_unique<Fuselage>(id, compName);
        fuse->getTransform() = readVspTransform(parm, ctx.unitScale);
        if (mirrorY) {
            glm::vec3 pos = fuse->getTransform().getPosition();
            pos.y = -pos.y;
            fuse->getTransform().setPosition(pos);
        }
        fuse->length().setValue(totalLengthM);

        if (xsecs.size() >= 2) {
            fuse->clearSections();
            for (std::size_t i = 0; i < xsecs.size(); ++i) {
                const pugi::xml_node xsecParm = xsecs[i].child("ParmContainer");
                const ResolvedXSecCurve curve = resolveXSecCurve(xsecs[i]);
                const pugi::xml_node curveParm = curve.curveParm ? curve.curveParm : curve.curveWrapper;

                const double frac = readParmDouble(
                    xsecParm,
                    "XLocPercent",
                    static_cast<double>(i) / static_cast<double>(xsecs.size() - 1)
                );
                const double yPct = readParmDouble(xsecParm, "YLocPercent", 0.0);
                const double zPct = readParmDouble(xsecParm, "ZLocPercent", 0.0);
                const double curveScale = std::max(0.0, readParmDouble(curveParm, "Scale", 1.0));
                const double deltaY = readParmDouble(curveParm, "DeltaX", 0.0);
                const double deltaZ = readParmDouble(curveParm, "DeltaY", 0.0);

                CrossSectionShape shape = CrossSectionShape::Ellipse;
                double rawWidth = 1.5;
                double rawHeight = 1.5;
                double topPower = 2.0;
                double botPower = -1.0;
                double cornerFrac = 0.25;
                double maxWidthLocFrac = 0.0;
                GeneralFuseParams gfParams;

                switch (curve.type) {
                case 0: // XS_POINT
                    shape = CrossSectionShape::Point;
                    rawWidth = 0.0;
                    rawHeight = 0.0;
                    break;
                case 1: // XS_CIRCLE
                    shape = CrossSectionShape::Circle;
                    rawWidth = readParmDouble(curveParm, "Circle_Diameter", readParmDouble(curveParm, "Diameter", 1.5));
                    rawHeight = rawWidth;
                    break;
                case 2: // XS_ELLIPSE
                    shape = CrossSectionShape::Ellipse;
                    rawWidth = readParmDouble(curveParm, "Ellipse_Width", readParmDouble(curveParm, "Width", 1.5));
                    rawHeight = readParmDouble(curveParm, "Ellipse_Height", readParmDouble(curveParm, "Height", rawWidth));
                    break;
                case 3: { // XS_SUPER_ELLIPSE
                    shape = CrossSectionShape::SuperEllipse;
                    rawWidth = readParmDouble(curveParm, "Super_Width", readParmDouble(curveParm, "Width", 1.5));
                    rawHeight = readParmDouble(curveParm, "Super_Height", readParmDouble(curveParm, "Height", rawWidth));
                    topPower = readParmDouble(curveParm, "Super_N", 2.0);
                    const bool topBotSym = readParmDouble(curveParm, "Super_TopBotSym", 1.0) >= 0.5;
                    if (!topBotSym) {
                        botPower = readParmDouble(curveParm, "Super_N_bot", topPower);
                    }
                    const double maxW = readParmDouble(curveParm, "Super_MaxWidthLoc", 0.0);
                    maxWidthLocFrac = maxW;
                    break;
                }
                case 4: { // XS_ROUNDED_RECTANGLE
                    shape = CrossSectionShape::RoundedRect;
                    rawWidth = readParmDouble(curveParm, "RoundedRect_Width", readParmDouble(curveParm, "Width", 1.5));
                    rawHeight = readParmDouble(curveParm, "RoundedRect_Height", readParmDouble(curveParm, "Height", rawWidth));
                    const double minDim = std::max(1e-4, std::min(rawWidth, rawHeight));
                    const double rad = readParmDouble(curveParm, "RoundRectXSec_Radius", 0.2 * minDim);
                    cornerFrac = std::clamp(rad / minDim, 0.01, 0.5);
                    break;
                }
                case 5: { // XS_GENERAL_FUSE
                    shape = CrossSectionShape::GeneralFuse;
                    rawWidth = readParmDouble(curveParm, "Width", 1.5);
                    rawHeight = readParmDouble(curveParm, "Height", rawWidth);
                    gfParams.topStr = readParmDouble(curveParm, "TopStr", 0.83);
                    gfParams.botStr = readParmDouble(curveParm, "BotStr", 0.83);
                    gfParams.upStr = readParmDouble(curveParm, "UpStr", 0.83);
                    gfParams.lowStr = readParmDouble(curveParm, "LowStr", 0.83);
                    gfParams.topTanAngleDeg = readParmDouble(curveParm, "TopTanAngle", 90.0);
                    gfParams.botTanAngleDeg = readParmDouble(curveParm, "BotTanAngle", 90.0);
                    maxWidthLocFrac = readParmDouble(curveParm, "MaxWidthLoc", 0.0);
                    if (rawWidth < 1e-4 && rawHeight < 1e-4) {
                        shape = CrossSectionShape::Point;
                    }
                    break;
                }
                default:
                    rawWidth = readParmDouble(curveParm, "Width", readParmDouble(curveParm, "Diameter", 1.5));
                    rawHeight = readParmDouble(curveParm, "Height", rawWidth);
                    if (rawWidth < 1e-4 && rawHeight < 1e-4) {
                        shape = CrossSectionShape::Point;
                    }
                    break;
                }

                const double widthM = rawWidth * curveScale * lenScale;
                const double heightM = rawHeight * curveScale * lenScale;
                const double yOffM = (yPct * rawLength + deltaY) * lenScale * (mirrorY ? -1.0 : 1.0);
                const double zOffM = (zPct * rawLength + deltaZ) * lenScale;

                FuselageSection sec(shape, frac, widthM, heightM, zOffM, topPower);
                sec.yOffset().setValue(yOffM);
                sec.setBottomSuperEllipseN(botPower);
                sec.setMaxWidthLocFrac(maxWidthLocFrac);
                sec.cornerRadiusFrac().setValue(cornerFrac);
                sec.setGeneralFuse(gfParams);

                FuselageSkinning skin;
                skin.enabled = true;
                skin.topLAngleDeg = readParmDouble(xsecParm, "TopLAngle", 0.0);
                skin.bottomLAngleDeg = readParmDouble(xsecParm, "BottomLAngle", skin.topLAngleDeg);
                skin.leftLAngleDeg = readParmDouble(xsecParm, "LeftLAngle", skin.topLAngleDeg);
                skin.rightLAngleDeg = readParmDouble(xsecParm, "RightLAngle", skin.leftLAngleDeg);
                skin.topRAngleDeg = readParmDouble(xsecParm, "TopRAngle", 0.0);
                skin.bottomRAngleDeg = readParmDouble(xsecParm, "BottomRAngle", skin.topRAngleDeg);
                skin.leftRAngleDeg = readParmDouble(xsecParm, "LeftRAngle", skin.topRAngleDeg);
                skin.rightRAngleDeg = readParmDouble(xsecParm, "RightRAngle", skin.leftRAngleDeg);
                skin.topLStrength = readParmDouble(xsecParm, "TopLStrength", 1.0);
                skin.bottomLStrength = readParmDouble(xsecParm, "BottomLStrength", skin.topLStrength);
                skin.leftLStrength = readParmDouble(xsecParm, "LeftLStrength", skin.topLStrength);
                skin.rightLStrength = readParmDouble(xsecParm, "RightLStrength", skin.leftLStrength);
                skin.topRStrength = readParmDouble(xsecParm, "TopRStrength", 1.0);
                skin.bottomRStrength = readParmDouble(xsecParm, "BottomRStrength", skin.topRStrength);
                skin.leftRStrength = readParmDouble(xsecParm, "LeftRStrength", skin.topRStrength);
                skin.rightRStrength = readParmDouble(xsecParm, "RightRStrength", skin.leftRStrength);
                sec.setSkinning(skin);

                fuse->appendSection(std::move(sec));
            }
        }

        fuse->setAxialSubdivisions(14);
        const VspEndCap endCap = readVspEndCap(parm);
        fuse->setNoseCapType(endCap.minType);
        fuse->setNoseCapLength(endCap.minLength);
        fuse->setNoseCapStrength(endCap.minStrength);
        fuse->setTailCapType(endCap.maxType);
        fuse->setTailCapLength(endCap.maxLength);
        fuse->setTailCapStrength(endCap.maxStrength);

        fuse->markDirty();
        fuse->ensureBuilt();
        recordBodyBounds(ctx, *fuse);
        aircraft.addComponent(std::move(fuse));
    };

    buildSingleFuselage(name, false);

    const int symFlag = static_cast<int>(std::lround(readParmDouble(parm, "Sym_Planar_Flag", 0.0)));
    const double rawY = readParmDouble(parm, "Y_Location", 0.0);
    if (symFlag == 2 && std::abs(rawY) > 1e-3) {
        buildSingleFuselage(name + " (Mirror)", true);
    }
}

void importPodGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, ImportContext& ctx, const std::string& name) {
    const std::uint64_t id = aircraft.allocateComponentId();
    auto pod = std::make_unique<Pod>(id, name);

    const pugi::xml_node parm = geomNode.child("ParmContainer");
    const double geomScale = std::max(1e-4, readParmDouble(parm, "Scale", 1.0));
    const double lenScale = ctx.unitScale * geomScale;
    pod->getTransform() = readVspTransform(parm, ctx.unitScale);

    const double rawLength = readParmDouble(parm, "Length", 3.5);
    const double fineRatio = readParmDouble(parm, "FineRatio", 15.0);
    const double lengthM = rawLength * lenScale;
    pod->length().setValue(lengthM);

    // In OpenVSP PodGeom: rad = length / FineRatio => diameter = 2 * length / FineRatio
    double diamM = (fineRatio > 0.1) ? (2.0 * lengthM / fineRatio) : 0.8;
    if (!parm.child("Design") || !parm.child("Design").child("FineRatio")) {
        const double bboxY = readParmDouble(parm.child("BBox"), "Y_Len", 0.0);
        if (bboxY > 1e-4) {
            diamM = bboxY * lenScale;
        }
    }
    pod->width().setValue(diamM);
    pod->height().setValue(diamM);

    const int symFlag = static_cast<int>(std::lround(readParmDouble(parm, "Sym_Planar_Flag", 0.0)));
    pod->setSymmetry((symFlag == 2) ? SymmetryMode::XZ : SymmetryMode::None);
    pod->markDirty();
    pod->ensureBuilt();
    recordBodyBounds(ctx, *pod);
    aircraft.addComponent(std::move(pod));
}

struct PCurveData {
    int crvType{0}; // 0=Linear, 1=PCHIP, 2=CEDIT Cubic Bezier
    std::vector<std::pair<double, double>> points;

    bool empty() const noexcept { return points.empty(); }

    static double evalCubicBezier(double p0, double p1, double p2, double p3, double t) noexcept {
        const double u = 1.0 - t;
        return u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3;
    }

    double eval(double r, double fallback) const noexcept {
        if (points.empty()) {
            return fallback;
        }
        if (r <= points.front().first) {
            return points.front().second;
        }
        if (r >= points.back().first) {
            return points.back().second;
        }

        // OpenVSP CEDIT (CrvType == 2): sequence of cubic Bezier spans (3*k + 1 points)
        if (crvType == 2 && points.size() >= 4 && ((points.size() - 1) % 3 == 0)) {
            for (std::size_t i = 0; i + 3 < points.size(); i += 3) {
                const double r0 = points[i].first;
                const double r1 = points[i + 1].first;
                const double r2 = points[i + 2].first;
                const double r3 = points[i + 3].first;
                if (r <= r3) {
                    double lo = 0.0;
                    double hi = 1.0;
                    for (int iter = 0; iter < 20; ++iter) {
                        const double mid = 0.5 * (lo + hi);
                        const double rMid = evalCubicBezier(r0, r1, r2, r3, mid);
                        if (rMid < r) {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    const double tBest = 0.5 * (lo + hi);
                    return evalCubicBezier(
                        points[i].second,
                        points[i + 1].second,
                        points[i + 2].second,
                        points[i + 3].second,
                        tBest
                    );
                }
            }
            return points.back().second;
        }

        // OpenVSP PCHIP (CrvType == 1): monotonic cubic Hermite
        if (crvType == 1 && points.size() >= 3) {
            const std::size_t n = points.size();
            std::vector<double> h(n - 1);
            std::vector<double> d(n - 1);
            for (std::size_t i = 0; i + 1 < n; ++i) {
                h[i] = std::max(1e-6, points[i + 1].first - points[i].first);
                d[i] = (points[i + 1].second - points[i].second) / h[i];
            }
            std::vector<double> m(n, 0.0);
            for (std::size_t i = 1; i + 1 < n; ++i) {
                if (d[i - 1] * d[i] > 0.0) {
                    const double w1 = 2.0 * h[i] + h[i - 1];
                    const double w2 = h[i] + 2.0 * h[i - 1];
                    m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
                }
            }
            m[0] = ((2.0 * h[0] + h[1]) * d[0] - h[0] * d[1]) / (h[0] + h[1]);
            if (m[0] * d[0] <= 0.0) {
                m[0] = 0.0;
            }
            m[n - 1] = ((2.0 * h[n - 2] + h[n - 3]) * d[n - 2] - h[n - 2] * d[n - 3]) / (h[n - 2] + h[n - 3]);
            if (m[n - 1] * d[n - 2] <= 0.0) {
                m[n - 1] = 0.0;
            }
            for (std::size_t i = 0; i + 1 < n; ++i) {
                if (r <= points[i + 1].first) {
                    const double t = std::clamp((r - points[i].first) / h[i], 0.0, 1.0);
                    const double t2 = t * t;
                    const double t3 = t2 * t;
                    return (2.0 * t3 - 3.0 * t2 + 1.0) * points[i].second +
                        (t3 - 2.0 * t2 + t) * h[i] * m[i] +
                        (-2.0 * t3 + 3.0 * t2) * points[i + 1].second +
                        (t3 - t2) * h[i] * m[i + 1];
                }
            }
            return points.back().second;
        }

        for (std::size_t i = 0; i + 1 < points.size(); ++i) {
            if (r <= points[i + 1].first) {
                const double dr = std::max(1e-6, points[i + 1].first - points[i].first);
                const double t = (r - points[i].first) / dr;
                return points[i].second + t * (points[i + 1].second - points[i].second);
            }
        }
        return points.back().second;
    }
};

PCurveData readPCurve(const pugi::xml_node& propGeomNode, const char* groupName, const char* valPrefix) {
    PCurveData data;
    const pugi::xml_node group = propGeomNode.child(groupName);
    if (!group) {
        return data;
    }
    const pugi::xml_node container = group.child("ParmContainer") ? group.child("ParmContainer").child(groupName) : group;
    if (!container) {
        return data;
    }

    data.crvType = static_cast<int>(std::lround(readParmDouble(container, "CrvType", 0.0)));

    for (int idx = 0; idx < 32; ++idx) {
        const std::string rKey = "r_" + std::to_string(idx);
        const std::string vKey = std::string(valPrefix) + "_" + std::to_string(idx);
        double r = 0.0;
        double v = 0.0;
        if (tryReadParmDouble(container, rKey.c_str(), r) && tryReadParmDouble(container, vKey.c_str(), v)) {
            data.points.emplace_back(r, v);
        } else {
            break;
        }
    }

    std::sort(data.points.begin(), data.points.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    return data;
}

void importPropGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, ImportContext& ctx, const std::string& name) {
    const std::uint64_t id = aircraft.allocateComponentId();
    auto prop = std::make_unique<Propeller>(id, name);

    const pugi::xml_node parm = geomNode.child("ParmContainer");
    const double geomScale = std::max(1e-4, readParmDouble(parm, "Scale", 1.0));
    const double lenScale = ctx.unitScale * geomScale;
    prop->getTransform() = readVspTransform(parm, ctx.unitScale);

    const double diameterM = readParmDouble(parm, "Diameter", 2.1) * lenScale;
    const double radiusM = 0.5 * diameterM;
    prop->diameter().setValue(diameterM);

    const int bladeCount = std::clamp(static_cast<int>(std::lround(readParmDouble(parm, "NumBlade", 3.0))), 2, 8);
    prop->setBladeCount(bladeCount);

    const int reverseFlag = static_cast<int>(std::lround(readParmDouble(parm, "ReverseFlag", 0.0)));
    prop->setClockwise(reverseFlag == 0);

    const double featherDeg = readParmDouble(parm, "Feather", 0.0);
    const pugi::xml_node propGeom = geomNode.child("PropellerGeom");

    std::vector<pugi::xml_node> xsecs;
    for (const pugi::xml_node xs : propGeom.child("XSecSurf").children("XSec")) {
        xsecs.push_back(xs);
    }

    double hubFrac = 0.16;
    if (!xsecs.empty()) {
        tryReadParmDouble(xsecs.front().child("ParmContainer"), "RadiusFrac", hubFrac);
    }
    hubFrac = std::clamp(hubFrac, 0.06, 0.45);
    const double hubDiamM = hubFrac * diameterM;
    prop->hubDiameter().setValue(hubDiamM);
    prop->setHasSpinner(false);
    prop->spinnerLength().setValue(std::clamp(0.08 * hubDiamM, 0.02, 0.25));

    // Apply blade airfoil from mid/outer airfoil XSec (index 0 is typically circular shank)
    pugi::xml_node foilXSec;
    for (const pugi::xml_node xs : xsecs) {
        const ResolvedXSecCurve curve = resolveXSecCurve(xs);
        if (curve.type >= 7) {
            foilXSec = xs;
            break;
        }
    }
    if (!foilXSec && !xsecs.empty()) {
        foilXSec = xsecs.back();
    }
    if (foilXSec) {
        applyVspAirfoil(foilXSec, prop->getBladeAirfoil());
    }

    const PCurveData chordCurve = propGeom ? readPCurve(propGeom, "Chord", "crd") : PCurveData{};
    const PCurveData twistCurve = propGeom ? readPCurve(propGeom, "Twist", "tw") : PCurveData{};
    const PCurveData sweepCurve = propGeom ? readPCurve(propGeom, "Sweep", "sw") : PCurveData{};
    const PCurveData skewCurve = propGeom ? readPCurve(propGeom, "Skew", "skw") : PCurveData{};
    const PCurveData tanCurve = propGeom ? readPCurve(propGeom, "Tangential", "tan") : PCurveData{};
    const PCurveData rakeCurve = propGeom ? readPCurve(propGeom, "Rake", "rak") : PCurveData{};
    const PCurveData thickCurve = propGeom ? readPCurve(propGeom, "Thick", "toc") : PCurveData{};

    if (!chordCurve.empty() || !twistCurve.empty()) {
        const double rMin = !chordCurve.empty() ? chordCurve.points.front().first : hubFrac;
        const double rMax = !chordCurve.empty() ? chordCurve.points.back().first : 1.0;

        constexpr int kCurveSamples = 21;
        std::vector<Propeller::BladeStation> stations;
        stations.reserve(kCurveSamples);

        const double fallbackCrd = readParmDouble(parm, "Chord", 0.15);
        const double beta34 = readParmDouble(parm, "Beta34", 24.0);

        for (int i = 0; i < kCurveSamples; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(kCurveSamples - 1);
            const double r = rMin + u * (rMax - rMin);
            Propeller::BladeStation st;
            st.radiusFrac = r;
            st.chord = std::max(0.01, chordCurve.eval(r, fallbackCrd) * radiusM);
            st.twistDeg = twistCurve.empty() ? beta34 : (twistCurve.eval(r, beta34) + featherDeg);
            const double tangOffset = !tanCurve.empty()
                ? tanCurve.eval(r, 0.0)
                : (sweepCurve.eval(r, 0.0) + skewCurve.eval(r, 0.0));
            st.sweepOffset = tangOffset * radiusM;
            st.rakeOffset = rakeCurve.eval(r, 0.0) * radiusM;
            if (!thickCurve.empty()) {
                st.thicknessRatio = std::clamp(thickCurve.eval(r, 0.12), 0.02, 0.85);
            }
            stations.push_back(st);
        }

        prop->rootChord().setValue(stations.front().chord);
        prop->tipChord().setValue(stations.back().chord);
        prop->collectivePitchDeg().setValue(twistCurve.empty() ? beta34 : (twistCurve.eval(0.75, beta34) + featherDeg));
        prop->washoutDeg().setValue(stations.front().twistDeg - stations.back().twistDeg);
        prop->setBladeCurve(std::move(stations));
    } else {
        const double crdNorm = readParmDouble(parm, "Chord", 0.14);
        const double estChordM = crdNorm * radiusM;
        prop->rootChord().setValue(std::clamp(estChordM, 0.02, 3.0));
        prop->tipChord().setValue(std::clamp(estChordM * 0.72, 0.02, 3.0));
        prop->collectivePitchDeg().setValue(readParmDouble(parm, "Beta34", 24.0));
    }

    const VspEndCap propCap = readVspEndCap(parm);
    prop->setTipCapType(propCap.maxType);

    prop->markDirty();
    prop->ensureBuilt();
    aircraft.addComponent(std::move(prop));
}

void importGearGeom(const pugi::xml_node& geomNode, Aircraft& aircraft, const ImportContext& ctx, const std::string& name) {
    const pugi::xml_node parm = geomNode.child("ParmContainer");
    const double geomScale = std::max(1e-4, readParmDouble(parm, "Scale", 1.0));
    int gearUnits = -1;
    double rawUnitVal = -1.0;
    if (tryReadParmDouble(parm.child("Gear"), "m_ModelLenUnits", rawUnitVal)) {
        gearUnits = static_cast<int>(std::lround(rawUnitVal));
    }
    const double baseUnitScale = (gearUnits >= 0) ? vspLenUnitToMeters(gearUnits) : ctx.unitScale;
    const double lenScale = baseUnitScale * geomScale;

    const Transform geomXf = readVspTransform(parm, baseUnitScale);
    const glm::vec3 geomOrigin = geomXf.getPosition();

    const pugi::xml_node gearNode = geomNode.child("Gear");
    if (!gearNode) {
        return;
    }

    std::vector<pugi::xml_node> bogies;
    for (const pugi::xml_node child : gearNode.children()) {
        const std::string tag = child.name();
        if (tag.rfind("Bogie_", 0) == 0) {
            bogies.push_back(child);
        }
    }

    for (std::size_t bIdx = 0; bIdx < bogies.size(); ++bIdx) {
        const pugi::xml_node bParm = bogies[bIdx].child("ParmContainer");
        if (!bParm) {
            continue;
        }
        const pugi::xml_node bogieGroup = bParm.child("Bogie");
        const pugi::xml_node tireGroup = bParm.child("Tire");

        const double xContact = readParmDouble(bogieGroup, "XContactPt", 0.0) * lenScale;
        const double yContact = readParmDouble(bogieGroup, "YContactPt", 0.0) * lenScale;
        const double zAboveGround = readParmDouble(bogieGroup, "ZAboveGround", 0.0) * lenScale;
        const double travelZ = readParmDouble(bogieGroup, "TravelZ", 0.5) * lenScale;
        const int numAcross = std::clamp(static_cast<int>(std::lround(readParmDouble(bogieGroup, "NumAcross", 1.0))), 1, 2);
        const bool symmetrical = readParmDouble(bogieGroup, "Symmetrical", 0.0) >= 0.5;

        double tireDiamM = 0.0;
        if (tryReadParmDouble(tireGroup, "DiameterModel", tireDiamM) && tireDiamM > 1e-4) {
            tireDiamM *= lenScale;
        } else {
            tireDiamM = readParmDouble(tireGroup, "DiameterIn", 20.0) * 0.0254 * geomScale;
        }

        double tireWidthM = 0.0;
        if (tryReadParmDouble(tireGroup, "WidthModel", tireWidthM) && tireWidthM > 1e-4) {
            tireWidthM *= lenScale;
        } else {
            tireWidthM = readParmDouble(tireGroup, "WidthIn", 7.0) * 0.0254 * geomScale;
        }

        tireDiamM = std::clamp(tireDiamM, 0.05, 5.0);
        tireWidthM = std::clamp(tireWidthM, 0.02, 2.0);

        const float wx = geomOrigin.x + static_cast<float>(xContact);
        const float wy = geomOrigin.y + static_cast<float>(yContact);
        const float wzBottom = geomOrigin.z + static_cast<float>(zAboveGround);
        const float wzAxle = wzBottom + 0.5f * static_cast<float>(tireDiamM);

        double attachZ = wzAxle + std::max(0.75 * tireDiamM, travelZ + 0.25 * tireDiamM);
        float lowestBodyBottom = 1e9f;
        for (const BodyBoundsXY& bb : ctx.bodyBounds) {
            if (wx >= bb.minPt.x - 0.5f && wx <= bb.maxPt.x + 0.5f &&
                wy >= bb.minPt.y - 0.5f && wy <= bb.maxPt.y + 0.5f) {
                if (bb.maxPt.z > wzAxle) {
                    lowestBodyBottom = std::min(lowestBodyBottom, std::max(bb.minPt.z, wzAxle + 0.2f));
                }
            }
        }
        if (lowestBodyBottom < 1e8f) {
            attachZ = std::max(attachZ, static_cast<double>(lowestBodyBottom) + 0.15 * tireDiamM);
        }

        const double strutLen = std::clamp(attachZ - static_cast<double>(wzAxle), 0.15, 15.0);
        const double topZ = static_cast<double>(wzAxle) + strutLen;

        std::string gearName = name;
        if (bogies.size() > 1) {
            gearName += " Bogie " + std::to_string(bIdx + 1);
        }

        const std::uint64_t id = aircraft.allocateComponentId();
        auto gear = std::make_unique<LandingGear>(id, gearName);
        gear->getTransform().setPosition(glm::vec3(wx, wy, static_cast<float>(topZ)));
        gear->strutLength().setValue(strutLen);
        gear->strutDiameter().setValue(std::clamp(0.18 * tireWidthM, 0.03, 0.45));
        gear->rakeDeg().setValue(0.0);
        gear->splayDeg().setValue(0.0);
        gear->wheelDiameter().setValue(tireDiamM);
        gear->wheelWidth().setValue(tireWidthM);
        gear->setWheelsPerStrut(numAcross);
        gear->setWheelPant(false);
        gear->setSymmetry(symmetrical ? SymmetryMode::XZ : SymmetryMode::None);

        gear->markDirty();
        gear->ensureBuilt();
        aircraft.addComponent(std::move(gear));
    }
}

void collectGeoms(const pugi::xml_node& rootNode, std::vector<pugi::xml_node>& outGeoms) {
    for (const pugi::xml_node geom : rootNode.children("Geom")) {
        if (!geom.child("GeomBase") && !geom.child("ParmContainer") && !geom.child("WingGeom") &&
            !geom.child("FuselageGeom") && !geom.child("PodGeom") && !geom.child("PropellerGeom") &&
            !geom.child("Gear")) {
            continue;
        }
        outGeoms.push_back(geom);
        collectGeoms(geom, outGeoms);
    }
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

    ImportContext ctx;
    double lenUnitRaw = 2.0;
    if (vehicle && tryReadParmDouble(vehicle.child("ParmContainer").child("STEPSettings"), "LenUnit", lenUnitRaw)) {
        ctx.unitScale = vspLenUnitToMeters(static_cast<int>(std::lround(lenUnitRaw)));
    }

    if (aircraft.getComponents().empty() && aircraft.getName() == "Untitled Aircraft") {
        aircraft.setName(fallbackName);
    }

    std::vector<pugi::xml_node> geoms;
    collectGeoms(searchRoot, geoms);

    std::size_t importedCount = 0;
    std::vector<pugi::xml_node> deferredGears;

    for (const pugi::xml_node geom : geoms) {
        const pugi::xml_node parm = geom.child("ParmContainer");
        const std::string geomName = readParmString(parm, "Name", "VSP Geom");
        const pugi::xml_node geomBase = geom.child("GeomBase");
        const std::string typeName = geomBase ? geomBase.child_value("TypeName") : "";

        if (typeName == "Wing" || geom.child("WingGeom")) {
            importWingGeom(geom, aircraft, ctx, geomName);
            ++importedCount;
        } else if (typeName == "Fuselage" || typeName == "Stack" || geom.child("FuselageGeom") || geom.child("StackGeom")) {
            importFuselageGeom(geom, aircraft, ctx, geomName);
            ++importedCount;
        } else if (typeName == "Pod" || geom.child("PodGeom")) {
            importPodGeom(geom, aircraft, ctx, geomName);
            ++importedCount;
        } else if (typeName == "Propeller" || geom.child("PropellerGeom")) {
            importPropGeom(geom, aircraft, ctx, geomName);
            ++importedCount;
        } else if (typeName == "Gear" || geom.child("Gear")) {
            deferredGears.push_back(geom);
        }
    }

    for (const pugi::xml_node gearGeom : deferredGears) {
        const pugi::xml_node parm = gearGeom.child("ParmContainer");
        const std::string geomName = readParmString(parm, "Name", "Landing Gear");
        importGearGeom(gearGeom, aircraft, ctx, geomName);
        ++importedCount;
    }

    if (importedCount > 0 && !aircraft.getComponents().empty()) {
        aircraft.setSelectedComponentId(aircraft.getComponents().back()->getId());
    }
    return importedCount > 0;
}

} // namespace opads
