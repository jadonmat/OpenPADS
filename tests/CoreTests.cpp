#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/Aircraft.hpp"
#include "core/Component.hpp"
#include "core/Transform.hpp"

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

TEST_CASE("Aircraft manages component ownership, dirty rebuilds, and selection", "[core][aircraft]") {
    opads::Aircraft aircraft("Test Jet");
    REQUIRE(aircraft.getName() == "Test Jet");

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

    // Calling rebuildDirty again without marking dirty should be a no-op.
    aircraft.rebuildDirty();
    REQUIRE(rawPtr->buildCount == 1);

    aircraft.setSelectedComponentId(id);
    REQUIRE(aircraft.getSelectedComponentId() == id);
    REQUIRE(aircraft.removeComponent(id));
    REQUIRE_FALSE(aircraft.getSelectedComponentId().has_value());
    REQUIRE(aircraft.getComponents().empty());
}
