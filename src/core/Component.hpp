#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/Mesh.hpp"
#include "core/Transform.hpp"

namespace opads {

/// Abstract base class for all parametric aircraft components (Wing, Fuselage, Pod, etc.).
class Component {
public:
    Component(std::uint64_t id, std::string name);
    virtual ~Component() = default;

    Component(const Component&) = delete;
    Component& operator=(const Component&) = delete;
    Component(Component&&) = default;
    Component& operator=(Component&&) = default;

    std::uint64_t getId() const noexcept;

    const std::string& getName() const noexcept;
    void setName(std::string name);

    bool isVisible() const noexcept;
    void setVisible(bool visible) noexcept;

    bool isDirty() const noexcept;
    void markDirty() noexcept;

    Transform& getTransform() noexcept;
    const Transform& getTransform() const noexcept;

    const Mesh& getMesh() const noexcept;

    /// Rebuilds the tessellated mesh when dirty.
    void ensureBuilt();

    /// Optional local-space 3D polylines highlighted when this component is selected.
    virtual std::vector<std::vector<glm::vec3>> getHighlightLoops() const;

    /// Local-space key geometry/surface sample points used for geometry-based snapping.
    virtual std::vector<glm::vec3> getSnapSamplePointsLocal() const;

    virtual std::string getTypeName() const = 0;

protected:
    virtual void rebuildMesh() = 0;

    Mesh m_mesh;

private:
    std::uint64_t m_id;
    std::string m_name;
    bool m_visible{true};
    bool m_dirty{true};
    Transform m_transform;
};

} // namespace opads
