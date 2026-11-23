#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/Component.hpp"

namespace opads {

/// Root parametric aircraft container owning all components and design metadata.
class Aircraft {
public:
    explicit Aircraft(std::string name = "Untitled Aircraft");

    const std::string& getName() const noexcept;
    void setName(std::string name);

    std::uint64_t allocateComponentId() noexcept;

    Component* addComponent(std::unique_ptr<Component> component);
    bool removeComponent(std::uint64_t id);

    Component* findComponent(std::uint64_t id) noexcept;
    const Component* findComponent(std::uint64_t id) const noexcept;

    const std::vector<std::unique_ptr<Component>>& getComponents() const noexcept;

    std::optional<std::uint64_t> getSelectedComponentId() const noexcept;
    void setSelectedComponentId(std::optional<std::uint64_t> id) noexcept;

    void rebuildDirty();

private:
    std::string m_name;
    std::uint64_t m_nextComponentId{1};
    std::optional<std::uint64_t> m_selectedComponentId;
    std::vector<std::unique_ptr<Component>> m_components;
};

} // namespace opads
