#pragma once

#include <string>

namespace opads {

/// Abstract base class for dockable editor panels.
class Panel {
public:
    explicit Panel(std::string title);
    virtual ~Panel() = default;

    Panel(const Panel&) = delete;
    Panel& operator=(const Panel&) = delete;

    const std::string& getTitle() const noexcept;
    bool isOpen() const noexcept;
    void setOpen(bool open) noexcept;

    virtual void onRender() = 0;

protected:
    std::string m_title;
    bool m_open{true};
};

} // namespace opads
