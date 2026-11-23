#include "ui/Panel.hpp"

#include <utility>

namespace opads {

Panel::Panel(std::string title)
    : m_title(std::move(title)) {
}

const std::string& Panel::getTitle() const noexcept {
    return m_title;
}

bool Panel::isOpen() const noexcept {
    return m_open;
}

void Panel::setOpen(bool open) noexcept {
    m_open = open;
}

} // namespace opads
