#include "mantle/display/legacy_bridge.h"

namespace LegacyScreen {

LegacyPage::LegacyPage(mantle::Display &screen, std::initializer_list<Page *> pages)
    : was_pressed(false), index(0), x_press(0), y_press(0), pages(pages), screen(&screen) {}

LegacyPage::LegacyPage(mantle::Display &screen, std::vector<Page *> pages)
    : was_pressed(false), index(0), x_press(0), y_press(0), pages(pages), screen(&screen) {}

} // namespace LegacyScreen