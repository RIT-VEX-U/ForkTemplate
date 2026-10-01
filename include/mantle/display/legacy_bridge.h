#pragma once

#include <initializer_list>
#include <vector>
#include <memory>

#include "mantle/display/legacy.h"
#include "mantle/display/screen_controller.h"

namespace LegacyScreen {

class LegacyPage {
public:
    LegacyPage(mantle::Display &screen, std::initializer_list<Page *> pages);
    LegacyPage(mantle::Display &screen, std::vector<Page *> pages);
    LegacyPage() = default;

    void set_display(mantle::Display &disp) { screen = &disp; }

    inline std::function<void()> handle() {
        return [this]() {
            if (!this->screen) return;
            if(this->pages.empty()) {
                this->screen->clear_screen();
                return;
            }

            Page *front_page = this->pages[this->index];
            std::optional<Point2d> pressing;

            if((pressing = ScreenController::get_press_pos()).has_value()) {
                this->x_press = pressing->x();
                this->y_press = pressing->y();
            }
            bool just_pressed = pressing.has_value() && !this->was_pressed;

            if (just_pressed && this->x_press < 40) {
                this->index--;
                if (this->index < 0) {
                    this->index += this->pages.size();
                }
            }
            if (just_pressed && this->x_press > 440) {
                this->index++;
                this->index %= this->pages.size();
            }

            // Update all pages
            for (auto page : this->pages) {
                if (page == front_page) {
                    page->update(this->was_pressed, this->x_press, this->y_press);
                } else {
                    page->update(false, 0, 0);
                }
            }

            // Draw First Page
            int frame = ScreenController::get_frame_count();
            if (frame % 2 == 0) {
                this->screen->clear_screen();
                this->screen->set_pen_color(mantle::Color::White);
                this->screen->set_fill_color(mantle::Color::Black);
                front_page->draw(*this->screen, false, frame / 5);

                // Draw side boxes
                this->screen->set_pen_color(mantle::Color(0x202020));
                this->screen->set_fill_color(mantle::Color(0x202020));
                this->screen->draw_rectangle(0, 0, 40, 240);
                this->screen->draw_rectangle(440, 0, 40, 240);
                this->screen->set_pen_color(mantle::Color::White);
                // left arrow
                this->screen->draw_line(30, 100, 15, 120);
                this->screen->draw_line(30, 140, 15, 120);
                // right arrow
                this->screen->draw_line(450, 100, 465, 120);
                this->screen->draw_line(450, 140, 465, 120);
            }

            this->was_pressed = pressing.has_value();
        };
    }

private:
    bool was_pressed = false;
    int index = 0;
    int x_press = 0;
    int y_press = 0;

    std::vector<Page *> pages;
    mantle::Display* screen = nullptr;
};

inline std::function<void()> pre_initialize(mantle::Display& screen, mantle::Initializer& initializer, LegacyPage* page, std::function<void()> o = nullptr) {
    return [&screen, o, page, &initializer]() {
        if(o) o();

        std::vector<Page*> pages; size_t initializations = 0;
        do {
            pages.push_back(new InitializerPage(initializer, initializations));
            initializations += 8;
        } while(initializations < initializer.initialization_count());

        *page = LegacyPage(screen, pages);
        ScreenController::set(page->handle());
    };
}

} // namespace LegacyScreen