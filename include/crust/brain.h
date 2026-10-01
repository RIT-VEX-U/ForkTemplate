#pragma once

#include "mantle/display/display.h"
#include "mantle/display/legacy_bridge.h"
#include "mantle/initializer.h"
#include "vex.h"

namespace crust {

/**
 * Concrete VEX Brain Screen driver implementing mantle::Display.
 */
class BrainScreen : public mantle::Display {
private:
    vex::brain::lcd& screen;

public:
    explicit BrainScreen(vex::brain::lcd& lcd) : screen(lcd) {}

    void draw_line(int x1, int y1, int x2, int y2) override {
        screen.drawLine(x1, y1, x2, y2);
    }

    void draw_rectangle(int x, int y, int width, int height) override {
        screen.drawRectangle(x, y, width, height);
    }

    void fill_rectangle(int x, int y, int width, int height) override {
        screen.drawRectangle(x, y, width, height);
    }

    void draw_circle(int x, int y, int radius, mantle::Color color = mantle::Color::White) override {
        screen.setPenColor(vex::color(color.rgb));
        screen.drawCircle(x, y, radius);
    }

    void fill_circle(int x, int y, int radius, mantle::Color color = mantle::Color::White) override {
        screen.setFillColor(vex::color(color.rgb));
        screen.drawCircle(x, y, radius);
    }

    void set_pen_width(int width) override {
        screen.setPenWidth(width);
    }

    void set_font(int) override {
        screen.setFont(vex::fontType::mono20);
    }

    void draw_image_from_buffer(const void* buf, int x, int y, int size) override {
        screen.drawImageFromBuffer((uint8_t*)buf, x, y, size);
    }

    void draw_image_from_buffer(const void* buf, int x, int y, int w, int h) override {
        screen.drawImageFromBuffer((uint32_t*)buf, x, y, w, h);
    }

    uint32_t getStringHeight(const char* s) override {
        return screen.getStringHeight(s);
    }

    uint32_t getStringWidth(const char* s) override {
        return screen.getStringWidth(s);
    }

    void print_at(int x, int y, const char* text) override {
        screen.printAt(x, y, text);
    }

    void set_pen_color(mantle::Color c) override {
        screen.setPenColor(vex::color(c.rgb));
    }

    void set_fill_color(mantle::Color c) override {
        screen.setFillColor(vex::color(c.rgb));
    }

    void clear_screen() override {
        screen.clearScreen();
    }

    bool pressing() override {
        return screen.pressing();
    }

    int touch_x() override {
        return screen.xPosition();
    }

    int touch_y() override {
        return screen.yPosition();
    }

    vex::brain::lcd& raw_screen() { return screen; }
};

} // namespace crust

namespace LegacyScreen {

inline std::function<void()> pre_initialize(vex::brain& brain, mantle::Initializer& initializer, LegacyPage* page, std::function<void()> o = nullptr) {
    static crust::BrainScreen s_brain_screen(brain.Screen);
    return pre_initialize(s_brain_screen, initializer, page, o);
}

} // namespace LegacyScreen
