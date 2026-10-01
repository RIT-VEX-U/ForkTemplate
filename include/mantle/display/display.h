#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

namespace mantle {

struct Color {
    uint32_t rgb;
    constexpr Color(uint32_t c = 0x000000) : rgb(c) {}
    constexpr Color(uint8_t r, uint8_t g, uint8_t b)
        : rgb((static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b)) {}
    explicit Color(const std::string &hex) {
        const char *p = hex.c_str();
        if (*p == '#') p++;
        rgb = static_cast<uint32_t>(strtoul(p, nullptr, 16));
    }

    static const Color White;
    static const Color Black;
    static const Color Red;
    static const Color Green;
    static const Color Blue;
    static const Color Yellow;
    static const Color Cyan;
    static const Color Orange;
    static const Color Purple;
    static const Color Transparent;

    // Lowercase aliases for backward compatibility
    static const Color white;
    static const Color black;
    static const Color red;
    static const Color green;
    static const Color blue;
    static const Color yellow;
    static const Color cyan;
    static const Color orange;
    static const Color purple;
    static const Color transparent;
};

inline const Color Color::White(0xFFFFFF);
inline const Color Color::Black(0x000000);
inline const Color Color::Red(0xFF0000);
inline const Color Color::Green(0x00FF00);
inline const Color Color::Blue(0x0000FF);
inline const Color Color::Yellow(0xFFFF00);
inline const Color Color::Cyan(0x00FFFF);
inline const Color Color::Orange(0xFFA500);
inline const Color Color::Purple(0x800080);
inline const Color Color::Transparent(0x00000000);

inline const Color Color::white(0xFFFFFF);
inline const Color Color::black(0x000000);
inline const Color Color::red(0xFF0000);
inline const Color Color::green(0x00FF00);
inline const Color Color::blue(0x0000FF);
inline const Color Color::yellow(0xFFFF00);
inline const Color Color::cyan(0x00FFFF);
inline const Color Color::orange(0xFFA500);
inline const Color Color::purple(0x800080);
inline const Color Color::transparent(0x00000000);

/**
 * Hardware Abstraction Layer interface for graphical LCD screens.
 */
class Display {
public:
    virtual ~Display() = default;

    virtual void draw_line(int x1, int y1, int x2, int y2) = 0;
    virtual void draw_rectangle(int x, int y, int width, int height) = 0;
    virtual void fill_rectangle(int x, int y, int width, int height) = 0;
    virtual void draw_circle(int x, int y, int radius, Color color = Color::White) {}
    virtual void fill_circle(int x, int y, int radius, Color color = Color::White) {}
    virtual void print_at(int x, int y, const char* text) = 0;
    virtual void set_pen_color(Color c) = 0;
    virtual void set_fill_color(Color c) = 0;
    virtual void set_pen_width(int) {}
    virtual void set_font(int) {}
    virtual void clear_screen() = 0;

    virtual bool pressing() { return false; }
    virtual int touch_x() { return 0; }
    virtual int touch_y() { return 0; }

    // CamelCase compatibility methods
    void drawLine(int x1, int y1, int x2, int y2) { draw_line(x1, y1, x2, y2); }
    void drawRectangle(int x, int y, int w, int h) { draw_rectangle(x, y, w, h); }
    void fillRectangle(int x, int y, int w, int h) { fill_rectangle(x, y, w, h); }
    void drawCircle(int x, int y, int r, Color c = Color::White) { draw_circle(x, y, r, c); }
    void fillCircle(int x, int y, int r, Color c = Color::White) { fill_circle(x, y, r, c); }
    void setPenColor(Color c) { set_pen_color(c); }
    void setPenColor(const char* hex) { set_pen_color(Color(hex)); }
    void setFillColor(Color c) { set_fill_color(c); }
    void setFillColor(const char* hex) { set_fill_color(Color(hex)); }
    void setPenWidth(int w) { set_pen_width(w); }
    void setFont(int font_id) { set_font(font_id); }
    virtual void draw_image_from_buffer(const void*, int, int, int) {}
    virtual void draw_image_from_buffer(const void*, int, int, int, int) {}
    void drawImageFromBuffer(const void* b, int x, int y, int s) { draw_image_from_buffer(b, x, y, s); }
    void drawImageFromBuffer(const void* b, int x, int y, int w, int h) { draw_image_from_buffer(b, x, y, w, h); }
    void clearScreen() { clear_screen(); }
    void printAt(int x, int y, const char* text) { print_at(x, y, text); }
    void printAt(int x, int y, bool, const char* text) { print_at(x, y, text); }

    template <typename... Args>
    void printAt(int x, int y, bool, const char* fmt, Args... args) {
        char buf[256];
        snprintf(buf, sizeof(buf), fmt, args...);
        print_at(x, y, buf);
    }

    template <typename... Args>
    void printAt(int x, int y, const char* fmt, Args... args) {
        char buf[256];
        snprintf(buf, sizeof(buf), fmt, args...);
        print_at(x, y, buf);
    }

    virtual uint32_t getStringHeight(const char*) { return 12; }
    virtual uint32_t getStringWidth(const char* s) { return s ? static_cast<uint32_t>(strlen(s) * 8) : 0; }
};

/**
 * Mock screen implementation for testing without physical V5 Brain.
 */
class MockDisplay : public Display {
public:
    void draw_line(int, int, int, int) override {}
    void draw_rectangle(int, int, int, int) override {}
    void fill_rectangle(int, int, int, int) override {}
    void print_at(int, int, const char*) override {}
    void set_pen_color(Color) override {}
    void set_fill_color(Color) override {}
    void clear_screen() override {}
};

} // namespace mantle
