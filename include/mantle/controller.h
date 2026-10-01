#pragma once

#include <string>

namespace mantle {

enum class ControllerAxis {
    LeftY,
    LeftX,
    RightY,
    RightX
};

enum class ControllerButton {
    L1,
    L2,
    R1,
    R2,
    Up,
    Down,
    Left,
    Right,
    X,
    B,
    Y,
    A
};

/**
 * Hardware Abstraction Layer interface for game controller screens.
 */
class ControllerScreen {
public:
    virtual ~ControllerScreen() = default;
    virtual void clearScreen() = 0;
    virtual void clearLine(int row = 0) = 0;
    virtual void setCursor(int row, int col) = 0;
    virtual void print(const char* str) = 0;
    virtual void newLine() = 0;
};

class MockControllerScreen : public ControllerScreen {
public:
    void clearScreen() override {}
    void clearLine(int = 0) override {}
    void setCursor(int, int) override {}
    void print(const char*) override {}
    void newLine() override {}
};

/**
 * Hardware Abstraction Layer interface for game controllers.
 */
class Controller {
public:
    struct ButtonProxy {
        Controller* c;
        ControllerButton b;
        bool pressing() const { return c ? c->button(b) : false; }
    };

    struct AxisProxy {
        Controller* c;
        ControllerAxis a;
        double position() const { return c ? c->axis(a) * 100.0 : 0.0; }
    };

    struct ScreenProxy {
        Controller* c;
        void clearScreen() { if (c) c->screen().clearScreen(); }
        void clearLine(int row = 0) { if (c) c->screen().clearLine(row); }
        void setCursor(int row, int col) { if (c) c->screen().setCursor(row, col); }
        void newLine() { if (c) c->screen().newLine(); }
        void print(const char* str) {
            if (c) c->screen().print(str);
        }
        template<typename... Args>
        void print(const char* fmt, Args... args) {
            if (!c) return;
            char buf[128];
            snprintf(buf, sizeof(buf), fmt, args...);
            c->screen().print(buf);
        }
    };


    ButtonProxy ButtonA{this, ControllerButton::A};
    ButtonProxy ButtonB{this, ControllerButton::B};
    ButtonProxy ButtonX{this, ControllerButton::X};
    ButtonProxy ButtonY{this, ControllerButton::Y};
    ButtonProxy ButtonUp{this, ControllerButton::Up};
    ButtonProxy ButtonDown{this, ControllerButton::Down};
    ButtonProxy ButtonLeft{this, ControllerButton::Left};
    ButtonProxy ButtonRight{this, ControllerButton::Right};
    ButtonProxy ButtonL1{this, ControllerButton::L1};
    ButtonProxy ButtonL2{this, ControllerButton::L2};
    ButtonProxy ButtonR1{this, ControllerButton::R1};
    ButtonProxy ButtonR2{this, ControllerButton::R2};

    AxisProxy Axis1{this, ControllerAxis::RightX};
    AxisProxy Axis2{this, ControllerAxis::RightY};
    AxisProxy Axis3{this, ControllerAxis::LeftY};
    AxisProxy Axis4{this, ControllerAxis::LeftX};

    ScreenProxy Screen{this};

    virtual ~Controller() = default;

    virtual double axis(ControllerAxis a) = 0;
    virtual bool button(ControllerButton b) = 0;
    virtual void rumble(const char* pattern) = 0;
    virtual ControllerScreen& screen() = 0;
};

/**
 * In-memory mock controller for testing.
 */
class MockController : public Controller {
private:
    MockControllerScreen mock_screen;

public:
    double left_y = 0.0;
    double left_x = 0.0;
    double right_y = 0.0;
    double right_x = 0.0;

    ControllerScreen& screen() override { return mock_screen; }

    double axis(ControllerAxis a) override {
        switch (a) {
            case ControllerAxis::LeftY: return left_y;
            case ControllerAxis::LeftX: return left_x;
            case ControllerAxis::RightY: return right_y;
            case ControllerAxis::RightX: return right_x;
        }
        return 0.0;
    }

    bool button(ControllerButton) override {
        return false;
    }

    void rumble(const char*) override {}
};


} // namespace mantle
