#pragma once

#include "mantle/controller.h"
#include "vex.h"

namespace crust {

/**
 * Concrete VEX V5 Controller Screen driver implementing mantle::ControllerScreen.
 */
class V5ControllerScreen : public mantle::ControllerScreen {
private:
    vex::controller& c;

public:
    explicit V5ControllerScreen(vex::controller& c) : c(c) {}

    void clearScreen() override { c.Screen.clearScreen(); }
    void clearLine(int row = 0) override {
        if (row > 0) c.Screen.clearLine(row);
        else c.Screen.clearLine();
    }
    void setCursor(int row, int col) override { c.Screen.setCursor(row, col); }
    void print(const char* str) override { c.Screen.print("%s", str); }
    void newLine() override { c.Screen.newLine(); }
};

/**
 * Concrete VEX V5 Controller driver implementing mantle::Controller.
 */
class V5Controller : public mantle::Controller {
private:
    vex::controller controller;
    V5ControllerScreen screen_impl;

public:
    explicit V5Controller(vex::controllerType type = vex::controllerType::primary)
        : controller(type), screen_impl(controller) {}

    mantle::ControllerScreen& screen() override {
        return screen_impl;
    }


    double axis(mantle::ControllerAxis a) override {
        switch (a) {
            case mantle::ControllerAxis::LeftY: return controller.Axis3.position() / 100.0;
            case mantle::ControllerAxis::LeftX: return controller.Axis4.position() / 100.0;
            case mantle::ControllerAxis::RightY: return controller.Axis2.position() / 100.0;
            case mantle::ControllerAxis::RightX: return controller.Axis1.position() / 100.0;
        }
        return 0.0;
    }

    bool button(mantle::ControllerButton b) override {
        switch (b) {
            case mantle::ControllerButton::L1: return controller.ButtonL1.pressing();
            case mantle::ControllerButton::L2: return controller.ButtonL2.pressing();
            case mantle::ControllerButton::R1: return controller.ButtonR1.pressing();
            case mantle::ControllerButton::R2: return controller.ButtonR2.pressing();
            case mantle::ControllerButton::Up: return controller.ButtonUp.pressing();
            case mantle::ControllerButton::Down: return controller.ButtonDown.pressing();
            case mantle::ControllerButton::Left: return controller.ButtonLeft.pressing();
            case mantle::ControllerButton::Right: return controller.ButtonRight.pressing();
            case mantle::ControllerButton::X: return controller.ButtonX.pressing();
            case mantle::ControllerButton::B: return controller.ButtonB.pressing();
            case mantle::ControllerButton::Y: return controller.ButtonY.pressing();
            case mantle::ControllerButton::A: return controller.ButtonA.pressing();
        }
        return false;
    }

    void rumble(const char* pattern) override {
        controller.rumble(pattern);
    }

    vex::controller& raw_controller() { return controller; }
};

} // namespace crust
