#pragma once
#include "core/utils/controls/pid.h"
#include "core/subsystems/tank_drive.h"
#include "vex.h"

class PIDTuner{
    public:
    PIDTuner(PID pid, TankDrive &drive_sys);
    void SetSetpoint(double setpoint);
    double GetSetpoint();
    void StartTuning();
    void StopTuning();
    bool IsTuning();
    PID pid;
    private:
    bool tuning = false;
    double setpoint = 0;
    TankDrive drive_sys;
    vex::task task;
    static int thread_fn(void *ptr);

};