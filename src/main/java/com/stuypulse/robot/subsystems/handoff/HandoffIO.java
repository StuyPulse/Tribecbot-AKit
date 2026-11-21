/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface HandoffIO {
    @AutoLog
    public static class HandoffIOInputs {
        public Current motorLeadSupplyCurrent = Amps.zero();
        public Current motorLeadStatorCurrent = Amps.zero();
        public Temperature motorLeadTemperature = Celsius.zero();
        public AngularVelocity motorLeadVelocity = DegreesPerSecond.zero();
        public Voltage motorLeadAppliedVoltage = Volts.zero();

        public Current motorFollowSupplyCurrent = Amps.zero();
        public Current motorFollowStatorCurrent = Amps.zero();
        public Temperature motorFollowTemperature = Celsius.zero();
        public AngularVelocity motorFollowVelocity = DegreesPerSecond.zero();
        public Voltage motorFollowAppliedVoltage = Volts.zero();
    }

    public enum HandoffIOOutputMode {
        DUTY_CYCLE,
        STOP
    }

    public static class HandoffIOOutputs {
        @AutoLogOutput(key = "Handoff/Output Mode")
        public HandoffIOOutputMode handoffMode = HandoffIOOutputMode.DUTY_CYCLE;

        @AutoLogOutput(key = "Handoff/Target Duty Cycle")
        public double handoffDutyCycle = 0.0;
    }

    public default void updateInputs(HandoffIOInputs inputs) {}

    public default void applyOutputs(HandoffIOOutputs outputs) {}
}
