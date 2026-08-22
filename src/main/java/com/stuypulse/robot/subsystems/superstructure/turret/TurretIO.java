/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public Current turretMotorSupplyCurrent = Amps.zero();
        public Current turretMotorStatorCurrent = Amps.zero();
        public Temperature turretMotorTemperature = Celsius.zero();
        public Angle turretMotorPosition = Degrees.zero();
        public Voltage turretMotorAppliedVoltage = Volts.zero();
        public AngularVelocity turretMotorVelocity = RPM.zero();

        public Angle encoder17tPosition = Degrees.zero();
        public Angle encoder18tPosition = Degrees.zero();

        public double encoder17tMagnetOffset = 0;
        public double encoder18tMagnetOffset = 0;
    }

    public enum TurretIOOutputMode {
        POSITION,
        STOP
    }

    public static class TurretIOOutputs {
        @AutoLogOutput(key = "Superstructure/Turret/Output Mode")
        public TurretIOOutputMode turretMode = TurretIOOutputMode.POSITION;

        @AutoLogOutput(key = "Superstructure/Turret/Target Position")
        public Angle turretPosition = Degrees.zero();

        @AutoLogOutput(key = "Superstructure/Turret/Gain Slot")
        public int gainSlot = 0;

        @AutoLogOutput(key = "Superstruture/Turret/Feedforward")
        public double feedForward = 0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void applyOutputs(TurretIOOutputs outputs) {}

    public default void seedTurretPosition(Angle position) {}

    public default void refreshEncoderMagnetSensorConfigurations() {}

    public default void reconfigureEncoderMagnetOffsets(double offset17t, double offset18t) {}
}
