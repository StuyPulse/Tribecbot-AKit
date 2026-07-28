package com.stuypulse.robot.subsystems.superstructure.turret;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;


public interface TurretIO {
    @AutoLog
    class TurretIOInputs{
        public Current turretMotorSupplyCurrent = Amps.zero();
        public Current turretMotorStatorCurrent = Amps.zero();
        public Temperature turrentMotorTemperature = Celsius.zero();
        public Angle turrentMotorPosition = Degrees.zero();
        public Voltage turrentMotorAppliedVoltage = Volts.zero();
        public AngularVelocity turretMotorVelocity = DegreesPerSecond.zero();
    }

    class TurretIOOutputs {
        public Angle turretPosition = Degrees.zero();
    }

    public default void updateInputs(TurretIOInputs inputs){}

    public default void applyOutputs(TurretIOOutputs outputs) {}

}
