package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.SimCanIds;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.*;

public final class IntakeHardware<T extends TalonFX> {
    public final T pivotMotor;
    public final T rollerLeaderMotor;
    public final T rollerFollowerMotor;

    // null on real robot gangalang
    public final SystemSim<SingleJointedArmSim> pivotSim;
    public final SystemSim<DCMotorSim> rollerSim;

    private IntakeHardware(
        T pivot, 
        T rollerLeader, 
        T rollerFollower, 
        SystemSim<SingleJointedArmSim> pivotSim, 
        SystemSim<DCMotorSim> rollerSim
    ) {
            this.pivotMotor = pivot;
            this.rollerLeaderMotor = rollerLeader;
            this.rollerFollowerMotor = rollerFollower;
            this.pivotSim = pivotSim;
            this.rollerSim = rollerSim;
        }

    public static IntakeHardware<TalonFX> createReal() {
        final TalonFX pivot = new TalonFX(Ports.Intake.PIVOT, Ports.RIO);
        final TalonFX rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER, Ports.RIO);
        final TalonFX rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER, Ports.RIO);
        return new IntakeHardware<TalonFX>(pivot, rollerLeader, rollerFollower, null, null);
    }

    public static IntakeHardware<TalonFXSimulation> createSim() {
        SystemSim<SingleJointedArmSim> pivotSim = SystemSim.of(
            new SingleJointedArmSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    Settings.Intake.PIVOT_MOI,
                    Settings.Intake.PIVOT_GEAR_RATIO
                ),
            DCMotor.getKrakenX60(1),
            Settings.Intake.PIVOT_GEAR_RATIO,
            Settings.Intake.ARM_LENGTH_METERS,
            Settings.Intake.PIVOT_MIN_ANGLE.in(Radians),
            Settings.Intake.PIVOT_MAX_ANGLE.in(Radians),
            true,
            Settings.Intake.PIVOT_MAX_ANGLE.in(Radians))
        );

        SystemSim<DCMotorSim> rollerSim = SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(2),
                    0.01, // arbitrary
                    1.0
                ),
                DCMotor.getKrakenX60(2))
        );

        final TalonFXSimulation pivot = new TalonFXSimulation(SimCanIds.get("Intake/Pivot"), Settings.Intake.PIVOT_GEAR_RATIO, pivotSim);
        final TalonFXSimulation rollerLeader = new TalonFXSimulation(SimCanIds.get("Intake/RollerLeader"), 1.0, rollerSim);
        final TalonFXSimulation rollerFollower = new TalonFXSimulation(SimCanIds.get("Intake/RollerFollower"), 1.0, rollerSim);
        return new IntakeHardware<TalonFXSimulation>(pivot, rollerLeader, rollerFollower, pivotSim, rollerSim);
    }
}
