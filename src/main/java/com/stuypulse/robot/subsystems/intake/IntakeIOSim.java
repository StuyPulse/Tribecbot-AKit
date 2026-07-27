package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
    private static final double ARM_LENGTH_METERS = 0.4;
    private static final double ARM_MASS_KG = 2.0;
    private static final double MOI = SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG);

    private final SingleJointedArmSim pivotSim;
    private final DCMotorSim rollerLeaderSim;
    private final DCMotorSim rollerFollowerSim;
    private final PIDController pivotController;
    
    public IntakeIOSim(){
        
        LinearSystem<N2, N1, N2> pivotSystem = LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60(1), 
            MOI,
            Settings.Intake.PIVOT_GEAR_RATIO
            );
        
        pivotSim = 
            new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                Settings.Intake.PIVOT_GEAR_RATIO,
                MOI,
                ARM_LENGTH_METERS,
                Settings.Intake.PIVOT_MIN_ANGLE.in(Radians),
                Settings.Intake.PIVOT_MAX_ANGLE.in(Radians),
                true,
                Settings.Intake.PIVOT_MAX_ANGLE.in(Radians)
        );

        pivotController = 
            new PIDController(
                Gains.Intake.Pivot.kP.get(), 
                Gains.Intake.Pivot.kI.get(), 
                Gains.Intake.Pivot.kD.get());
        
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Gains.Intake.Pivot.kS.get(), Gains.Intake.Pivot.kV.get());
        
        rollerLeaderSim = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
                DCMotor.getKrakenX60(1),MOI);
        
        rollerFollowerSim = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
                DCMotor.getKrakenX60(1));
        
        
    }
    
    
    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.pivotMotorPosition = Degrees.of(pivotSim.getAngleRads());
        inputs.pivotMotorStatorCurrent = Amps.of(pivotSim.getCurrentDrawAmps());
        inputs.pivotMotorSupplyCurrent = Amps.of(pivotSim.getCurrentDrawAmps());
        inputs.pivotMotorVelocity = DegreesPerSecond.of(pivotSim.getVelocityRadPerSec());
        //TO DO: Find a way to get Temperature and Position of the Pivot

        inputs.rollerLeaderMotorAppliedVoltage = Volts.of(rollerLeaderSim.getInputVoltage());
        inputs.rollerLeaderMotorStatorCurrent = Amps.of(rollerLeaderSim.getCurrentDrawAmps());
        inputs.rollerLeaderMotorSupplyCurrent = Amps.of(rollerLeaderSim.getCurrentDrawAmps());
        inputs.rollerLeaderMotorVelocity = DegreesPerSecond.of(rollerLeaderSim.getAngularVelocityRadPerSec());
        inputs.rollerLeaderMotorPosition = Degrees.of(rollerLeaderSim.getAngularPositionRad());
        //TO DO: Find a way to get the Temperature

        inputs.rollerFollowerMotorAppliedVoltage = Volts.of(rollerFollowerSim.getInputVoltage());
        inputs.rollerFollowerMotorStatorCurrent = Amps.of(rollerFollowerSim.getCurrentDrawAmps());
        inputs.rollerFollowerMotorSupplyCurrent = Amps.of(rollerFollowerSim.getCurrentDrawAmps());
        inputs.rollerFollowerMotorVelocity = DegreesPerSecond.of(rollerFollowerSim.getAngularVelocityRadPerSec());
        inputs.rollerFollowerMotorPosition = Degrees.of(rollerFollowerSim.getAngularPositionRad());
        //TO DO: Find a way to get the Temperature
    }
    
    @Override
    public void applyOutputs(IntakeIOOutputs outputs){
        double pivotAngleDegrees = Units.degreesToRadians(37);

        pivotSim.setInputVoltage(pivotController.calculate(pivotAngleDegrees * Gains.Intake.Pivot.kG));
        pivotSim.update(0.02);
        
        rollerLeaderSim.setInputVoltage(12 * outputs.rollerDutyCycle);
        rollerLeaderSim.update(0.02);
        
        rollerFollowerSim.setInputVoltage(-12 * outputs.rollerDutyCycle);
        rollerFollowerSim.update(0.02);
    }
}
