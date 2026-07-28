package com.stuypulse.robot.subsystems.superstructure.turret;

import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Degrees;

import java.nio.file.attribute.PosixFileAttributeView;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Turret extends SubsystemBase {
    private static final Turret instance; 

    static {
        switch (Settings.currentMode) {
            case REAL -> instance = new Turret(new TurretIOTalonFX());

            case SIM -> instance = new Turret(new TurretIOTalonFX());

            default -> instance = new Turret(new TurretIO() {});
        }
    }
    public static Turret getInstance(){
        return instance;
    }

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final TurretIOOutputs outputs;

    private boolean OTM;
    private boolean atTolerance;

    private final Debouncer readyToShootDebouncer;

    public Turret(TurretIO io){
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.outputs = new TurretIOOutputs();

        readyToShootDebouncer = new Debouncer(0.5, DebounceType.kBoth);
        OTM = false;
        atTolerance = false;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public void periodicAfterScheduler(){
        Logger.recordOutput("Superstructure/Turret/Driver Input", null);

        io.applyOutputs(outputs);
    }

    public boolean atTolerance(){
        Angle error = inputs.turretMotorPosition.minus(outputs.position);

        if (Robot.isReal()){
            if (OTM) {
                return error.abs(Degrees) < Settings.Superstructure.SHOOTER_SOTM_TOLERANCE_RPM_HIGH;
                return error.abs(Degrees) < Settings.Superstructure.SHOOTER_SOTM_TOLERANCE_RPM_LOW;
            }else{
                return error.abs(Degrees) < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_HIGH;
                return error.abs(Degrees) < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_LOW;
            }
        }else{
            
        }
    }

    private void runVoltage(Voltage voltage){
        outputs.outputMode = ShooterIOOutputMode.VOLTAGE;
    }

    private void runPosition(Angle position, boolean OTM){
        this.OTM = OTM;

        outputs.outputMode = ShooterIOOutputMode.POSITION;
        outputs.position = position;
    }

    private Command runAnalog(CommandXboxController gamepad){
        return run(() -> runPosition(null, OTM));
    }
}
