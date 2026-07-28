package com.stuypulse.robot.subsystems.superstructure.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private static final Turret instance; 

    static {
        switch (Settings.currentMode) {
            case REAL -> instance = new Turret(new TurretIOTalonFX());

            case SIM -> instance = new Turret(new TurretIOTalonFX());

        
        }
    }
}
