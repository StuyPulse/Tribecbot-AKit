/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.RGBWColor;

import com.stuypulse.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOSim implements LEDIO {
    // this should appear in the WPILib simulation menu.
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LEDIOSim() {
        this.led = new AddressableLED(0); // dummy port
        this.buffer = new AddressableLEDBuffer(LEDSettings.LED_LENGTH);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) { // simulation is always healthy
        inputs.isConnected = true;
        inputs.supplyVoltage = Volts.of(12.0);
        inputs.fiveVRailVoltage = Volts.of(5.0);
        inputs.outputCurrentAmps = Amps.of(0.5);
        inputs.LEDTemperature = Celsius.of(25.0);
        inputs.hardwareFault = false;
        inputs.underVoltageFault = false;
    }

    @Override
    public void applyOutputs(LEDIOOutputs outputs) {
        for (LEDPattern pattern : outputs.patterns) {
            RGBWColor color = pattern.color();

            int start = Math.max(pattern.start(), 0);
            int end = Math.min(pattern.end(), buffer.getLength() - 1);

            for (int i = start; i <= end; i++) {
                buffer.setRGB(i, color.Red, color.Green, color.Blue);
            }
        }
    }

    @Override
    public void periodic() {
        // For the future if LED animations are added. This is only necessary in sim because CANdle
        // has
        // native animation support already, while the WPILib AddressableLED class does not.
    }
}
