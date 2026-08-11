package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;

public class LEDIOSim implements LEDIO {
    // this should appear in the WPILib simulation menu.
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LEDIOSim() {
        this.led = new AddressableLED(0); // dummy port
        this.buffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

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
        if (outputs.pattern instanceof SolidColor) { // should be extendable if animations are added. If so, then there should be a periodic method.
            applySolidColor((SolidColor) outputs.pattern);
        }
    }

    private void applySolidColor(SolidColor colorRequest) {
        int start = Math.max(colorRequest.LEDStartIndex, LEDConstants.STRIP_START);
        int end = Math.min(colorRequest.LEDEndIndex, LEDConstants.STRIP_START + buffer.getLength() - 1);

        Color color = new Color(colorRequest.Color.Red / 255.0, colorRequest.Color.Green / 255.0, colorRequest.Color.Blue / 255.0);

        for (int i = start; i <= end; i++) {
            buffer.setLED(i, color);
        }

        led.setData(buffer);
    }

    @Override
    public void periodic() {
        // For the future if LED animations are added. This is only necessary in sim because CANdle has native animation support already.
    }
}
