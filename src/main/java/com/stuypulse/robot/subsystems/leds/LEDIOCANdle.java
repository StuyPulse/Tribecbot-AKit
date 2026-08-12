package com.stuypulse.robot.subsystems.leds;

import edu.wpi.first.units.measure.*;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.leds.LEDConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LEDIOCANdle implements LEDIO {
    private final CANdle leds;

    private StatusSignal<Voltage> supplyVoltage;
    private StatusSignal<Voltage> fiveVRailVoltage;
    private StatusSignal<Current> outputCurrentAmps;
    private StatusSignal<Temperature> LEDTemperature;
    private StatusSignal<Boolean> hardwareFault;
    private StatusSignal<Boolean> underVoltageFault;

    private CANdleConfiguration candleConfigs;

    public LEDIOCANdle() {
        leds = new CANdle(LEDIds.CANDLE_PORT, GlobalSettings.CANIVORE);
        candleConfigs = new CANdleConfiguration()
                .withLED(
                        new LEDConfigs()
                                .withBrightnessScalar(1.0)
                                .withStripType(StripTypeValue.GRB)
                                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning))
                .withCANdleFeatures(
                        new CANdleFeaturesConfigs()
                                .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled));

        leds.getConfigurator().apply(candleConfigs);

        this.supplyVoltage = leds.getSupplyVoltage();
        this.fiveVRailVoltage = leds.getFiveVRailVoltage();
        this.outputCurrentAmps = leds.getOutputCurrent();
        this.LEDTemperature = leds.getDeviceTemp();
        this.hardwareFault = leds.getFault_Hardware();
        this.underVoltageFault = leds.getFault_Undervoltage();

        leds.getSupplyVoltage().setUpdateFrequency(10);
        leds.getFiveVRailVoltage().setUpdateFrequency(10);
        leds.getOutputCurrent().setUpdateFrequency(10);
        leds.getDeviceTemp().setUpdateFrequency(10);
        leds.getFault_Hardware().setUpdateFrequency(4);
        leds.getFault_Undervoltage().setUpdateFrequency(4);
        leds.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            supplyVoltage,
            fiveVRailVoltage,
            outputCurrentAmps,
            LEDTemperature,
            hardwareFault,
            underVoltageFault);

        inputs.isConnected = leds.isConnected();
        inputs.supplyVoltage = supplyVoltage.getValue();
        inputs.fiveVRailVoltage = fiveVRailVoltage.getValue();
        inputs.outputCurrentAmps = outputCurrentAmps.getValue();
        inputs.LEDTemperature = LEDTemperature.getValue();
        inputs.hardwareFault = hardwareFault.getValue();
        inputs.underVoltageFault = underVoltageFault.getValue();
    }

    @Override
    public void applyOutputs(LEDIOOutputs outputs) {
        for (LEDPattern pattern : outputs.patterns) {
            SolidColor request = new SolidColor(pattern.start(), pattern.end()).withColor(pattern.color());
            leds.setControl(request);
        }
    }
}
