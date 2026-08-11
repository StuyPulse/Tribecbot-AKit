package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

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
        leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.CANIVORE);
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
        leds.setControl(outputs.pattern);
        if (outputs.leftLimelightDead) {
            leds.setControl(LEDConstants.LEFT_DEAD_STRIP.withColor(LEDConstants.LLDEAD));
        }
        if (outputs.rightLimelightDead) {
            leds.setControl(LEDConstants.RIGHT_DEAD_STRIP.withColor(LEDConstants.LLDEAD));
        }
        if (outputs.backLimelightDead) {
            leds.setControl(LEDConstants.BACK_DEAD_STRIP.withColor(LEDConstants.LLDEAD));
        }
    }

    @Override
    public void clearAllAnimations() {
        leds.clearAllAnimations();
    }
}
