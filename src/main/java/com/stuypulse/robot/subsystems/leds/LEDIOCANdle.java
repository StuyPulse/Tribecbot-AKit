package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
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
  public boolean isConnected = false;
  
  public StatusSignal<Voltage> supplyVoltage;
  public StatusSignal<Voltage> fiveVRailVoltage;
  public StatusSignal<Current> outputCurrentAmps;
  public StatusSignal<Temperature> LEDTemperature;
  public boolean hardwareFault = false;
  public boolean underVoltageFault = false;

  private CANdleConfiguration candleConfigs;
  private ControlRequest ledPattern =
      LEDConstants.LED.solidColorRequest.withColor(LEDConstants.LED.DISABLED);

  public LEDIOCANdle() {

    leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.CANIVORE);
    candleConfigs =
        new CANdleConfiguration()
            .withLED(
                new LEDConfigs()
                    .withBrightnessScalar(1.0)
                    .withStripType(StripTypeValue.GRB)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning))
            .withCANdleFeatures(
                new CANdleFeaturesConfigs()
                    .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled));

    leds.getConfigurator().apply(candleConfigs);

    leds.setControl(ledPattern);

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
    inputs.isConnected = leds.isConnected();
    inputs.supplyVoltage = leds.getSupplyVoltage().getValue();
    inputs.fiveVRailVoltage = leds.getFiveVRailVoltage().getValue();
    inputs.outputCurrentAmps = leds.getOutputCurrent().getValue();
    inputs.LEDTemperature = leds.getDeviceTemp().getValue();
    inputs.hardwareFault = leds.getFault_Hardware().getValue();
    inputs.underVoltageFault = leds.getFault_Undervoltage().getValue();
  }

  @Override
  public void setControl(ControlRequest request) {
    leds.setControl(request);
  }

  @Override
  public void clearAllAnimations() {
      leds.clearAllAnimations();
    }
}
