package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
  private final VoltageOut voltageController = new VoltageOut(0.0);

  public RollerIOTalonFX(
      int id1,
      int id2,
      boolean motorInverted,
      double voltageLimit,
      double velocityKp,
      double velocityKd,
      double velocityKs,
      double velocityKv,
      double velocityKa) {
    this(
        id1,
        motorInverted,
        voltageLimit,
        velocityKp,
        velocityKd,
        velocityKs,
        velocityKv,
        velocityKa);
    TalonFX talon2 = new TalonFX(id2);
    talon2.setControl(new Follower(id1, MotorAlignmentValue.Opposed));
  }

  public RollerIOTalonFX(
      int id1,
      boolean motorInverted,
      double voltageLimit,
      double velocityKp,
      double velocityKd,
      double velocityKs,
      double velocityKv,
      double velocityKa) {
    talon = new TalonFX(id1);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted =
        motorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = velocityKp;
    config.Slot0.kD = velocityKd;
    config.Slot0.kS = velocityKs;
    config.Slot0.kV = velocityKv;
    config.Slot0.kA = velocityKa;
    config.Voltage.withPeakForwardVoltage(voltageLimit);
    config.Voltage.withPeakReverseVoltage(-voltageLimit);
    talon.getConfigurator().apply(config);
    talon.setPosition(0);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocity = talon.getVelocity().getValueAsDouble() * 60;
    inputs.velocityPerSecond = talon.getVelocity().getValueAsDouble();
    inputs.position = talon.getPosition().getValueAsDouble();
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = talon.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rotationsPerMinute) {
    talon.setControl(velocityVoltage.withVelocity(rotationsPerMinute / 60));
  }

  @Override
  public void setOutput(double value) {
    talon.setVoltage(value * maxVoltage);
  }

  public void setVoltage(double voltage) {
    talon.setControl(voltageController.withOutput(voltage));
  }
}
