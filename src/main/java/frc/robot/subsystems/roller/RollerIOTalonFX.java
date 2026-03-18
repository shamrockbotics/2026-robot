package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
  ;

  public RollerIOTalonFX(
      int id1,
      int id2,
      boolean motorInverted,
      double voltageLimit,
      double velocityKp,
      double velocityKd) {
    this(id1, motorInverted, voltageLimit, velocityKp, velocityKd);
    TalonFX talon2 = new TalonFX(id2);
    TalonFXConfiguration configs2 = new TalonFXConfiguration();
    configs2.Slot0.kP = velocityKp;
    configs2.Slot0.kD = velocityKd;
    configs2.Voltage.withPeakForwardVoltage(voltageLimit);
    configs2.Voltage.withPeakReverseVoltage(-voltageLimit);
    talon2.getConfigurator().apply(configs2);

    talon2.setControl(new Follower(id1, MotorAlignmentValue.Opposed));
  }

  public RollerIOTalonFX(
      int id1, boolean motorInverted, double voltageLimit, double velocityKp, double velocityKd) {
    talon = new TalonFX(id1);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = velocityKp;
    configs.Slot0.kD = velocityKd;
    configs.Voltage.withPeakForwardVoltage(voltageLimit);
    configs.Voltage.withPeakReverseVoltage(-voltageLimit);
    talon.getConfigurator().apply(configs);
    talon.setPosition(0);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocity = talon.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rotationsPerSecond) {
    double rotationsPerMinute = rotationsPerSecond / 60;
    talon.setControl(velocityVoltage.withVelocity(rotationsPerMinute));
  }

  @Override
  public void setOutput(double value) {
    talon.setVoltage(value * maxVoltage);
  }
}
