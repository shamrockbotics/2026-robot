package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public boolean connected = false;
    public double velocityMetersPerSec = 0.0;
    public boolean hasObject = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public double maxVoltage = 12.0;

  /** Update the set of loggable inputs. */
  public default void updateInputs(RollerIOInputs inputs) {}

  /** Run the intake at the specified velocity. */
  public default void setVelocity(double metersPerSec) {}

  /** Run open loop at the specified output in the range [-1, 1]. */
  public default void setOutput(double output) {}
}
