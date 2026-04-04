package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public boolean connected = false;
    public double velocity = 0.0;
    public double position = 0.0;
    public double velocityPerSecond = 0.0;
    public boolean hasObject = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public double maxVoltage = 12.0;

  /** Update the set of loggable inputs. */
  public default void updateInputs(RollerIOInputs inputs) {}

  /** Run the intake at the specified velocity. */
  public default void setVelocity(double value) {}

  /** Run open loop at the specified output in the range [-1, 1]. */
  public default void setOutput(double value) {}

  public default void setVoltage(double voltage) {}

  public default Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return null;
  }

  public default Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return null;
  }

  public default DoubleSupplier getVelocity() {
    return () -> 0.0;
  }
}
