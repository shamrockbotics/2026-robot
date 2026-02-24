package frc.robot.subsystems.mechanism;

import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface MechanismIO {
  @AutoLog
  public static class MechanismIOInputs {
    public boolean connected = false;
    public double targetPosition = 0.0;
    public double currentPosition = 0.0;
    public boolean onTarget = false;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double appliedOutput = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(MechanismIOInputs inputs) {}

  /** Run the mechanism to the specified position. */
  public default void setPosition(double position) {}

  public default void setPositionOffset(DoubleSupplier supplier) {}

  /** Run open loop at the specified output in the range [-1, 1]. */
  public default void setOutput(double output) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage voltage) {}
}
