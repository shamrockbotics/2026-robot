package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.roller.Roller;
import java.util.function.DoubleSupplier;

public class PositionBasedAction {
  public static Command shoot(
      DoubleSupplier shootVelocity, DoubleSupplier shuttleVelocity, Drive drive, Roller shooter) {
    if (isInShuttleRange(drive)) {
      return shooter.runAtVelocityCommand(shuttleVelocity);
    } else {
      return shooter.runAtVelocityCommand(shootVelocity);
    }
  }

  public static boolean isInShuttleRange(Drive drive) {
    return drive.getPose().getX() <= Units.inchesToMeters(469.11)
        && drive.getPose().getX() >= Units.inchesToMeters(182.11);
  }
}
