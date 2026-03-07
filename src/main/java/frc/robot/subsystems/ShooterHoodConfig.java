package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class ShooterHoodConfig extends MechanismConfig {
  public ShooterHoodConfig() {
    this(true);
  }

  public ShooterHoodConfig(boolean real) {
    name = "Shooter Hood";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -100;
    maxPosition = 100;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new MechanismIOSparkMax(
              14, 0.0, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 12.0, 1.7, 0.0);

    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
