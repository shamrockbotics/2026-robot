package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class JellybeanArmConfig extends MechanismConfig {
  public JellybeanArmConfig() {
    this(true);
  }

  public JellybeanArmConfig(boolean real) {
    name = "Jellybean Arm";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -100;
    maxPosition = 100;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new MechanismIOSparkMax(
                  15, 0.0, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 12.0, 1.7, 0.0)
              .addFollower(13, true);

    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
