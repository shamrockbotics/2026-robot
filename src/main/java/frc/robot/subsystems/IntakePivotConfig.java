package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class IntakePivotConfig extends MechanismConfig {
  public IntakePivotConfig() {
    this(true);
  }

  public IntakePivotConfig(boolean real) {
    name = "Intake Config";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -100;
    maxPosition = 100;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io = new MechanismIOTalonFX(17, 9, 0.0, true, false, 2 * Math.PI, 40, 12.0, 1.7, 0.0);

    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
