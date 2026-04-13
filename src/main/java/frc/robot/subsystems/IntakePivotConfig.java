package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class IntakePivotConfig extends MechanismConfig {
  public IntakePivotConfig() {
    this(true);
  }

  public IntakePivotConfig(boolean real) {
    name = "Intake Pivot";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -0.1;
    maxPosition = 1.8;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io = new MechanismIOTalonFX(17, 9, 1.662, false, false, 2 * Math.PI, 40, 12.0, 3, 0.0);

    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
