package frc.robot.subsystems;

import frc.robot.subsystems.roller.RollerConfig;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOSparkFlex;

public class JellybeanIntakeConfig extends RollerConfig {
  public JellybeanIntakeConfig() {
    this(true);
  }

  public JellybeanIntakeConfig(boolean real) {
    name = "Jellybean Intake";
    intakePercent = 0.2;
    releasePercent = 0.6;
    if (real) {
      io = new RollerIOSparkFlex(10, 9, true, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5, 0.0);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}
