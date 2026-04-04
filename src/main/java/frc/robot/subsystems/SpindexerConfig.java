package frc.robot.subsystems;

import frc.robot.subsystems.roller.RollerConfig;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOSparkFlex;

public class SpindexerConfig extends RollerConfig {
  public SpindexerConfig() {
    this(true);
  }

  public SpindexerConfig(boolean real) {
    name = "Spindexer";
    intakePercent = 0.05;
    releasePercent = -0.1;
    if (real) {
      io = new RollerIOSparkFlex(16, false, false, 0.25 / 21, 0.25 / 21, 40, 0.002, 0.0);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}
