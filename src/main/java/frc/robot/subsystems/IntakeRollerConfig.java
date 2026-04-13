package frc.robot.subsystems;

import frc.robot.subsystems.roller.RollerConfig;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;

public class IntakeRollerConfig extends RollerConfig {
  public IntakeRollerConfig() {
    this(true);
  }

  public IntakeRollerConfig(boolean real) {
    name = "Intake Roller";
    intakePercent = -0.8; // Might need to be inverted as well
    releasePercent = 0.5;
    if (real) {
      io = new RollerIOTalonFX(13, false, 12, 0.5, 0.0, 0.0, 0, 0);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}
