package frc.robot.subsystems;

import frc.robot.subsystems.roller.RollerConfig;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;

public class ShooterRollerConfig extends RollerConfig {
  public ShooterRollerConfig() {
    this(true);
  }

  public ShooterRollerConfig(boolean real) {
    name = "Shooter Roller";
    intakePercent = 0.2;
    releasePercent = 0.2;
    if (real) {
      io = new RollerIOTalonFX(11, 12, true, 12, 0.067721, 0.0, 0.075528, 0.14168, 0.039921);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}
