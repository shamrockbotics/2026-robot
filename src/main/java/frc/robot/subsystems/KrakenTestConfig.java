package frc.robot.subsystems;

import frc.robot.subsystems.roller.RollerConfig;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;

public class KrakenTestConfig extends RollerConfig {
  public KrakenTestConfig() {
    this(true);
  }

  public KrakenTestConfig(boolean real) {
    name = "Kraken Test";
    intakePercent = 0.2;
    releasePercent = 0.6;
    if (real) {
      io = new RollerIOTalonFX(10, false, 12, 0.5, 0.0);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}