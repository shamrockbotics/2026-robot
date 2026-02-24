package frc.robot.subsystems.mechanism;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.Mechanism.MotionType;

public abstract class MechanismConfig {
  public String name = "Arm";
  public MotionType motionType = MotionType.ANGULAR;
  public double minPosition = Units.degreesToRadians(-90);
  public double maxPosition = Units.degreesToRadians(90);
  public double allowedError = Units.degreesToRadians(2);
  public MechanismIO io;
}
