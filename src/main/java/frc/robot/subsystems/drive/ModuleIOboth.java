package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ModuleIOboth {
  public class TeleopDrive extends Command {
    private final Drive drive;
    private final DoubleSupplier xSup;
    private final DoubleSupplier ySup;
    private final DoubleSupplier rotSup;
    private final BooleanSupplier robotRelativeSup;

    public TeleopDrive(
        Drive drive,
        DoubleSupplier xSup,
        DoubleSupplier ySup,
        DoubleSupplier rotSup,
        BooleanSupplier robotRelativeSup) {
      this.drive = drive;
      this.xSup = xSup;
      this.ySup = ySup;
      this.rotSup = rotSup;
      this.robotRelativeSup = robotRelativeSup;
      addRequirements(drive);
    }

    @Override
    public void execute() {
      double x = MathUtil.applyDeadband(xSup.getAsDouble(), 0.05);
      double y = MathUtil.applyDeadband(ySup.getAsDouble(), 0.05);
      double rot = MathUtil.applyDeadband(rotSup.getAsDouble(), 0.05);

      x *= drive.getMaxLinearSpeedMetersPerSec();
      y *= drive.getMaxLinearSpeedMetersPerSec();
      rot *= drive.getMaxAngularSpeedRadPerSec();

      ChassisSpeeds speeds =
          robotRelativeSup.getAsBoolean()
              ? new ChassisSpeeds(x, y, rot)
              : ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, drive.getRotation());

      drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
      drive.stop();
    }
  }
}
