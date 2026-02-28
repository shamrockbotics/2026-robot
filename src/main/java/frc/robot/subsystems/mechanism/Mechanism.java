package frc.robot.subsystems.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Mechanism extends SubsystemBase {
  private final MechanismIO io;
  private final MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();
  private final MotionType motionType;
  private final SysIdRoutine sysId;

  private final double minPosition;
  private final double maxPosition;
  private final double allowedError;

  private final Alert disconnectedAlert;

  private boolean holding = false;
  private double setpoint = 0.0;
  private double lastRunPosition = 0.0;

  public MechanismLigament2d visualization;
  public DoubleSupplier visualizationAngleOffset = () -> 0.0;
  public DoubleSupplier visualizationLengthOffset = () -> 0.0;
  public boolean visualizationReversed = false;

  public static enum MotionType {
    LINEAR,
    ANGULAR
  }

  public Mechanism(MechanismConfig config) {
    setName(config.name);
    io = config.io;
    motionType = config.motionType;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(getName() + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltage, null, this));

    minPosition = config.minPosition;
    maxPosition = config.maxPosition;
    allowedError = config.allowedError;

    disconnectedAlert = new Alert(getName() + " disconnected.", AlertType.kError);

    visualization = new MechanismLigament2d(getName(), 1, lastRunPosition);

    setDefaultCommand(holdCommand());

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);
    inputs.onTarget = (Math.abs(getPosition() - setpoint) <= allowedError);

    if (!holding || DriverStation.isDisabled()) {
      if (onTarget()) {
        lastRunPosition = setpoint;
      } else {
        lastRunPosition = getPosition();
      }
    }

    updateVisualization();

    disconnectedAlert.set(!inputs.connected);
  }

  private void updateVisualization() {
    if (motionType == MotionType.ANGULAR) {
      double visualizationAngle =
          Units.radiansToDegrees(getPosition()) + visualizationAngleOffset.getAsDouble();
      if (visualizationReversed) visualizationAngle = 180 - visualizationAngle;
      visualization.setAngle(visualizationAngle);
    } else {
      visualization.setLength(getPosition() + visualizationLengthOffset.getAsDouble());
    }
  }

  /**
   * Runs the mechanism to the desired position.
   *
   * @param position Desired position
   */
  public void runToPosition(double position) {
    if (holding) inputs.onTarget = false;
    setpoint = position;
    holding = false;
    io.setPosition(MathUtil.clamp(position, minPosition, maxPosition));
  }

  /**
   * Runs the motor at the desired percent.
   *
   * @param value Output value, -1 to +1
   */
  public void run(double value) {
    holding = false;
    if (value < 0.0 && getPosition() <= minPosition) {
      io.setOutput(0.0);
    } else if (value > 0.0 && getPosition() >= maxPosition) {
      io.setOutput(0.0);
    } else {
      io.setOutput(value);
    }
  }

  /** Disables all outputs to motors. */
  public void stop() {
    holding = false;
    io.setOutput(0.0);
  }

  /** Holds the mechanism at the last run position. */
  public void hold() {
    holding = true;
    io.setPosition(lastRunPosition);
  }

  /** Returns the current position of the mechanism. */
  public double getPosition() {
    return inputs.currentPosition;
  }

  public void setPositionOffset(DoubleSupplier supplier) {
    io.setPositionOffset(supplier);
  }

  public double getPositionPercent() {
    return ((inputs.currentPosition - minPosition) / (maxPosition - minPosition));
  }

  /** Returns true if the position is within the allowed error of the target. */
  public boolean onTarget() {
    return inputs.onTarget;
  }

  public Command runToPositionCommand(double position) {
    return run(() -> runToPosition(position)).withName("Run To Position " + position);
  }

  public Command holdCommand() {
    return run(() -> hold()).withName("Hold");
  }

  public Command runPercentCommand(DoubleSupplier valueSupplier) {
    return run(() -> run(valueSupplier.getAsDouble())).withName("Run Percent");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void addSysIdCommands(LoggedDashboardChooser<Command> chooser) {
    chooser.addOption(
        getName() + " SysId (Quasistatic Forward)",
        sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        getName() + " SysId (Quasistatic Reverse)",
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    chooser.addOption(
        getName() + " SysId (Dynamic Forward)", sysIdDynamic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        getName() + " SysId (Dynamic Reverse)", sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
