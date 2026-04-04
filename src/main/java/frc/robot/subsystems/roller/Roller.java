package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private final double intakePercent;
  private final double releasePercent;

  private final Alert disconnectedAlert;
  public Command runAtVelocity;
  private SysIdRoutine sysIdRoutine;

  public Roller(RollerConfig config) {
    setName(config.name);
    io = config.io;

    this.intakePercent = config.intakePercent;
    this.releasePercent = config.releasePercent;

    disconnectedAlert = new Alert(getName() + " disconnected.", AlertType.kError);

    setDefaultCommand(run(() -> stop()).withName("Stop"));

    SmartDashboard.putData(this);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);

    disconnectedAlert.set(!inputs.connected);
  }

  /**
   * Runs the intake to the desired velocity.
   *
   * @param value target velocity
   */
  public void runAtVelocity(double value) {
    io.setVelocity(value);
  }

  /**
   * Runs the intake motor at the desired percent.
   *
   * @param value Output value, -1 to +1, + output intakes
   */
  public void run(double value) {
    if (value > 0.0 && hasObject()) {
      io.setOutput(0.0);
    } else {
      io.setOutput(value);
    }
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setOutput(0.0);
  }

  /** Returns the current angle of the arm in radians. */
  public double getSpeedMetersPerSec() {
    return inputs.velocity;
  }

  /** Returns true if the arm angle is within the allowed error of the target angle. */
  public boolean hasObject() {
    return inputs.hasObject;
  }

  public Command runPercentCommand(DoubleSupplier valueSupplier) {
    double value = valueSupplier.getAsDouble();
    return run(() -> run(value)).withName("Run Percent " + value);
  }

  public Command stopCommand() {
    return run(() -> stop()).withName("Stop");
  }

  public Command intakeCommand() {
    return run(() -> run(intakePercent)).withName("Intake");
  }

  public Command runAtVelocityCommand(DoubleSupplier velocity) {
    return run(() -> runAtVelocity(velocity.getAsDouble())).withName("Release");
  }

  public Command releaseCommand() {
    return run(() -> run(releasePercent)).withName("Release");
  }

  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public DoubleSupplier getVelocity() {
    return io.getVelocity();
  }
}
