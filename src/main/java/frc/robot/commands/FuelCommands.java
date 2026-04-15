package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.roller.Roller;

public class FuelCommands {
  private Roller shooterTransfer;
  private Roller shooterRoller;
  private Roller spindexer;
  private Roller intakeRoller;
  private Mechanism intakePivot;

  public FuelCommands(
      Roller shooterTransfer,
      Roller shooterRoller,
      Roller spindexer,
      Roller intakeRoller,
      Mechanism intakePivot) {
    this.shooterTransfer = shooterTransfer;
    this.shooterRoller = shooterRoller;
    this.spindexer = spindexer;
    this.intakeRoller = intakeRoller;
    this.intakePivot = intakePivot;
  }

  public Command release(double velocity) {
    return Commands.parallel(
        shooterRoller.runAtVelocityCommand(() -> velocity),
        shooterTransfer.intakeCommand(),
        spindexer.intakeCommand());
  }

  public Command intake(Command command) {
    return intakeRoller.intakeCommand();
  }

  public Command rollOut() {
    return Commands.run(
        () -> {
          intakePivot.run(0.1);
        });
  }
}
