package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.roller.Roller;

public class FuelCommands {
  private Roller shooterTransfer;
  private Roller shooterRoller;
  private Roller spindexer;

  public FuelCommands(Roller shooterTransfer, Roller shooterRoller, Roller spindexer) {
    this.shooterTransfer = shooterTransfer;
    this.shooterRoller = shooterRoller;
    this.spindexer = spindexer;
  }

  public Command release(double velocity) {
    return Commands.parallel(
        shooterRoller.runAtVelocityCommand(velocity),
        shooterTransfer.intakeCommand(),
        spindexer.intakeCommand());
  }
}
