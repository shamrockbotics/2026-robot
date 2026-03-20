package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.roller.Roller;

public class IntakeCommands {

  private Mechanism Spindexer;
  private Mechanism ShooterHood;
  private Mechanism ShooterTransfer;
  private Roller ShooterRoller;

  public IntakeCommands(
      Mechanism Spindexer, Mechanism ShooterHood, Mechanism ShooterTransfer, Roller ShooterRoller) {

    this.Spindexer = Spindexer;
    this.ShooterHood = ShooterHood;
    this.ShooterTransfer = ShooterTransfer;
    this.ShooterRoller = ShooterRoller;
  }

  public Command intake() {
    return Commands.parallel(
            Spindexer.runToPositionCommand(0.0),
            ShooterHood.runToPositionCommand(0.0),
            ShooterTransfer.runToPositionCommand(0.0),
            ShooterRoller.intakeCommand())
        .withName("Intake");
  }

  public Command release() {
    return Commands.parallel(
            Spindexer.runToPositionCommand(0.0),
            ShooterHood.runToPositionCommand(0.0),
            ShooterTransfer.runToPositionCommand(0.0),
            ShooterRoller.releaseCommand())
        .withName("Release");
  }
}
