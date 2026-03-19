package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanism.Mechanism;

public class ClimbCommands {
  private Mechanism climberArm;

  private final double armStowPosition = 0.0;
  private final double armClimbPosition = 1.5;

  public ClimbCommands(Mechanism climberArm) {
    this.climberArm = climberArm;
  }

  public Command stow() {
    return Commands.parallel(climberArm.runToPositionCommand(armStowPosition).withName("Stow"));
  }

  public Command Climb() {
    return Commands.parallel(
        climberArm.runToPositionCommand(armClimbPosition).withName("Climb Tower"));
  }
}
