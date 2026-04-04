// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FuelCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mechanism.*;
import frc.robot.subsystems.roller.*;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Roller climber;
  private final Roller shooterRoller;
  private final Roller intakeRoller;
  private final Mechanism shooterHood;
  private final Roller shooterTransfer;
  private final Roller spindexer;
  private final Mechanism intakePivot;
  private final FuelCommands feulCommands;
  // private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkNumber shooterVelocity;
  private final LoggedNetworkNumber idleShooterVelocity;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        climber = new Roller(new ClimberConfig());
        shooterRoller = new Roller(new ShooterRollerConfig());
        intakeRoller = new Roller(new IntakeRollerConfig());
        shooterHood = new Mechanism(new ShooterHoodConfig());
        shooterTransfer = new Roller(new ShooterTransferConfig());
        spindexer = new Roller(new SpindexerConfig());
        intakePivot = new Mechanism(new IntakePivotConfig());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        climber = new Roller(new ClimberConfig(false));
        shooterRoller = new Roller(new ShooterRollerConfig(false));
        intakeRoller = new Roller(new IntakeRollerConfig(false));
        shooterHood = new Mechanism(new ShooterHoodConfig(false));
        shooterTransfer = new Roller(new ShooterTransferConfig(false));
        spindexer = new Roller(new SpindexerConfig(false));
        intakePivot = new Mechanism(new IntakePivotConfig(false));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        climber = new Roller(new ClimberConfig() {});
        shooterRoller = new Roller(new ShooterRollerConfig() {});
        intakeRoller = new Roller(new IntakeRollerConfig() {});
        shooterHood = new Mechanism(new ShooterHoodConfig() {});
        shooterTransfer = new Roller(new ShooterTransferConfig() {});
        spindexer = new Roller(new SpindexerConfig() {});
        intakePivot = new Mechanism(new IntakePivotConfig() {});
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }
    shooterVelocity = new LoggedNetworkNumber("/Tuning/ShooterVelocity", 1125
    );
    feulCommands =
        new FuelCommands(shooterTransfer, shooterRoller, spindexer, intakeRoller, intakePivot);
    NamedCommands.registerCommand("Release", feulCommands.release(shooterVelocity.getAsDouble()));
    NamedCommands.registerCommand("Intake", feulCommands.intake());
    NamedCommands.registerCommand("Rollout", feulCommands.rollOut());
    NamedCommands.registerCommand("Debug 1", Commands.run(() -> System.out.println("Debug 1")));
    NamedCommands.registerCommand("Debug 2", Commands.run(() -> System.out.println("Debug 2")));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    idleShooterVelocity = new LoggedNetworkNumber("/Tuning/IdleShooterVelocity", 500);
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "2 cycle shooting auto right side",
        AutoSequences.twoBallAutoRight(
            drive, shooterHood, shooterRoller, shooterTransfer, spindexer, intakeRoller));
    autoChooser.addOption(
        "2 cycle shooting auto left side",
        AutoSequences.twoBallAutoLeft(
            drive, shooterHood, shooterRoller, shooterTransfer, spindexer, intakeRoller));
    autoChooser.addOption("Climb Auto", AutoSequences.climbAuto(drive, climber));
    autoChooser.addOption(
        "Center to shoot to depot to shoot left side",
        AutoSequences.centerToShootToDepotToShootLeft(
            drive, shooterHood, shooterRoller, shooterTransfer, spindexer, intakeRoller));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held

    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    
    shooterRoller.setDefaultCommand(shooterRoller.intakeCommand());

    // Reset gyro to 0° when B button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    // controller.rightBumper().whileTrue(climber.intakeCommand());
    // controller.leftBumper().whileTrue(climber.releaseCommand());
    operatorController.rightBumper().whileTrue(spindexer.releaseCommand());
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               spindexer.run(.1);
    //             }));
    operatorController.rightTrigger().whileTrue(spindexer.intakeCommand());
    operatorController.rightTrigger().whileTrue(shooterTransfer.intakeCommand());
    operatorController
        .x()
        .whileTrue(
            Commands.run(
                () -> {
                  shooterRoller.runAtVelocity(shooterVelocity.getAsDouble());
                }));
    // controller
    //     .leftBumper()
    //         Commands.run(
    //     .whileTrue(
    //             () -> {
    //               shooterRoller.runAtVelocity(shooterVelocity.getAsDouble());
    //             }));
    operatorController.leftBumper().whileTrue(intakeRoller.releaseCommand());
    operatorController.leftTrigger().whileTrue(intakeRoller.intakeCommand());
    operatorController
        .a()
        .whileTrue(
            Commands.run(
                () -> {
                  intakePivot.run(0.1);
                }));
    operatorController
        .y()
        .whileTrue(
            Commands.run(
                () -> {
                  intakePivot.run(-0.1);
                }));
    operatorController
        .b()
        .whileTrue(
            Commands.run(
                () -> {
                  shooterRoller.runAtVelocity(getTargetVelocity());
                }));
    // controller.y().whileTrue(shooterRoller.intakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public double getTargetVelocity() {
    double x_error = 0;
    double y_error = 0;
    // if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
    //   x_error = Math.abs(drive.getPose().getX() - 12);
    //   y_error = Math.abs(drive.getPose().getY() - 4);
    // } else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
    //   x_error = Math.abs(drive.getPose().getX() - 4.5);
    //   y_error = Math.abs(drive.getPose().getY() - 4);
    // } else {
    //   System.out.println("Team not found shooter back to custom value");
    //   return shooterVelocity.getAsDouble();
    // }
    x_error = Math.abs(drive.getPose().getX() - 12);
    y_error = Math.abs(drive.getPose().getY() - 4);
    double distance = Units.metersToInches(Math.sqrt(Math.pow(x_error, 2) + Math.pow(y_error, 2)));
    return (4.77 * distance + 820);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private static class AutoSequences {
    public static Command twoBallAutoRight(
        Drive drive,
        Mechanism shooterHood,
        Roller shooterRoller,
        Roller shooterTransfer,
        Roller spindexer,
        Roller intakeRoller) {
      return Commands.none();
    }

    @SuppressWarnings("unused")
    public static Command BackupShootAuto(
        Drive drive, Roller shooterTransfer, Roller spindexer, Mechanism shooterHood) {
      return Commands.none();
    }

    public static Command twoBallAutoLeft(
        Drive drive,
        Mechanism shooterHood,
        Roller shooterRoller,
        Roller shooterTransfer,
        Roller spindexer,
        Roller intakeRoller) {
      return Commands.none();
    }

    public static Command climbAuto(Drive drive, Roller climber) {
      return Commands.none();
    }

    public static Command centerToShootToDepotToShootLeft(
        Drive drive,
        Mechanism shooterHood,
        Roller shooterRoller,
        Roller shooterTransfer,
        Roller spindexer,
        Roller intakeRoller) {
      return Commands.none();
    }
  }
}
