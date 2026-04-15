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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
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
  private final Roller shooterRoller;
  private final Vision vision;
  private final Roller intakeRoller;
  private final Roller shooterTransfer;
  private final Roller spindexer;
  private final Mechanism intakePivot;
  private final FuelCommands feulCommands;
  private final InterpolatingDoubleTreeMap shooterInterpolatingTreeMap;
  private final InterpolatingDoubleTreeMap shuttleInterpolatingTreeMap;
  // private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Adjustable offset to correct which way the robot considers its "front" when aiming at the hub.
  // Set to 180 degrees by default to flip the heading. Change this value to tune without
  // modifying the rest of the targeting code.
  private static final Rotation2d HUB_FACING_OFFSET = Rotation2d.fromDegrees(180.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkNumber shooterVelocity;
  private final LoggedNetworkNumber idleShooterVelocity;
  private final LoggedNetworkBoolean autoShootEnabled;

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
        shooterRoller = new Roller(new ShooterRollerConfig());
        intakeRoller = new Roller(new IntakeRollerConfig());
        shooterTransfer = new Roller(new ShooterTransferConfig());
        spindexer = new Roller(new SpindexerConfig());
        intakePivot = new Mechanism(new IntakePivotConfig());
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));

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
        shooterRoller = new Roller(new ShooterRollerConfig(false));
        intakeRoller = new Roller(new IntakeRollerConfig(false));
        shooterTransfer = new Roller(new ShooterTransferConfig(false));
        spindexer = new Roller(new SpindexerConfig(false));
        intakePivot = new Mechanism(new IntakePivotConfig(false));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));

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
        shooterRoller = new Roller(new ShooterRollerConfig() {});
        intakeRoller = new Roller(new IntakeRollerConfig() {});
        shooterTransfer = new Roller(new ShooterTransferConfig() {});
        spindexer = new Roller(new SpindexerConfig() {});
        intakePivot = new Mechanism(new IntakePivotConfig() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        break;
    }
    shooterInterpolatingTreeMap = new InterpolatingDoubleTreeMap();
    shooterInterpolatingTreeMap.put(3.9628, (double) 1400);
    shooterInterpolatingTreeMap.put(3.66, (double) 1300);
    shooterInterpolatingTreeMap.put(2.17, (double) 1125);
    shuttleInterpolatingTreeMap = new InterpolatingDoubleTreeMap();
    shuttleInterpolatingTreeMap.put(7.64, (double) 1400);
    shuttleInterpolatingTreeMap.put(6.57, (double) 1300);
    shuttleInterpolatingTreeMap.put(3.05, (double) 1125);

    shooterVelocity = new LoggedNetworkNumber("/Tuning/ShooterVelocity", 1125);
    autoShootEnabled = new LoggedNetworkBoolean("/Tuning/AutoShootEnabled", true);
    feulCommands =
        new FuelCommands(shooterTransfer, shooterRoller, spindexer, intakeRoller, intakePivot);
    NamedCommands.registerCommand("Release", feulCommands.release(() -> getTargetVelocity()));
    NamedCommands.registerCommand("Intake", feulCommands.intake(intakeRoller.getDefaultCommand()));
    NamedCommands.registerCommand("Rollout", intakePivot.runToPositionCommand(0));
    NamedCommands.registerCommand("RollBack", intakePivot.runToPositionCommand(1.556));
    NamedCommands.registerCommand("Debug 1", Commands.run(() -> System.out.println("Debug 1")));
    NamedCommands.registerCommand("Debug 2", Commands.run(() -> System.out.println("Debug 2")));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    idleShooterVelocity = new LoggedNetworkNumber("/Tuning/IdleShooterVelocity", 500);
    autoChooser.addOption(
        "Shooter SysID (Quasistatic Forward)",
        shooterRoller.sysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysID (Quasistatic Reverse)",
        shooterRoller.sysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysID (Dynamic Forward)",
        shooterRoller.sysIdDynamicCommand(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysID (Dynamic Reverse)",
        shooterRoller.sysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
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
        "Shooter SysID (Quasistatic Forward)",
        shooterRoller.sysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysID (Quasistatic Reverse)",
        shooterRoller.sysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysID (Dynamic Forward)",
        shooterRoller.sysIdDynamicCommand(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysID (Dynamic Reverse)",
        shooterRoller.sysIdDynamicCommand(SysIdRoutine.Direction.kReverse));
    intakePivot.addSysIdCommands(autoChooser);

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

    shooterRoller.setDefaultCommand(shooterRoller.runAtVelocityCommand(() -> 15 * 60));

    // Reset gyro to 0° when B button is pressed
    controller.y().onTrue(DriveCommands.resetPoseForward(drive).ignoringDisable(true));

    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               spindexer.run(.1);
    //             }));
    operatorController
        .rightTrigger()
        .and(
            () ->
                Math.abs(shooterRoller.getVelocity().getAsDouble() - getTargetVelocity("hub"))
                    <= 50.0)
        .whileTrue(spindexer.intakeCommand())
        .whileTrue(shooterTransfer.intakeCommand());
    operatorController
        .x()
        .whileTrue(
            PositionBasedAction.shoot(
                () -> getTargetVelocity("hub"),
                () -> getTargetVelocity("shuttle"),
                drive,
                shooterRoller)); // shooterRoller.runAtVelocityCommand(() ->
    // getTargetVelocity("hub")));
    controller
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> {
                  var t = getRobotToHubTranslation();
                  var rot = new Rotation2d(t.getX(), t.getY());
                  return rot.plus(HUB_FACING_OFFSET);
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
    operatorController.leftTrigger().whileTrue(intakePivot.runToPositionCommand(0.0));
    operatorController.a().whileTrue(intakePivot.runToPositionCommand(0.0));
    operatorController.y().whileTrue(intakePivot.runToPositionCommand(1.556));
    operatorController
        .b()
        .whileTrue(
            Commands.run(
                () ->
                    System.out.println(
                        "x: "
                            + getRobotToHubTranslation().getX()
                            + " y: "
                            + getRobotToHubTranslation().getY()
                            + " distance to hub: "
                            + Math.hypot(
                                getRobotToHubTranslation().getX(),
                                getRobotToHubTranslation().getY()))));
    operatorController.rightBumper().whileTrue(spindexer.releaseCommand());
    // controller.y().whileTrue(shooterRoller.intakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public double getTargetVelocity(String target) {
    if (autoShootEnabled.getAsBoolean() == false) {
      return shooterVelocity.getAsDouble();
    }
    switch (target) {
      case "hub":
        Translation2d robotToHub = getRobotToHubTranslation();
        double distance = Math.hypot(robotToHub.getX(), robotToHub.getY());
        return (shooterInterpolatingTreeMap.get(distance));
      case "shuttle":
        double robotToShuttleSetpoint = getRobotToShuttleSetpointDistance();
        return shuttleInterpolatingTreeMap.get(robotToShuttleSetpoint);
      default:
        return 0.0;
    }
  }

  public Translation2d getRobotToHubTranslation() {
    if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
      return new Translation2d(
          Units.inchesToMeters(469.11) - drive.getPose().getX(),
          Units.inchesToMeters(158.84) - drive.getPose().getY());
    } else if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      return new Translation2d(
          Units.inchesToMeters(182.11) - drive.getPose().getX(),
          Units.inchesToMeters(158.84) - drive.getPose().getY());
    } else {
      System.out.println("Team not found shooter back to custom value");
      return new Translation2d(0, 0);
    }
  }

  public double getRobotToShuttleSetpointDistance() {
    if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
      return Math.abs(Units.inchesToMeters(569.11) - drive.getPose().getX());
    } else if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      return Math.abs(Units.inchesToMeters(82.11) - drive.getPose().getX());
    } else {
      System.out.println("Team not found shuttle back to custom value");
      return 0.0;
    }
  }

  public double getAngle() {
    Translation2d robotToHub = getRobotToHubTranslation();
    double x_error = robotToHub.getX();
    double y_error = robotToHub.getY();
    return Math.atan(y_error / x_error);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
