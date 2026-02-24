// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.JellybeanArmConfig;
import frc.robot.subsystems.JellybeanInsideConfig;
import frc.robot.subsystems.JellybeanIntakeConfig;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkCANCoder;
import frc.robot.subsystems.mechanism.*;
import frc.robot.subsystems.roller.*;
import frc.robot.subsystems.vision.*;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Mechanism jellybeanArm;
  private final Roller jellybeanIntake;
  private final Roller jellybeanInside;
  private final Vision vision;
  private final Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOTalonFX(TunerConstants.FrontLeft),
        //         new ModuleIOTalonFX(TunerConstants.FrontRight),
        //         new ModuleIOTalonFX(TunerConstants.BackLeft),
        //         new ModuleIOTalonFX(TunerConstants.BackRight));
        // CASE SPARK
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOSpark(0),
        //         new ModuleIOSpark(1),
        //         new ModuleIOSpark(2),
        //         new ModuleIOSpark(3));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkCANCoder(0, 4),
                new ModuleIOSparkCANCoder(1, 3),
                new ModuleIOSparkCANCoder(2, 2),
                new ModuleIOSparkCANCoder(3, 1));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        jellybeanArm = new Mechanism(new JellybeanArmConfig());
        jellybeanIntake = new Roller(new JellybeanIntakeConfig());
        jellybeanInside = new Roller(new JellybeanInsideConfig());
        // new VisionIOPhotonVision(camera2Name, robotToCamera2));
        // new VisionIOPhotonVision(camera2Name, robotToCamera2));

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
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        jellybeanArm = new Mechanism(new JellybeanArmConfig(false));
        jellybeanIntake = new Roller(new JellybeanIntakeConfig(false));
        jellybeanInside = new Roller(new JellybeanInsideConfig(false));

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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        jellybeanArm = new Mechanism(new JellybeanArmConfig() {});
        jellybeanIntake = new Roller(new JellybeanIntakeConfig() {});
        jellybeanInside = new Roller(new JellybeanInsideConfig() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
            () -> controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));
    DoubleSupplier manualLeft = () -> -operatorController.getLeftY() / 10;
    operatorController.a().whileTrue(jellybeanArm.runPercentCommand(manualLeft));
    operatorController.b().whileTrue(jellybeanArm.runToPositionCommand(0));

    operatorController.rightTrigger().whileTrue(jellybeanIntake.releaseCommand());
    operatorController.leftTrigger().whileTrue(jellybeanIntake.intakeCommand());
    operatorController.leftBumper().whileTrue(jellybeanInside.releaseCommand());
    operatorController.leftTrigger().whileTrue(jellybeanInside.intakeCommand());

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.defer(
    //             () -> {
    //               // Capture the tag ID ONCE at the moment the button is first pressed
    //               var tagIdOpt = vision.returnNearestTagID(0);
    //               if (tagIdOpt.isEmpty()) return Commands.none();

    //               int tagId = tagIdOpt.getAsInt();
    //               var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
    //               if (tagPoseOpt.isEmpty()) return Commands.none();

    //               // Create a command that continuously drives toward the pose
    //               Pose3d targetPose = tagPoseOpt.get();
    //               return Commands.run(() -> drive.driveToPose(targetPose), drive);
    //             },
    //             Set.of(drive)));
    // controller
    //     .a()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               var tagIdOpt = vision.returnNearestTagID(0);
    //               if (tagIdOpt.isEmpty()) return;

    //               int tagId = tagIdOpt.getAsInt();
    //               var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
    //               System.out.println("Got apriltag positions");
    //               PathPlannerPath jellyBeanPath =
    //                   new PathPlannerPath(
    //                       PathPlannerPath.waypointsFromPoses(
    //                           new Pose2d[] {drive.getPose(), tagPoseOpt.get().toPose2d()}),
    //                       new PathConstraints(
    //                           4.0, 4.0, Units.degreesToRadians(360),
    // Units.degreesToRadians(540)),
    //                       null,
    //                       new GoalEndState(0.0, drive.getPose().getRotation()));
    //               System.out.println("created Path");

    //               AutoBuilder.followPath(jellyBeanPath);
    //               System.out.println("following path");
    //            }));

    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Pose2d currentPose = drive.getPose();

                  // The rotation component in these poses represents the direction of travel
                  Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
                  Pose2d endPos =
                      new Pose2d(
                          currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)),
                          new Rotation2d());

                  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
                  PathPlannerPath path =
                      new PathPlannerPath(
                          waypoints,
                          new PathConstraints(
                              4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                          null, // Ideal starting state can be null for on-the-fly paths
                          new GoalEndState(0.0, currentPose.getRotation()));

                  // Prevent this path from being flipped on the red alliance, since the given
                  // positions are already correct
                  path.preventFlipping = true;

                  AutoBuilder.followPath(path).schedule();
                }));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Apply the latest vision measurement (force-apply whatever it is) when Y is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  var m = vision.getLatestVisionMeasurement();
                  if (m != null) {
                    // Force-apply the vision pose regardless of checks
                    drive.applyVisionMeasurementNow(m.pose, m.timestamp, m.stdDevs, true);
                  }
                },
                drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
