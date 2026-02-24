// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);
  // Whether we've used vision to initialize the odometry yet
  private boolean visionPoseInitialized = false;
  // Scale factor applied to vision measurement standard deviations when applying
  // manual corrections (larger = less trust in vision corrections)
  private double visionStdDevScale = 5.0;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. Only used to initialize odometry once. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // Only accept vision on startup: if we've already initialized, ignore
    // subsequent vision updates entirely (per request).
    odometryLock.lock();
    try {
      if (visionPoseInitialized) {
        // Ignore periodic vision updates after initialization
        return;
      }

      // Check reliability of the vision measurement using provided std devs.
      // Matrix layout: [linearX; linearY; angular]
      double linearStd = visionMeasurementStdDevs.get(0, 0);
      double angularStd = visionMeasurementStdDevs.get(2, 0);

      if (linearStd > visionInitMaxLinearStdDevMeters
          || angularStd > visionInitMaxAngularStdDevRadians) {
        // Measurement not reliable enough to initialize odometry
        Logger.recordOutput("Odometry/VisionInitRejected/LinearStd", linearStd);
        Logger.recordOutput("Odometry/VisionInitRejected/AngularStd", angularStd);
        return;
      }

      // Accept and set initial pose
      poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), visionRobotPoseMeters);
      visionPoseInitialized = true;
      Logger.recordOutput("Odometry/VisionInitialized", true);
    } finally {
      odometryLock.unlock();
    }
  }

  /**
   * Manually apply a vision measurement to the odometry. This can be used from a command or a
   * dashboard button to force a vision correction at a controlled time. By default the method
   * enforces the same reliability/proximity checks as automatic initialization; set {@code
   * forceApply} to true to bypass checks.
   *
   * @param visionRobotPoseMeters pose reported by vision
   * @param timestampSeconds timestamp of the measurement (seconds)
   * @param visionMeasurementStdDevs measurement standard deviations (N3x1)
   * @param forceApply if true, bypass proximity/reliability checks and apply
   */
  public void driveToPose(Pose3d tagPose) {
    double offsetDistance = 1.0; // Distance in FRONT of the tag (tunable)

    // Convert tag pose to 2D
    Pose2d tagPose2d = tagPose.toPose2d();
    Pose2d currentPose = getPose();

    // Compute offset in the direction the tag is facing
    double offsetX = offsetDistance * Math.cos(tagPose2d.getRotation().getRadians());
    double offsetY = offsetDistance * Math.sin(tagPose2d.getRotation().getRadians());

    // Target pose = tag pose + forward offset
    Pose2d targetPose2d =
        new Pose2d(tagPose2d.getX() + offsetX, tagPose2d.getY() + offsetY, tagPose2d.getRotation());

    // Control gains
    double kP = 2.0;
    double kPRotation = 4.0;

    System.out.println("Target Pose (in front): " + targetPose2d);
    System.out.println("Current Pose: " + currentPose);

    // Calculate errors
    double xError = targetPose2d.getX() - currentPose.getX();
    double yError = targetPose2d.getY() - currentPose.getY();
    double rotationError =
        targetPose2d
            .getRotation()
            .minus(currentPose.getRotation())
            .minus(Rotation2d.kPi)
            .getRadians();

    // Compute speed commands
    double xSpeed = kP * xError;
    double ySpeed = kP * yError;
    double rotSpeed = kPRotation * rotationError;

    // Clamp speeds
    xSpeed =
        MathUtil.clamp(xSpeed, -getMaxLinearSpeedMetersPerSec(), getMaxLinearSpeedMetersPerSec());
    ySpeed =
        MathUtil.clamp(ySpeed, -getMaxLinearSpeedMetersPerSec(), getMaxLinearSpeedMetersPerSec());
    rotSpeed =
        MathUtil.clamp(rotSpeed, -getMaxAngularSpeedRadPerSec(), getMaxAngularSpeedRadPerSec());

    // Convert to field-relative speeds and drive
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getRotation());
    System.out.println("Commanded Speeds: " + speeds);

    runVelocity(speeds);
  }

  public void applyVisionMeasurementNow(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs,
      boolean forceApply) {
    odometryLock.lock();
    try {
      // If not initialized yet, allow initialization (respecting reliability
      // unless forceApply is true).
      if (!visionPoseInitialized) {
        double linearStd = visionMeasurementStdDevs.get(0, 0);
        double angularStd = visionMeasurementStdDevs.get(2, 0);
        if (!forceApply
            && (linearStd > visionInitMaxLinearStdDevMeters
                || angularStd > visionInitMaxAngularStdDevRadians)) {
          Logger.recordOutput("Odometry/VisionInitRejected/LinearStd", linearStd);
          Logger.recordOutput("Odometry/VisionInitRejected/AngularStd", angularStd);
          return;
        }

        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), visionRobotPoseMeters);
        visionPoseInitialized = true;
        Logger.recordOutput("Odometry/VisionInitializedManual", true);
        return;
      }

      // Already initialized: either enforce proximity checks or apply directly
      if (!forceApply) {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        double dist =
            currentPose.getTranslation().getDistance(visionRobotPoseMeters.getTranslation());
        double angleDiff =
            Math.abs(
                currentPose.getRotation().minus(visionRobotPoseMeters.getRotation()).getRadians());
        if (dist > maxVisionPoseDistanceMeters || angleDiff > maxVisionPoseAngleRadians) {
          Logger.recordOutput("Odometry/VisionRejectedManual/Distance", dist);
          Logger.recordOutput("Odometry/VisionRejectedManual/Angle", angleDiff);
          return;
        }
      }

      // Apply as a low-gain correction (scale std devs to reduce influence)
      Matrix<N3, N1> scaledStdDevs = visionMeasurementStdDevs.times(visionStdDevScale);
      poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, scaledStdDevs);
      Logger.recordOutput("Odometry/VisionAppliedManually", true);
    } finally {
      odometryLock.unlock();
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}
