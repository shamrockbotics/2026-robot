package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation using Spark MAX for both drive and turn motors, and a CTRE CANCoder for
 * the turn angle.
 */
public class ModuleIOSparkCANCoder implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final CANcoder turnCANCoder;

  // Closed-loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queues for odometry
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private boolean sparkStickyFault = false;

  // CANCoder configuration
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  public ModuleIOSparkCANCoder(int module, int turnEncoderCanId) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    // Spark motors
    driveSpark = new SparkFlex(getDriveCanId(module), MotorType.kBrushless);
    turnSpark = new SparkFlex(getTurnCanId(module), MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure CANCoder
    turnCANCoder = new CANcoder(turnEncoderCanId);
    cancoderConfig.MagnetSensor.MagnetOffset = getTurnOffset(module);
    cancoderConfig.MagnetSensor.SensorDirection =
        turnEncoderInverted
            ? com.ctre.phoenix6.signals.SensorDirectionValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;
    turnCANCoder.getConfigurator().apply(cancoderConfig);

    // Configure motors
    configureDriveMotor();
    configureTurnMotor();

    // Odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(turnSpark, () -> turnCANCoder.getAbsolutePosition().getValueAsDouble());
  }

  private int getDriveCanId(int module) {
    return switch (module) {
      case 0 -> frontLeftDriveCanId;
      case 1 -> frontRightDriveCanId;
      case 2 -> backLeftDriveCanId;
      case 3 -> backRightDriveCanId;
      default -> 0;
    };
  }

  private int getTurnCanId(int module) {
    return switch (module) {
      case 0 -> frontLeftTurnCanId;
      case 1 -> frontRightTurnCanId;
      case 2 -> backLeftTurnCanId;
      case 3 -> backRightTurnCanId;
      default -> 0;
    };
  }

  private double getTurnOffset(int module) {
    return switch (module) {
      case 0 -> frontLeftZeroRotation.getDegrees();
      case 1 -> frontRightZeroRotation.getDegrees();
      case 2 -> backLeftZeroRotation.getDegrees();
      case 3 -> backRightZeroRotation.getDegrees();
      default -> 0.0;
    };
  }

  private void configureDriveMotor() {
    var driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));
  }

  private void configureTurnMotor() {
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Drive inputs
    ifOk(driveSpark, driveEncoder::getPosition, val -> inputs.drivePositionRad = val);
    ifOk(driveSpark, driveEncoder::getVelocity, val -> inputs.driveVelocityRadPerSec = val);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        vals -> inputs.driveAppliedVolts = vals[0] * vals[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, val -> inputs.driveCurrentAmps = val);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Turn inputs from CANCoder
    inputs.turnPosition =
        Rotation2d.fromRotations(turnCANCoder.getAbsolutePosition().getValueAsDouble());
    inputs.turnVelocityRadPerSec = 0; // Optional: can implement if needed
    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();
    inputs.turnConnected = turnConnectedDebounce.calculate(true);

    // Odometry
    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(d -> d).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble(d -> d).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(d -> Rotation2d.fromRotations(d)).toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        com.revrobotics.spark.SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(rotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }
}
