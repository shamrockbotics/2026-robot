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

/** Spark drive motor + Spark turn motor CTRE CANcoder used ONLY for absolute reference */
public class ModuleIOSparkCANCoder implements ModuleIO {

  private final Rotation2d zeroRotation;

  private final SparkBase driveSpark;
  private final SparkBase turnSpark;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final CANcoder turnCANCoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOSparkCANCoder(int module, int turnEncoderCanId) {

    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    driveSpark = new SparkFlex(getDriveCanId(module), MotorType.kBrushless);
    turnSpark = new SparkFlex(getTurnCanId(module), MotorType.kBrushless);

    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();

    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    turnCANCoder = new CANcoder(turnEncoderCanId);
    CANcoderConfiguration cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.MagnetOffset = 0.0; // zeroRotation.getDegrees();

    turnCANCoder.getConfigurator().apply(cfg);

    configureDriveMotor();
    configureTurnMotor();
    resetTurnToAbsolute();

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);

    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  private void configureDriveMotor() {
    var cfg = new SparkFlexConfig();
    cfg.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor);

    cfg.closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);

    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    driveEncoder.setPosition(0.0);
  }

  private void configureTurnMotor() {
    var cfg = new SparkMaxConfig();
    cfg.inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(turnEncoderPositionFactor / (150 / 7))
        .velocityConversionFactor(turnEncoderVelocityFactor / (150 / 7));

    cfg.closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);

    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  private void resetTurnToAbsolute() {
    double absRad =
        Math.toRadians(turnCANCoder.getAbsolutePosition().getValueAsDouble() * 360.0)
            - zeroRotation.getRadians();

    turnEncoder.setPosition(MathUtil.inputModulus(absRad, turnPIDMinInput, turnPIDMaxInput));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    resetTurnToAbsolute();

    ifOk(driveSpark, driveEncoder::getPosition, val -> inputs.drivePositionRad = val);

    ifOk(driveSpark, driveEncoder::getVelocity, val -> inputs.driveVelocityRadPerSec = val);

    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        v -> inputs.driveAppliedVolts = v[0] * v[1]);

    inputs.driveConnected = driveConnectedDebounce.calculate(true);

    inputs.turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition());

    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();

    inputs.turnConnected = turnConnectedDebounce.calculate(true);

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(d -> d).toArray();

    inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble(d -> d).toArray();

    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveSpark.setVoltage(volts);
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnSpark.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ff = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;

    driveController.setSetpoint(
        velocityRadPerSec,
        SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ff,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(rotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
  }

  private int getDriveCanId(int module) {
    return switch (module) {
      case 3 -> frontLeftDriveCanId;
      case 2 -> frontRightDriveCanId;
      case 1 -> backLeftDriveCanId;
      case 0 -> backRightDriveCanId;
      default -> 0;
    };
  }

  private int getTurnCanId(int module) {
    return switch (module) {
      case 3 -> frontLeftTurnCanId;
      case 2 -> frontRightTurnCanId;
      case 1 -> backLeftTurnCanId;
      case 0 -> backRightTurnCanId;
      default -> 0;
    };
  }
}
