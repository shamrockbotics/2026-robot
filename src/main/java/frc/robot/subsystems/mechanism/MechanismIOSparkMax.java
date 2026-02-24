package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class MechanismIOSparkMax implements MechanismIO {
  // Hardware objects
  private final SparkMax spark;
  private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController controller;
  private ArmFeedforward armFeedforward = null;
  private ElevatorFeedforward elevatorFeedforward = null;
  private DoubleSupplier positionOffsetSupplier = () -> 0;
  private double setpoint = 0.0;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  // Private variables
  private double maxVoltage = 12.0;

  /* Mechanism with a zero offset indicates absolute encoder */
  public MechanismIOSparkMax(
      int id,
      double zeroOffset,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double voltageLimit,
      double kP,
      double kD) {
    maxVoltage = voltageLimit;
    spark = new SparkMax(id, MotorType.kBrushless);
    absoluteEncoder = spark.getAbsoluteEncoder();
    relativeEncoder = null;
    controller = spark.getClosedLoopController();

    double scaledZeroOffset = zeroOffset / encoderPositionFactor;
    if (scaledZeroOffset < 0) scaledZeroOffset += 1;

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage);
    sparkConfig
        .absoluteEncoder
        .inverted(encoderInverted)
        .zeroCentered(true)
        .zeroOffset(scaledZeroOffset)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .averageDepth(2);
    sparkConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(kP, 0.0, kD);
    sparkConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /* Mechanism with no zero offset indicates relative encoder */
  public MechanismIOSparkMax(
      int id,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double voltageLimit,
      double kP,
      double kD) {
    maxVoltage = voltageLimit;
    spark = new SparkMax(id, MotorType.kBrushless);
    absoluteEncoder = null;
    relativeEncoder = spark.getAlternateEncoder();
    controller = spark.getClosedLoopController();

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage);
    sparkConfig
        .alternateEncoder
        .inverted(encoderInverted)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .averageDepth(2)
        .setSparkMaxDataPortConfig();
    sparkConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(kP, 0.0, kD);
    sparkConfig
        .signals
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderPosition(20)
        .externalOrAltEncoderVelocityAlwaysOn(true)
        .externalOrAltEncoderVelocity(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  // Add a follower to the existing IO
  public MechanismIOSparkMax addFollower(int id, boolean inverted) {
    SparkMax followerSpark = new SparkMax(id, MotorType.kBrushless);
    SparkMaxConfig followerSparkConfig = new SparkMaxConfig();
    followerSparkConfig.idleMode(IdleMode.kBrake).follow(spark, inverted);
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerSparkConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters));
    return this;
  }

  public MechanismIOSparkMax addFeedforward(ArmFeedforward feedforward) {
    this.armFeedforward = feedforward;
    return this;
  }

  public MechanismIOSparkMax addFeedforward(ElevatorFeedforward feedforward) {
    this.elevatorFeedforward = feedforward;
    return this;
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    // Update inputs
    sparkStickyFault = false;
    if (absoluteEncoder != null) {
      ifOk(spark, absoluteEncoder::getPosition, (value) -> inputs.currentPosition = value);
      ifOk(spark, absoluteEncoder::getVelocity, (value) -> inputs.velocity = value);
    } else if (relativeEncoder != null) {
      if (relativeEncoder.getPosition() < 0.0) relativeEncoder.setPosition(0.0);
      ifOk(spark, relativeEncoder::getPosition, (value) -> inputs.currentPosition = value);
      ifOk(spark, relativeEncoder::getVelocity, (value) -> inputs.velocity = value);
    }
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getAppliedOutput, (value) -> inputs.appliedOutput = value);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.targetPosition = setpoint;
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(double position) {
    setpoint = position;
    controller.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedforward(setpoint));
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }

  private double calculateFeedforward(double setpoint) {
    double feedforwardValue = 0.0;

    if (armFeedforward != null && absoluteEncoder != null) {
      feedforwardValue =
          armFeedforward.calculate(setpoint + positionOffsetSupplier.getAsDouble(), 0);
    } else if (elevatorFeedforward != null && relativeEncoder != null) {
      feedforwardValue = elevatorFeedforward.calculate(0);
    }

    return feedforwardValue;
  }

  public void setPositionOffset(DoubleSupplier supplier) {
    positionOffsetSupplier = supplier;
  }
}
