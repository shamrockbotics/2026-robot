package frc.robot.subsystems.roller;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class RollerIOSparkMax implements RollerIO {

  // Hardware objects
  private final SparkMax spark;
  private final AbsoluteEncoder encoder;

  // Closed loop controllers
  private final SparkClosedLoopController controller;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public RollerIOSparkMax(
      int id1,
      int id2,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double velocityKp,
      double velocityKd) {
    this(
        id1,
        motorInverted,
        encoderInverted,
        encoderPositionFactor,
        encoderVelocityFactor,
        currentLimit,
        velocityKp,
        velocityKd);
    SparkMax followerSpark = new SparkMax(id2, MotorType.kBrushless);
    SparkMaxConfig followerSparkConfig = new SparkMaxConfig();
    followerSparkConfig
        .inverted(!motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage)
        .follow(spark);
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerSparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public RollerIOSparkMax(
      int id,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double velocityKp,
      double velocityKd) {
    spark = new SparkMax(id, MotorType.kBrushless);
    encoder = spark.getAbsoluteEncoder();
    controller = spark.getClosedLoopController();

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage);
    sparkConfig
        .absoluteEncoder
        .inverted(encoderInverted)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .averageDepth(2);
    sparkConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2 * Math.PI)
        .pidf(velocityKp, 0.0, velocityKd, 0.0);
    sparkConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
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

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    // Update inputs
    sparkStickyFault = false;
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  public RollerIOSparkMax addFollower(int id, boolean inverted) {
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

  @Override
  public void setVelocity(double metersPerSec) {
    controller.setReference(metersPerSec, ControlType.kVelocity);
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }
}
