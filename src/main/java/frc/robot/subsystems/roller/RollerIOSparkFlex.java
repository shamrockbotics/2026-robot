package frc.robot.subsystems.roller;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class RollerIOSparkFlex implements RollerIO {

  // Hardware objects
  private final SparkFlex spark;
  private final AbsoluteEncoder encoder;

  // Closed loop controllers
  private final SparkClosedLoopController controller;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public RollerIOSparkFlex(
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
    SparkFlex followerSpark = new SparkFlex(id2, MotorType.kBrushless);
    SparkFlexConfig followerSparkConfig = new SparkFlexConfig();
    followerSparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage)
        .follow(spark, true);
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerSparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public RollerIOSparkFlex(
      int id,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double velocityKp,
      double velocityKd) {
    spark = new SparkFlex(id, MotorType.kBrushless);
    encoder = spark.getAbsoluteEncoder();
    controller = spark.getClosedLoopController();

    SparkFlexConfig sparkConfig = new SparkFlexConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kCoast)
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
        .pid(velocityKp, 0.0, velocityKd);
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

  @Override
  public void setVelocity(double metersPerSec) {
    controller.setSetpoint(metersPerSec, ControlType.kVelocity);
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }
}
