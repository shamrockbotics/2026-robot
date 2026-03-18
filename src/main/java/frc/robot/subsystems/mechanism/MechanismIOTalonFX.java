package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.function.DoubleSupplier;

public class MechanismIOTalonFX implements MechanismIO {

  private final TalonFX talon;
  private final DutyCycleEncoder encoder;

  private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private ArmFeedforward armFeedforward = null;
  private ElevatorFeedforward elevatorFeedforward = null;
  private DoubleSupplier positionOffsetSupplier = () -> 0.0;

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private double maxVoltage = 12.0;
  private double setpoint = 0.0;

  private final double encoderPositionFactor; 
  private final double zeroOffset;            
  private final boolean encoderInverted;

  /**
   * Creates a MechanismIOTalonFX using a duty-cycle encoder wired to the RIO.
   *
   * @param talonId           CAN ID of the TalonFX (Kraken)
   * @param encoderChannel    DIO channel on the RIO the encoder signal wire is connected to
   * @param zeroOffset        Raw encoder reading
   * @param motorInverted     Whether the motor output should be inverted
   * @param encoderInverted   Whether the encoder direction should be inverted
   * @param encoderPositionFactor  Converts encoder rotations → mechanism units (e.g. 2*PI for radians)
   * @param currentLimit      Stator current limit in amps
   * @param voltageLimit      Voltage Limit
   * @param kP                P Value
   * @param kD                D Value
   */
  public MechanismIOTalonFX(
      int talonId,
      int encoderChannel,
      double zeroOffset,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      int currentLimit,
      double voltageLimit,
      double kP,
      double kD) {

    this.maxVoltage = voltageLimit;
    this.encoderPositionFactor = encoderPositionFactor;
    this.zeroOffset = zeroOffset;
    this.encoderInverted = encoderInverted;

    talon = new TalonFX(talonId);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted =
        motorInverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;

    config.Voltage.PeakForwardVoltage = voltageLimit;
    config.Voltage.PeakReverseVoltage = -voltageLimit;

    config.Slot0.kP = kP;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = kD;

    talon.getConfigurator().apply(config);

    encoder = new DutyCycleEncoder(encoderChannel);
  }


  public MechanismIOTalonFX addFeedforward(ArmFeedforward feedforward) {
    this.armFeedforward = feedforward;
    return this;
  }

  public MechanismIOTalonFX addFeedforward(ElevatorFeedforward feedforward) {
    this.elevatorFeedforward = feedforward;
    return this;
  }

  public void setPositionOffset(DoubleSupplier supplier) {
    positionOffsetSupplier = supplier;
  }


  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    boolean encoderConnected = encoder.isConnected();
    inputs.connected = connectedDebounce.calculate(encoderConnected);

    if (encoderConnected) {
      inputs.currentPosition = getRawPositionMechanismUnits();
      inputs.velocity = 0.0;
    }

    inputs.appliedVolts =
        talon.getMotorVoltage().getValueAsDouble();
    inputs.appliedOutput =
        talon.getDutyCycle().getValueAsDouble();
    inputs.currentAmps =
        talon.getStatorCurrent().getValueAsDouble();

    inputs.targetPosition = setpoint;
  }

  @Override
  public void setPosition(double position) {
    setpoint = position;
    double feedforward = calculateFeedforward(position);

    syncTalonToEncoder();

    double talonSetpointRotations = mechanismUnitsToTalonRotations(position);
    talon.setControl(
        positionVoltage
            .withPosition(talonSetpointRotations)
            .withFeedForward(feedforward));
  }

  @Override
  public void setOutput(double value) {
    talon.setControl(voltageOut.withOutput(value * maxVoltage));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    talon.setControl(voltageOut.withOutput(voltage.in(Volts)));
  }

  private double getRawPositionMechanismUnits() {
    double raw = encoder.get(); // 0.0 – 1.0 rotations
    raw -= zeroOffset;

    if (raw > 0.5) raw -= 1.0;
    if (raw < -0.5) raw += 1.0;

    if (encoderInverted) raw = -raw;

    return raw * encoderPositionFactor;
  }

  private double mechanismUnitsToTalonRotations(double mechanismUnits) {
    return mechanismUnits / encoderPositionFactor;
  }

  private void syncTalonToEncoder() {
    double encoderRotations = getRawPositionMechanismUnits() / encoderPositionFactor;
    talon.setPosition(encoderRotations);
  }

  private double calculateFeedforward(double setpoint) {
    if (armFeedforward != null) {
      return armFeedforward.calculate(setpoint + positionOffsetSupplier.getAsDouble(), 0);
    } else if (elevatorFeedforward != null) {
      return elevatorFeedforward.calculate(0);
    }
    return 0.0;
  }
}