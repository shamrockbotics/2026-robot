package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim motorSim;

  private DCMotor gearbox = DCMotor.getNEO(1);
  private PIDController controller = new PIDController(50, 0, 0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  public RollerIOSim(double gearReduction, double radsPerPulse) {
    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 1, gearReduction), gearbox);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {

    if (closedLoop) {
      appliedVolts = controller.calculate(motorSim.getAngularPositionRad());
    } else {
      controller.reset();
      controller.setSetpoint(motorSim.getAngularPositionRad());
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    inputs.connected = true;
    inputs.velocityMetersPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double metersPerSec) {
    closedLoop = true;
    controller.setSetpoint(metersPerSec);
  }

  @Override
  public void setOutput(double output) {
    closedLoop = false;
    appliedVolts = output * maxVoltage;
  }
}
