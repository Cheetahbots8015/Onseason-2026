package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSim;
  private double HoodAppliedVolts = 0.0;

  public HoodIOSim() {
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    // Update simulation state
    motorSim.setInputVoltage(MathUtil.clamp(HoodAppliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    // Update motor inputs
    inputs.HoodPositionDeg = Math.toDegrees(motorSim.getAngularPositionRad());
    inputs.HoodVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.HoodAppliedVolts = motorSim.getInputVoltage();
    inputs.HoodCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setHoodVoltage(double volts) {
    HoodAppliedVolts = volts;
  }
}
