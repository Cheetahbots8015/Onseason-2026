package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSim;
  private double MotorAppliedVolts = 0.0;

  public ShooterIOSim() {
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    // Update simulation state
    motorSim.setInputVoltage(MathUtil.clamp(MotorAppliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    // Update motor inputs
    inputs.MotorPositionRad = motorSim.getAngularPositionRad();
    inputs.MotorVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.MotorAppliedVolts = motorSim.getInputVoltage();
    inputs.MotorCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double motorOutput) {
    MotorAppliedVolts = motorOutput * 12.0;
  }

  @Override
  public void setMotorVoltage(double volts) {
    MotorAppliedVolts = volts;
  }
}
