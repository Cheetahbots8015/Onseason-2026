package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim clawSim;
  private double ClawAppliedVolts = 0.0;

  public ClimberIOSim() {
    clawSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    // Update simulation state
    clawSim.setInputVoltage(MathUtil.clamp(ClawAppliedVolts, -12.0, 12.0));
    clawSim.update(0.02);

    // Update motor inputs
    inputs.ClawPositionRad = clawSim.getAngularPositionRad();
    inputs.ClawVelocityRadPerSec = clawSim.getAngularVelocityRadPerSec();
    inputs.ClawAppliedVolts = clawSim.getInputVoltage();
    inputs.ClawCurrentAmps = Math.abs(clawSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double clawOutput) {
    ClawAppliedVolts = clawOutput * 12.0;
  }

  @Override
  public void setClawVoltage(double volts) {
    ClawAppliedVolts = volts;
  }
}
