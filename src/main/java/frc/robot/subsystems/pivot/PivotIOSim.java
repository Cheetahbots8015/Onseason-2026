package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim pivotIOSim;
  private double AppliedVolts = 0.0;

  public PivotIOSim() {
    pivotIOSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {

    // Update simulation state
    pivotIOSim.setInputVoltage(MathUtil.clamp(AppliedVolts, -12.0, 12.0));
    pivotIOSim.update(0.02);

    // Update roller inputs
    inputs.PositionRad = pivotIOSim.getAngularPositionRad();
    inputs.VelocityRadPerSec = pivotIOSim.getAngularVelocityRadPerSec();
    inputs.AppliedVolts = AppliedVolts;
    inputs.CurrentAmps = Math.abs(pivotIOSim.getCurrentDrawAmps());
  }

  @Override
  public void setPivotVoltage(double output) {
    AppliedVolts = output;
  }

  @Override
  public void setPosition(double position) {
    pivotIOSim.setAngle(position);
  }
}
