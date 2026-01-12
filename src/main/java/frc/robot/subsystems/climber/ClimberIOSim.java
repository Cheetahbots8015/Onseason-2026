package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim rollerSim;
  private final DCMotorSim climberSim;
  private double ClawAppliedVolts = 0.0;
  private double PivotAppliedVolts = 0.0;

  public ClimberIOSim() {
    rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
    climberSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    // Update simulation state
    rollerSim.setInputVoltage(MathUtil.clamp(ClawAppliedVolts, -12.0, 12.0));
    rollerSim.update(0.02);
    climberSim.setInputVoltage(MathUtil.clamp(PivotAppliedVolts, -12.0, 12.0));
    climberSim.update(0.02);

    // Update roller inputs
    inputs.ClawPositionRad = rollerSim.getAngularPositionRad();
    inputs.ClawVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.ClawAppliedVolts = ClawAppliedVolts;
    inputs.ClawCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    inputs.PivotPositionDeg = climberSim.getAngularPositionRad();
    inputs.PivotVelocityRadPerSec = climberSim.getAngularVelocityRadPerSec();
    inputs.PivotAppliedVolts = PivotAppliedVolts;
    inputs.PivotCurrentAmps = Math.abs(climberSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double clawOutput, double pivotOutput) {
    ClawAppliedVolts = clawOutput * 12.0;
    PivotAppliedVolts = pivotOutput * 12.0;
  }
}
