package frc.robot.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClawIOSim implements ClawIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim intakeSim;
  private final DCMotorSim shooterSim;
  private double IntakeAppliedVolts = 0.0;
  private double ShooterAppliedVolts = 0.0;

  public ClawIOSim() {
    intakeSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
    shooterSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {

    // Update simulation state
    intakeSim.setInputVoltage(MathUtil.clamp(IntakeAppliedVolts, -12.0, 12.0));
    intakeSim.update(0.02);
    shooterSim.setInputVoltage(MathUtil.clamp(ShooterAppliedVolts, -12.0, 12.0));
    shooterSim.update(0.02);

    // Update intake inputs
    inputs.IntakePositionRad = intakeSim.getAngularPositionRad();
    inputs.IntakeVelocityRadPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.IntakeAppliedVolts = intakeSim.getInputVoltage();
    inputs.IntakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());
    inputs.ShooterPositionRad = shooterSim.getAngularPositionRad();
    inputs.ShooterVelocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
    inputs.ShooterAppliedVolts = shooterSim.getInputVoltage();
    inputs.ShooterCurrentAmps = Math.abs(shooterSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double intakeOutput, double shooterOutput) {
    IntakeAppliedVolts = intakeOutput * 12.0;
    ShooterAppliedVolts = shooterOutput * 12.0;
  }

  @Override
  public void setIntakeVoltage(double volts) {
    IntakeAppliedVolts = volts;
  }

  @Override
  public void setShooterVoltage(double volts) {
    ShooterAppliedVolts = volts;
  }
}
