package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim leftSim;
  private final DCMotorSim rightSim;

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  public ShooterIOSim() {
    leftSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
    rightSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    leftSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
    rightSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.leftPositionRad = leftSim.getAngularPositionRad();
    inputs.leftVelocityRotPerSec = Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec());
    inputs.leftAppliedVolts = leftSim.getInputVoltage();
    inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());

    inputs.rightPositionRad = rightSim.getAngularPositionRad();
    inputs.rightVelocityRotPerSec =
        Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec());
    inputs.rightAppliedVolts = rightSim.getInputVoltage();
    inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
  }

  @Override
  public void setMotorVoltage(double leftVolts, double rightVolts) {
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  @Override
  public void setMotorVoltage(double volts) {
    leftAppliedVolts = volts;
    rightAppliedVolts = volts;
  }

  @Override
  public void setVelocityControl(double leftRadPerSec, double rightRadPerSec) {
    leftAppliedVolts = leftRadPerSec * 0.01;
    rightAppliedVolts = rightRadPerSec * 0.01;
  }

  @Override
  public void setVelocityControl(double radPerSec) {
    leftAppliedVolts = radPerSec * 0.01;
    rightAppliedVolts = radPerSec * 0.01;
  }
}
