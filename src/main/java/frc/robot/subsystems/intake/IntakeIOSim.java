package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.CheetahUtil;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim indexerIOSim;
  private final DCMotorSim intakeIOSim;
  private final DCMotorSim armIOSim;
  private double IndexerAppliedVolts = 0.0;
  private double IntakeAppliedVolts = 0.0;
  private double ArmAppliedVolts = 0.0;

  public IntakeIOSim() {
    indexerIOSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
    intakeIOSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
    armIOSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Update simulation state
    indexerIOSim.setInputVoltage(MathUtil.clamp(IndexerAppliedVolts, -12.0, 12.0));
    indexerIOSim.update(0.02);
    intakeIOSim.setInputVoltage(MathUtil.clamp(IntakeAppliedVolts, -12.0, 12.0));
    intakeIOSim.update(0.02);
    armIOSim.setInputVoltage(MathUtil.clamp(ArmAppliedVolts, -12.0, 12.0));
    armIOSim.update(0.02);
    // Update indexer inputs
    inputs.IndexerPositionRad = indexerIOSim.getAngularPositionRad();
    inputs.IndexerVelocityRadPerSec = indexerIOSim.getAngularVelocityRadPerSec();
    inputs.IndexerAppliedVolts = IndexerAppliedVolts;
    inputs.IndexerCurrentAmps = Math.abs(indexerIOSim.getCurrentDrawAmps());

    // Update intake inputs
    inputs.IntakePositionRad = intakeIOSim.getAngularPositionRad();
    inputs.IntakeVelocityRadPerSec = intakeIOSim.getAngularVelocityRadPerSec();
    inputs.IntakeAppliedVolts = IntakeAppliedVolts;
    inputs.IntakeCurrentAmps = Math.abs(intakeIOSim.getCurrentDrawAmps());

    // Update arm inputs
    inputs.ArmPositionDeg =
        CheetahUtil.intakeArmRotationToDegrees(armIOSim.getAngularPositionRotations());
    inputs.ArmVelocityRadPerSec = armIOSim.getAngularVelocityRadPerSec();
    inputs.ArmAppliedVolts = ArmAppliedVolts;
    inputs.ArmCurrentAmps = Math.abs(armIOSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double indexerOutput, double intakeOutput, double armOutput) {
    IndexerAppliedVolts = indexerOutput * 12.0;
    IntakeAppliedVolts = intakeOutput * 12.0;
    ArmAppliedVolts = armOutput * 12.0;
  }
}
