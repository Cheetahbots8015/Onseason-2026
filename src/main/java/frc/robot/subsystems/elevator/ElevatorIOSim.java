package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  private final DCMotorSim elevatorIOSim;
  private double AppliedVolts = 0.0;

  public ElevatorIOSim() {
    elevatorIOSim =
        new DCMotorSim(LinearSystemId.createElevatorSystem(GEARBOX, 800.0, 0.03, 6.12), GEARBOX);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    // Update simulation state
    elevatorIOSim.setInputVoltage(MathUtil.clamp(AppliedVolts, -12.0, 12.0));
    elevatorIOSim.update(0.02);

    // Update roller inputs
    inputs.PositionRot = elevatorIOSim.getAngularPositionRotations();
    inputs.VelocityRPS = elevatorIOSim.getAngularVelocityRPM() / 60;
    inputs.AppliedVolts = AppliedVolts;
    inputs.CurrentAmps = Math.abs(elevatorIOSim.getCurrentDrawAmps());
    inputs.AccelerationRPSS = elevatorIOSim.getAngularAccelerationRadPerSecSq();
  }

  @Override
  public void setElevatorVoltage(double volts) {
    AppliedVolts = volts;
  }

  @Override
  public void VelocityVoltage(double velocity) {
    AppliedVolts = 0.05;
  }

  @Override
  public void setPosition(double position) {
    elevatorIOSim.setAngle(position);
  }
}
