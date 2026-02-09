package frc.robot.subsystems.barrel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class BarrelIOSim implements BarrelIO {
  // Simulation constants
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double GEARING = 50.0;
  private static final double J_INERTIA = 0.01;

  private final DCMotorSim barrelSim;
  private double barrelAppliedVolts = 0.0;

  public BarrelIOSim() {
    // Initialize motor simulation model
    barrelSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, J_INERTIA, GEARING), GEARBOX);
  }

  @Override
  public void updateInputs(BarrelIOInputs inputs) {
    // Update simulation state (20ms loop)
    barrelSim.setInputVoltage(MathUtil.clamp(barrelAppliedVolts, -12.0, 12.0));
    barrelSim.update(0.020);

    // Update loggable inputs
    inputs.BarrelPositionRad = barrelSim.getAngularPositionRad();
    inputs.BarrelVelocityRotPerSec =
        Units.radiansToRotations(barrelSim.getAngularVelocityRadPerSec());
    inputs.BarrelAppliedVolts = barrelAppliedVolts;
    inputs.BarrelCurrentAmps = Math.abs(barrelSim.getCurrentDrawAmps());
  }

  @Override
  public void setBarrelVoltage(double volts) {
    // Set demand voltage
    barrelAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    barrelAppliedVolts = 0.0;
  }
}
