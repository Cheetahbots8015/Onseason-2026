package frc.robot.subsystems.barrel;

import org.littletonrobotics.junction.AutoLog;

public interface BarrelIO {
  @AutoLog
  public static class BarrelIOInputs {
    public double BarrelPositionRad = 0.0;
    public double BarrelVelocityRadPerSec = 0.0;
    public double BarrelAppliedVolts = 0.0;
    public double BarrelCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(BarrelIOInputs inputs) {}

  public default void setBarrelVoltage(double volts) {}

  public default void stop() {}
}
