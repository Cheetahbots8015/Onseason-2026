package frc.robot.subsystems.barrel;

import org.littletonrobotics.junction.AutoLog;

public interface BarrelIO {
  @AutoLog
  public static class BarrelIOInputs {
    public double BarrelPositionRad = 0.0;
    public double BarrelVelocityRotPerSec = 0.0;
    public double BarrelAppliedVolts = 0.0;
    public double BarrelCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(BarrelIOInputs inputs) {}

  public default void setBarrelVoltage(double volts) {}

  public default void setBarrelVelocity(double velocity) {}

  public default void stop() {}
}
