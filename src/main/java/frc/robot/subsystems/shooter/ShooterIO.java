package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double ShooterPositionRad = 0.0;
    public double ShooterVelocityRotPerSec = 0.0;
    public double ShooterAppliedVolts = 0.0;
    public double ShooterCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVoltage(double volts) {}

  public default void ShooterVelocityVoltage(double velocity) {}
}
