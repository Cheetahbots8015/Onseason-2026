package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double HoodPositionDeg = 0.0;
    public double HoodVelocityRadPerSec = 0.0;
    public double HoodAppliedVolts = 0.0;
    public double HoodCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodVoltage(double volts) {}

  public default void setHoodToDegrees(double degrees) {}
}
