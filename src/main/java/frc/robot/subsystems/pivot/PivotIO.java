package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double PositionRad = 0.0;
    public double VelocityRadPerSec = 0.0;
    public double AppliedVolts = 0.0;
    public double CurrentAmps = 0.0;
    public double PivotPositionDegree = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPivotVoltage(double volts) {}

  public default void setPosition(double position) {}
}
