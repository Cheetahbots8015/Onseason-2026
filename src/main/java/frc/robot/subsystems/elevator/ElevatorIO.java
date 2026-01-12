package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double PositionRot = 0.0;
    public double VelocityRPS = 0.0;
    public double AppliedVolts = 0.0;
    public double CurrentAmps = 0.0;
    public double ElevatorHeightMeters = 0.0;
    public double AccelerationRPSS = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorVoltage(double volts) {}

  public default void VelocityVoltage(double velocity) {}

  public default void setPosition(double position) {}

  public default void resetElevetorPosition() {}
}
