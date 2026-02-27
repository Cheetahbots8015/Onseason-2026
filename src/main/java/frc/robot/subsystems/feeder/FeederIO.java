package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double FeederPositionRad = 0.0;
    public double FeederVelocityRadPerSec = 0.0;
    public double FeederAppliedVolts = 0.0;
    public double FeederCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFeederVoltage(double volts) {}

  public default void updateOutputs(FeederIOInputs inputs, double targetVelocity) {}

  public default void idle(FeederIOInputs inputs) {}
}
