package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double MotorPositionRad = 0.0;
    public double MotorVelocityRadPerSec = 0.0;
    public double MotorAppliedVolts = 0.0;
    public double MotorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the roller at the specified open loop value. */
  public default void setOpenLoop(double motorOutput) {}

  public default void setMotorVoltage(double volts) {}

  public default void MotorVelocityVoltage(double velocity) {}

  public default void MotorTorqueCurrent(double current) {}
}
