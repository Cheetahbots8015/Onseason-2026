package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // Right / follower motor
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Updates the motor outputs with LQR. */
  public default void updateOutputs(ShooterIOInputs inputs, double targetVelocity) {}

  /** Direct set motor voltage - single motor (applied to both/follower). */
  public default void setMotorVoltage(double volts) {}

  /** Velocity control - single argument follower. */
  public default void setVelocityControl(double rotPerSec) {}

  public default void idle(ShooterIOInputs inputs) {}

  public default void resetFilter() {}

  public default void stop() {}
}
