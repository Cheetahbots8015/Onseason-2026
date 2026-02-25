package frc.robot.subsystems.shooter;

public interface ShooterIO {
  public static class ShooterIOInputs {
    // Left motor
    public double leftPositionRad = 0.0;
    public double leftVelocityRotPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;

    // Right / follower motor
    public double rightPositionRad = 0.0;
    public double rightVelocityRotPerSec = 0.0;
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
}
