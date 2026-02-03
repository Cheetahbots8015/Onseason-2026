package frc.robot.subsystems.shooter;

public interface ShooterIO {
  public static class ShooterIOInputs {
    // Left motor
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;

    // Right / follower motor
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Direct set motor voltage - two motor overload. */
  public default void setMotorVoltage(double leftVolts, double rightVolts) {}

  /** Direct set motor voltage - single motor (applied to both/follower). */
  public default void setMotorVoltage(double volts) {}

  /** Velocity control using VelocityVoltage / MotionMagic style - two motor overload. */
  public default void setVelocityControl(double leftRadPerSec, double rightRadPerSec) {}

  /** Velocity control - single argument follower. */
  public default void setVelocityControl(double radPerSec) {}
}
