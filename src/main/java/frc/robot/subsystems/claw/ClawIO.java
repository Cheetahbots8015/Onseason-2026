package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public double IntakePositionRad = 0.0;
    public double IntakeVelocityRadPerSec = 0.0;
    public double IntakeAppliedVolts = 0.0;
    public double IntakeCurrentAmps = 0.0;
    public double ShooterPositionRad = 0.0;
    public double ShooterVelocityRadPerSec = 0.0;
    public double ShooterAppliedVolts = 0.0;
    public double ShooterCurrentAmps = 0.0;
    public double lightTrigger = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  /** Run the roller at the specified open loop value. */
  public default void setOpenLoop(double intakeOutput, double shooterOutput) {}

  public default void setIntakeVoltage(double volts) {}

  public default void setShooterVoltage(double volts) {}

  public default void IntakeVelocityVoltage(double velocity) {}

  public default void ClawIntakeTorqueCurrent(double current) {}
}
