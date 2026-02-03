package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double ClawPositionRad = 0.0;
    public double ClawVelocityRadPerSec = 0.0;
    public double ClawAppliedVolts = 0.0;
    public double ClawCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the roller at the specified open loop value. */
  public default void setOpenLoop(double clawOutput) {}

  public default void setClawVoltage(double volts) {}

  public default void ClawVelocityVoltage(double velocity) {}

  public default void ClawTorqueCurrent(double current) {}

  public default void clawMotionMagic(double rotation) {}

  public default void resetClawPosition() {}

  public default void stopClaw() {}
}
