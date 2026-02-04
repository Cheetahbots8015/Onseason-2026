package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double ClimberPositionDeg = 0.0;
    public double ClimberVelocityRadPerSec = 0.0;  
    public double ClimberAppliedVolts = 0.0;
    public double ClimberCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the roller at the specified open loop value. */
  public default void setOpenLoop(double clawOutput) {}

  public default void setClimberVoltage(double volts) {}

  public default void setClimberPosition(double degree) {}

  public default void resetClimberPosition() {}

  public default void stopClimber() {}
}
