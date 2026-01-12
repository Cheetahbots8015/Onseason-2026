package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double ClawPositionRad = 0.0;
    public double ClawVelocityRadPerSec = 0.0;
    public double ClawAppliedVolts = 0.0;
    public double ClawCurrentAmps = 0.0;
    public double PivotPositionDeg = 0.0;
    public double PivotVelocityRadPerSec = 0.0;
    public double PivotAppliedVolts = 0.0;
    public double PivotCurrentAmps = 0.0;
    public double lightTrigger2 = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the climber at the specified open loop value. */
  public default void setOpenLoop(double rollerOutput, double climberOutput) {}

  public default void setClawVoltage(double volts) {}
  ;

  public default void setPivotVoltage(double volts) {}
  ;

  public default void setClimberPivotFinalPosition(double degeree) {}

  public default void setClimberPivotDefaultPosition(double degeree) {}
}
