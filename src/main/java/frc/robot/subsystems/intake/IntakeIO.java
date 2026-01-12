package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double IntakePositionRad = 0.0;
    public double IntakeVelocityRadPerSec = 0.0;
    public double IntakeAppliedVolts = 0.0;
    public double IntakeCurrentAmps = 0.0;

    public double IndexerPositionRad = 0.0;
    public double IndexerVelocityRadPerSec = 0.0;
    public double IndexerAppliedVolts = 0.0;
    public double IndexerCurrentAmps = 0.0;

    public double ArmPositionDeg = 0.0;
    public double ArmVelocityRadPerSec = 0.0;
    public double ArmAppliedVolts = 0.0;
    public double ArmCurrentAmps = 0.0;

    public boolean Canrange = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the roller at the specified open loop value. */
  public default void setOpenLoop(double indexerOutput, double intakeOutput, double armOutput) {}

  public default void setArmVoltage(double volts) {}

  public default void setIntakeVoltage(double volts) {}

  public default void setIndexerVoltage(double volts) {}

  public default boolean getCanRange() {
    return false;
  }

  public default void setArmToDegrees(double degrees) {}

  public default void armVelocityVoltage(double velocity) {}

  public default void resetArmPosition() {}
}
