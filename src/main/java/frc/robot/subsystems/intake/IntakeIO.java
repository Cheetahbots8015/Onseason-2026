package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double FlywheelPositionRad = 0.0;
    public double FlywheelVelocityRadPerSec = 0.0;
    public double FlywheelAppliedVolts = 0.0;
    public double FlywheelCurrentAmps = 0.0;

    public double ArmPositionRad = 0.0;
    public double ArmVelocityRadPerSec = 0.0;
    public double ArmAppliedVolts = 0.0;
    public double ArmCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the roller at the specified open loop value. */
  public default void setOpenLoop(double flywheelOutput, double armOutput) {}

  public default void setFlywheelVoltage(double volts) {}

  public default void setArmVoltage(double volts) {}

  public default void armMotionMagic(double targetPosition) {}

  public default void flywheelVelocityVoltage(double targetVelocity) {}

  public default void flywheelStop() {}

  public default void resetArmPosition() {}
}
