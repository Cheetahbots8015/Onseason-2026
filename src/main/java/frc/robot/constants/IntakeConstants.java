package frc.robot.constants;

public class IntakeConstants {
  public static final int flywheelID = 37;
  public static final int armID = 38;

  public static final boolean flywheel_neutralmode_Coast = true;
  public static final boolean flywheel_inverted_CounterClockwisePositive = true;
  public static final boolean arm_neutralmode_Coast = true;
  public static final boolean arm_inverted_CounterClockwisePositive = true;

  public static final double flywheelVolts = 0.0;
  public static final double armVolts = 0.0;

  public static final double dutyCycleDeadband = 0.05;
  public static final double statusUpdateFrequency = 50.0;
  public static final double flywheelkP = 0.1;
  public static final double flywheelkI = 0.0;
  public static final double flywheelkD = 0.0;
  public static final double flywheelkA = 0.0;
  public static final double flywheelkS = 0.5;
  public static final double flywheelkV = 0.0;
  public static final double armkP = 0.1;
  public static final double armkI = 0.0;
  public static final double armkD = 0.0;
  public static final double armkA = 0.0;
  public static final double armkS = 0.5;
  public static final double armkV = 0.0;
  public static final double armkG = 0.5;

  public static final double flywheelMotionMagicCruiseVelocity = 10.0;
  public static final double flywheelMotionMagicAcceleration = 50.0;
  public static final double armMotionMagicCruiseVelocity = 1.5;
  public static final double armMotionMagicAcceleration = 3.0;
}
