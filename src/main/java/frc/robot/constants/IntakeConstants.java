package frc.robot.constants;

public class IntakeConstants {
  public static final int flywheelID = 26;
  public static final int armID = 25;
  public static final int sensorID = 27;

  public static final boolean flywheel_neutralmode_Coast = true;
  public static final boolean flywheel_inverted_CounterClockwisePositive = false;
  public static final boolean arm_neutralmode_Coast = false;
  public static final boolean arm_inverted_CounterClockwisePositive = false;

  public static final double flywheelVolts = 0.0;
  public static final double armVolts = 0.0;

  public static final double dutyCycleDeadband = 0.05;
  public static final double statusUpdateFrequency = 50.0;
  public static final double flywheelkP = 0.05;
  public static final double flywheelkI = 0.0;
  public static final double flywheelkD = 0.0;
  public static final double flywheelkA = 0.0;
  public static final double flywheelkS = 0.12419;
  public static final double flywheelkV = 0.10696;

  public static final double armkP = 0.7;
  public static final double armkI = 0.0;
  public static final double armkD = 0.0;
  public static final double armkA = 0.01;
  public static final double armkS = 0.0;
  public static final double armkV = 0.3;
  public static final double armkG = 0.0;

  public static final double armMotionMagicCruiseVelocity = 1.5;
  public static final double armMotionMagicAcceleration = 3.0;

  public static final double armDeployPosition = 0.0;
  public static final double armRetractPosition = 0.0;
}
